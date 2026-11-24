"""Planning queries for the policy: "can this arm reach this target / this route?" before committing a plan.

Not part of upstream Q0-Preview. The sandbox has no network, so the channel is files in the agent workspace:
the policy's Python (via the read-only helper /work/station_tools.py, written by `stage`) writes
/work/station_queries/<id>.json and waits for /work/station_answers/<id>.json. A thread in the episode launcher
(`QueryServer`) answers with the backend's own IK and path search (controller.motion.plan_path, the same cuRobo planner
and joint limits that accept or reject the policy's plans), starting from the arm's current joints.

What it answers is about the robot only: reachability, joint-limit margin, path length. It never returns object poses
or evaluator errors. Targets use the plan protocol's own format (two target points + orientation point), so a target
that passes here is planned the same way when the policy sends it (collisions with parts the planner does not model
are still the policy's responsibility, as in a plan).
"""
import json
import threading
import time
import traceback
from pathlib import Path

import numpy as np

CLIENT = r'''"""Station planning queries (read-only helper). Ask before committing a plan whether an arm can reach targets.

    from station_tools import check_pose, check_route
    r = check_pose('right', {'target_points': [[x,y,z],[x,y,z]], 'orientation_point': [x,y,z]})
    # -> {'reachable': bool, 'reason': str|None, 'goal_joint_limit_margin_deg': float, 'max_joint_excursion_deg': float}
    r = check_route('right', [target1, target2, ...])
    # -> {'all_reachable': bool, 'legs': [... one dict per leg, chained from the current joints ...], 'first_failed_leg': int|None}

Targets use exactly the plan format (two target points + orientation point; contact_constraint may be omitted). Each leg
is planned with the same planner, joint limits and path search that accept or reject a plan, starting from the arm's
current joints (route legs chain from the previous leg's end), and every leg is also checked for the 2 mm
clearance between the two arms with the OTHER arm where it is now (move the other arm first if it will be elsewhere).
Answers take a few seconds; no motion happens.
The planner does not check carried parts or the other arm against obstacles: that stays your job, as in a plan.
"""
import json, os, time, uuid

_Q, _A = '/work/station_queries', '/work/station_answers'


def _ask(kind, arm, targets, timeout_s=180):
    os.makedirs(_Q, exist_ok=True)
    qid = uuid.uuid4().hex
    norm = []
    for t in targets:
        t = dict(t); t.setdefault('contact_constraint', None); norm.append(t)
    tmp = f'{_Q}/.{qid}.tmp'
    with open(tmp, 'w') as f:
        json.dump({'id': qid, 'kind': kind, 'arm': arm, 'targets': norm}, f)
    os.replace(tmp, f'{_Q}/{qid}.json')
    path, t0 = f'{_A}/{qid}.json', time.time()
    while time.time() - t0 < timeout_s:
        if os.path.exists(path):
            with open(path) as f:
                return json.load(f)
        time.sleep(0.2)
    return {'error': 'no answer from the station within %d s' % timeout_s}


def check_pose(arm, target):
    return _ask('pose', arm, [target])


def check_route(arm, targets):
    return _ask('route', arm, list(targets))
'''


def stage(sandbox):
    """Write the read-only client helper into the workspace and create the query folders. -> info dict."""
    sandbox = Path(sandbox)
    for d in ('station_queries', 'station_answers'):
        (sandbox/d).mkdir(exist_ok=True)
    f = sandbox/'station_tools.py'
    f.write_text(CLIENT); f.chmod(0o444)
    return {'path': '/work/station_tools.py', 'functions': ['check_pose', 'check_route']}


class QueryServer(threading.Thread):
    def __init__(self, backend, sandbox, log_path=None):
        super().__init__(daemon=True)
        self.b, self.sandbox = backend, Path(sandbox)
        self.q, self.a = self.sandbox/'station_queries', self.sandbox/'station_answers'
        self.log = Path(log_path) if log_path else None
        self.stop_flag = threading.Event()
        self.count = 0

    def answer(self, req):
        from .controller.motion import plan_path
        from .controller.executor import Replan
        from .runtime.protocol import frame
        arm = req['arm']
        if arm not in ('left', 'right'):
            return {'error': "arm must be 'left' or 'right'"}
        if getattr(self.b, 'step_phase', None) == 'motion':
            return {'error': 'the robot is moving; ask again while planning'}
        q = np.asarray(self.b.joints()[arm], float)
        legs = []
        for i, t in enumerate(req['targets']):
            try:
                p, r = frame(t)
            except Exception as e:  # noqa: BLE001 - malformed target
                legs.append({'leg': i, 'reachable': False, 'reason': 'invalid target: '+str(e)}); break
            try:
                path = plan_path(self.b, arm, q, p, r)
                diag = dict(getattr(self.b, 'path_diagnostics', {}).get(arm, {}))
                # The executor also requires 2 mm between the two arms at every step; check the path against the other
                # arm where it is now (plan_path alone does not).
                other = 'right' if arm == 'left' else 'left'
                q_other = np.asarray(self.b.joints()[other], float)
                pts = list(path)
                pts = pts[::max(1, len(pts)//40)] + pts[-1:]
                for k, qq in enumerate(pts):
                    try:
                        self.b.validate_pair({arm: np.asarray(qq, float), other: q_other})
                    except Replan as e:
                        raise Replan(f'{e} (at {100*k/max(1, len(pts)-1):.0f}% of this leg, with the {other} arm where it is now)')
                legs.append({'leg': i, 'reachable': True, 'reason': None, 'inter_arm_checked': True,
                             'goal_joint_limit_margin_deg': diag.get('goal_joint_limit_margin_deg'),
                             'max_joint_excursion_deg': diag.get('max_joint_excursion_deg')})
                q = np.asarray(path[-1], float)
            except Replan as e:
                legs.append({'leg': i, 'reachable': False, 'reason': str(e)}); break
        if req['kind'] == 'pose':
            return legs[0]
        failed = next((x['leg'] for x in legs if not x['reachable']), None)
        return {'all_reachable': failed is None and len(legs) == len(req['targets']), 'first_failed_leg': failed, 'legs': legs}

    def run(self):
        while not self.stop_flag.is_set():
            for f in sorted(self.q.glob('*.json')):
                try:
                    req = json.loads(f.read_text())
                    t0 = time.time()
                    ans = self.answer(req)
                    ans['seconds'] = round(time.time()-t0, 2)
                except Exception as e:  # noqa: BLE001 - never kill the episode over a query
                    req, ans = {'id': f.stem}, {'error': 'query failed: '+repr(e)}
                    if self.log:
                        with self.log.open('a') as lg:
                            lg.write(traceback.format_exc())
                tmp = self.a/f'.{f.stem}.tmp'
                tmp.write_text(json.dumps(ans, default=float))
                tmp.replace(self.a/f'{f.stem}.json')
                f.unlink(missing_ok=True)
                self.count += 1
                if self.log:
                    with self.log.open('a') as lg:
                        lg.write(json.dumps({'t': time.time(), 'req': req, 'ans': ans}, default=float)+'\n')
            self.stop_flag.wait(0.2)
