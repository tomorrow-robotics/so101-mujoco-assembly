"""Cross-episode experience for the Q0 harness (not part of upstream Q0-Preview).

Upstream gives the policy a `memory` string that lives inside one episode only: every episode starts with an empty
session and a fresh /work, and nothing an episode learned reaches the next one. This module is that missing channel.

After each episode the launcher calls `record(run_dir)`: it stores the station evaluator's result, the targets the
planner rejected, and the policy's own final memory in `assets/experience/<task>.json`. Before the next episode of
the same task the launcher calls `stage(task, sandbox)`: it writes `/work/experience.md` (read-only) from that store
plus the measured arm-reach table (`assets/experience/robot_reach.md`, built by scripts/build_q0_reach_table.py).
The task card tells the policy the file exists. Nothing here is written by hand except optional operator notes.

Rules for what is stored: the policy's own words, the evaluator's measurements, and planner feedback. An episode the
simulator invalidated keeps only what happened before the defect.

    python -m q0_harness.experience record <run_dir> [--note "..."]      # (re)record one finished episode
    python -m q0_harness.experience show <task>                          # print the /work/experience.md text
"""
import argparse
import json
import re
import time
from pathlib import Path


STORE = Path(__file__).resolve().parent/'assets/experience'
FULL_MEMORY_EPISODES = 1     # the newest episode keeps the policy's full final memory; older ones a short excerpt
OLD_MEMORY_CHARACTERS = 350
MAX_EPISODES_LISTED = 4      # the guide carries the lessons of older ones
AXIAL_SIGN = {'so101_base_m1_insert': 'negative = short of the designed depth, positive = past it',
              'so101_m1_cap_install': 'positive = above its CAD seat position',
              'so101_shoulder_fork_install': 'negative = short of its depth, positive = past it',
              'so101_shoulder_seat_install': 'negative = short of its depth, positive = past it',
              'so101_motor2_install': 'negative = short of its depth, positive = past it',
              'so101_upper_arm_install': 'negative = short of its depth, positive = past it',
              'so101_motor3_install': 'negative = short of its depth, positive = past it',
              'so101_forearm_install': 'negative = short of its depth, positive = past it'}
STEP_NAMES = {'so101_base_m1_insert': 'step 1 (M1 into Base)', 'so101_m1_cap_install': 'step 2 (retaining cap)',
              'so101_shoulder_fork_install': 'step 3 (shoulder fork)',
              'so101_shoulder_seat_install': 'step 4 (shoulder seat)', 'so101_motor2_install': 'step 5 (M2 servo)',
              'so101_upper_arm_install': 'step 6 (upper arm)',
              'so101_motor3_install': 'step 7 (M3 servo)', 'so101_forearm_install': 'step 8 (forearm)'}


def _events(run):
    return [json.loads(line) for line in (run/'session/execution.jsonl').read_text().splitlines() if line.strip()]


def _tool_target(target):
    from .runtime.protocol import frame
    p, r = frame(target)
    return {'tcp_m': [round(float(x), 3) for x in p], 'tool_direction': [round(float(x), 2) for x in r[:, 0]],
            'jaw_axis': [round(float(x), 2) for x in r[:, 1]]}


def summarize_episode(run, note=None):
    """One finished episode -> experience entry (dict)."""
    run = Path(run).resolve()
    launch = json.loads((run/'LAUNCH.json').read_text())
    result = json.loads((run/'RESULT.json').read_text())
    events = _events(run)
    physics = result.get('physics_events') or []
    invalid_step = min((e['step'] for e in physics), default=None)
    # Walk the log: planner rejections with the rejected arm's target, plan memories, control-step count.
    rejections, last_plan, steps, memory_before_defect, plans, groups_done = {}, None, 0, None, 0, 0
    final_memory = None
    for e in events:
        ev = e['event']
        if ev == 'action_acknowledged':   # one per executed control step (the backend's step counter)
            steps += 1
        elif ev == 'plan':
            if invalid_step is not None and steps >= invalid_step:
                continue
            last_plan = e['payload']; plans += 1
            if last_plan.get('memory'):
                final_memory = last_plan['memory']
                memory_before_defect = last_plan['memory']
        elif ev == 'group_completed' and (invalid_step is None or steps < invalid_step):
            groups_done += 1
        elif ev == 'plan_cancelled' and (invalid_step is None or steps < invalid_step):
            reason = str(e.get('reason'))
            key = re.sub(r'-?\d+\.\d+', '#', reason)[:200]
            item = rejections.setdefault(key, {'reason': reason[:240], 'count': 0, 'examples': []})
            item['count'] += 1
            arm = reason.split()[0] if reason.split() and reason.split()[0] in ('left', 'right') else None
            if arm and last_plan and len(item['examples']) < 4:
                for g in last_plan['groups']:
                    if g.get(arm):
                        try:
                            ex = {'arm': arm, **_tool_target(g[arm])}
                        except Exception:
                            continue
                        if ex not in item['examples']:
                            item['examples'].append(ex)
                        break
    end = result.get('end') or (json.loads((run/'session/END.json').read_text()) if (run/'session/END.json').exists() else {})
    stop = (run/'STOP_REASON.txt').read_text().strip() if (run/'STOP_REASON.txt').exists() else None
    if invalid_step is not None:
        status = 'invalid'
    elif result.get('success'):
        status = 'success'
    elif str(end.get('reason', '')).startswith('api_'):
        status = 'aborted'   # the model service stopped answering; the task was cut off, not failed by the plan
    elif plans == 0:
        status = 'aborted'   # no plan was ever accepted (agent or service never started): nothing about the task was tried
    else:
        status = 'failed'
    return {'episode': run.name, 'task': launch['task'], 'started_unix_s': float(launch.get('started_unix_s', run.stat().st_mtime)), 'date': time.strftime('%Y-%m-%d', time.localtime(launch.get('started_unix_s', run.stat().st_mtime))),
            'status': status, 'invalid_from_control_step': invalid_step,
            'evaluator': {k: result.get(k) for k in ('success', 'fastened_after_seating', 'final_error', 'gates', 'best_error_during_episode',
                                                     'process_violations', 'base_max_rise_mm', 'base_drift_mm', 'earlier_parts', 'events')},
            'end_reason': end.get('reason'), 'operator_stop': stop, 'plans': plans, 'groups_completed': groups_done,
            'control_steps': result.get('control_steps', end.get('control_steps')), 'planner': launch.get('planner'),
            'planner_rejections': sorted(rejections.values(), key=lambda x: -x['count']),
            'policy_final_memory': memory_before_defect if invalid_step is not None else final_memory,
            'operator_note': note}


def _load(task):
    path = STORE/f'{task}.json'
    return json.loads(path.read_text()) if path.exists() else {'task': task, 'episodes': []}


def record(run, note=None):
    """Add (or replace) one episode in the task's store. -> entry."""
    entry = summarize_episode(run, note)
    store = _load(entry['task'])
    old = [e for e in store['episodes'] if e['episode'] == entry['episode']]
    if note is None and old:
        entry['operator_note'] = old[0].get('operator_note')
    store['episodes'] = [e for e in store['episodes'] if e['episode'] != entry['episode']] + [entry]
    store['episodes'].sort(key=lambda e: (e.get('started_unix_s', 0.), e['episode']))   # chronological
    STORE.mkdir(parents=True, exist_ok=True)
    (STORE/f'{entry["task"]}.json').write_text(json.dumps(store, indent=1, ensure_ascii=False)+'\n')
    return entry


def add_note(task, text):
    """Append an operator note (a lesson stated by the person running the station) to the task's store."""
    store = _load(task)
    store.setdefault('operator_notes', []).append({'date': time.strftime('%Y-%m-%d'), 'text': text.strip()})
    STORE.mkdir(parents=True, exist_ok=True)
    (STORE/f'{task}.json').write_text(json.dumps(store, indent=1, ensure_ascii=False)+'\n')
    return store['operator_notes']


def _fmt_error(err, task):
    if not err:
        return 'not measured'
    return (f"lateral {err['lateral_mm']:.3f} mm, along the insertion direction {err['axial_mm']:+.3f} mm "
            f"({AXIAL_SIGN.get(task, 'signed')}), angle {err['angle_deg']:.2f} deg")


def _episode_text(e, full_memory):
    ev = e['evaluator']
    head = {'success': 'SUCCESS', 'failed': 'FAILED', 'invalid': 'INVALID (simulator defect, since fixed)',
            'aborted': 'CUT OFF MID-TASK (the station lost the planning service; not a failure of the plan)'}[e['status']]
    lines = [f"### {e['episode']} ({e['date']}): {head}", '']
    if e['status'] == 'invalid':
        lines.append(f"The simulator misbehaved at control step {e['invalid_from_control_step']}; the result says nothing about the plan. "
                     'Only what happened before that step is kept below.')
    else:
        big = max(abs(v) for v in (ev.get('final_error') or {'x': 0}).values()) > 50
        lines.append(('State when it was cut off, by the evaluator' if e['status'] == 'aborted' else 'Evaluator') +
                     ' (true simulation state against the CAD target): ' +
                     ('the part ended far from its seat (never inserted)' if big else 'final error ' + _fmt_error(ev.get('final_error'), e['task'])) +
                     (f"; fastened after seating: {ev.get('fastened_after_seating')}" if ev.get('fastened_after_seating') is not None else '') + '.')
        if ev.get('base_max_rise_mm') is not None:
            lines.append(f"Base lifted off the table by at most {ev['base_max_rise_mm']:.1f} mm (more than 1 mm is a violation); "
                         f"violations: {ev.get('process_violations') or 'none'}.")
        for x in ev.get('events') or []:
            if x['event'] == 'fastened_after_seating':
                lines.append(f"Station event: the part was seated within tolerance and fastened at control step {x['control_step']} "
                             f"({_fmt_error(x['error_at_activation'], e['task'])}).")
            elif x['event'] == 'base_lifted':
                lines.append(f"Station event: the Base started to leave the table at control step {x['control_step']}.")
        for part, st in (ev.get('earlier_parts') or {}).items():
            lines.append(f"Earlier part {part}: still fastened {st.get('still_fastened')}, within tolerance {st.get('within_gates')}.")
    lines.append(f"Used {e['plans']} planning rounds, {e['groups_completed']} motion groups completed, {e['control_steps']} control steps; "
                 f"ended by: {e['end_reason']}.")
    if e.get('operator_note'):
        lines.append('Station note: ' + e['operator_note'])
    if e['planner_rejections']:
        lines += ['', 'Targets the planner rejected before any motion:']
        for r in e['planner_rejections']:
            lines.append(f"- {r['count']}x \"{r['reason']}\"")
            for ex in r['examples']:
                lines.append(f"  - {ex['arm']} tool point {ex['tcp_m']} m, tool pointing {ex['tool_direction']}, jaw axis {ex['jaw_axis']}")
    mem = e.get('policy_final_memory')
    if mem:
        label = "The policy's own last summary in that episode (its claims were not all verified)"
        if e['status'] == 'invalid':
            label = "The policy's own summary just before the simulator defect"
        if not full_memory and len(mem) > OLD_MEMORY_CHARACTERS:
            mem = mem[:OLD_MEMORY_CHARACTERS] + ' [...]'
        lines += ['', label + ':', '', '> ' + mem.replace('\n', '\n> ')]
    return '\n'.join(lines)


def resume_section(resume):
    """Text that tells the policy this episode continues one that was cut off (see run_q0_episode.py --resume-from)."""
    if resume.get('reason') == 'rewound':
        why = (f"This episode continues episode {resume['episode']} of this task from the end of its planning round {resume.get('episode_round', resume['rounds_used'])}. "
               'That episode went on from there and did not get the part fastened; the station has since learned more about this '
               'step (see the guide above, which now includes what went wrong after that point), and the work is taken up again '
               'from there. ')
    else:
        why = (f"Episode {resume['episode']} of this task was cut off after planning round {resume['rounds_used']} because the station lost "
               'its planning service; nothing in the task caused it. ')
    lines = ['## This episode continues an earlier one', '',
             why + 'The simulation has been restored to exactly the state at the end of '
             'that round by re-executing the same commands from the same start: both arms, the grippers and every part are where that '
             'round left them, and nothing has moved since. The home poses are the original ones.',
             f"Budget left: {resume['rounds_left']} planning rounds, {resume['control_steps_left']} control steps.",
             'Your Python session is new, so variables are gone. Files you saved in /work during that session up to that round are in '
             '/work/previous_session/ (read-only), including your scripts (_python_*.py) and their printed outputs (_python_*.out). '
             'Observation ids restart at obs_00000; ids in the summary below belong to that earlier session and those images are '
             'no longer available.',
             'The motion group submitted together with your last summary was executed to completion' +
             (f" ({resume['last_outcome']})" if resume.get('last_outcome') else '') +
             '; you have not seen its result yet, so check it in the first observation.', '',
             'Your last summary at that point, verbatim:', '',
             '> ' + str(resume.get('memory') or '(none)').replace('\n', '\n> '), '']
    return lines


def render(task, resume=None):
    """The /work/experience.md text for one task (resume: see resume_section)."""
    store = _load(task)
    if resume:   # the continued episode itself, and anything that happened after it started, is not "earlier"
        me = next((e for e in store['episodes'] if e['episode'] == resume['episode']), None)
        store = {**store, 'episodes': [e for e in store['episodes'] if e['episode'] != resume['episode'] and
                                       (resume.get('reason') != 'rewound' or me is None
                                        or e.get('started_unix_s', 0.) < me.get('started_unix_s', 0.))]}
    parts = ['# Experience from earlier episodes', '',
             f"Task: {STEP_NAMES.get(task, task)}. This file is the station's record of earlier episodes of this task on this robot. "
             'Each episode starts with an empty memory; this file is the only thing carried over. It contains measurements and '
             'what earlier attempts did, not instructions: the current images and the CAD decide what to do now. '
             'Object poses differ between episodes, so earlier coordinates are examples, not current positions.', '']
    if resume:
        parts += resume_section(resume)
    guide = STORE/f'{task}_guide.md'   # the station's distilled account of what worked in this step
    if guide.exists():
        parts += [guide.read_text().strip(), '']
    if store.get('operator_notes'):
        parts += ['## Notes from the station operator', '',
                  'Written by the person running the station after watching earlier episodes.', '']
        parts += [f"- ({n['date']}) {n['text']}" for n in store['operator_notes']] + ['']
    reach = STORE/'robot_reach.md'
    if reach.exists():
        parts += [reach.read_text().strip(), '']
    measured = STORE/f'{task}_measured.md'   # station measurements about this task's parts (scripts/build_q0_*_note.py)
    if measured.exists():
        parts += [measured.read_text().strip(), '']
    related = [(t, _load(t)) for t in STEP_NAMES if t != task]
    # an episode in which no plan was ever accepted tried nothing (agent or service never started): kept in the store, not shown
    eps = [e for e in store['episodes'] if not (e['status'] == 'aborted' and e.get('plans', 1) == 0)][::-1]
    total, eps = len(eps), eps[:MAX_EPISODES_LISTED]
    parts.append(f'## Earlier episodes of this task (newest {len(eps)} of {total}, newest first)')
    parts.append('')
    if not eps:
        parts.append('None yet.')
    for i, e in enumerate(eps):
        parts += [_episode_text(e, i < FULL_MEMORY_EPISODES), '']
    for t, other in related:
        done = [e for e in other['episodes'] if e['status'] != 'invalid']
        if done:
            parts += [f'## Results of {STEP_NAMES[t]} episodes (other step of the same assembly)', '']
            for e in done[::-1][:4]:
                ev = e['evaluator']
                parts.append(f"- {e['episode']}: {e['status']}; final error {_fmt_error(ev.get('final_error'), t)}."
                             + (f" Station note: {e['operator_note']}" if e.get('operator_note') else ''))
            parts.append('')
    return '\n'.join(parts).rstrip()+'\n'


def stage(task, sandbox, resume=None):
    """Write /work/experience.md (read-only) into the agent workspace. -> path, size."""
    sandbox = Path(sandbox); sandbox.mkdir(parents=True, exist_ok=True)
    path = sandbox/'experience.md'
    text = render(task, resume)
    path.write_text(text)
    path.chmod(0o444)
    return {'path': '/work/experience.md', 'characters': len(text), 'episodes': len(_load(task)['episodes'])}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record'); r.add_argument('run', type=Path); r.add_argument('--note')
    s = sub.add_parser('show'); s.add_argument('task')
    n = sub.add_parser('note'); n.add_argument('task'); n.add_argument('text')
    a = ap.parse_args()
    if a.cmd == 'record':
        e = record(a.run, a.note)
        print(json.dumps({k: e[k] for k in ('episode', 'task', 'status', 'plans', 'groups_completed')}),
              'rejections', [(x['count'], x['reason'][:60]) for x in e['planner_rejections']])
    elif a.cmd == 'note':
        print(json.dumps(add_note(a.task, a.text), indent=1, ensure_ascii=False))
    else:
        print(render(a.task))


if __name__ == '__main__':
    main()
