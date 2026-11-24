"""Run one Q0-Preview episode on the native Panthera/MuJoCo SO-101 scene.

Process layout mirrors upstream tools/policy_service.py:
- this process: native backend (physics + rendered cameras) driven by the UNCHANGED upstream
  runtime/episode.py (observation publishing, deadlines, max_plans, checkpoints, STOP, END, hold);
- child process: the UNCHANGED upstream harness/runner.py Agent (model tool loop + bwrap sandbox),
  talking to the model through upstream harness/api.py (provider openai, base_url = local gateway);
- they exchange READY.json / observations/<id>/reply.json files exactly as upstream.
After END, the evaluator side (adapters/so101_station.py) writes the official result, unseen by the policy.

    python scripts/run_q0_episode.py --scene /tmp/q0-step1-scene/scene.xml --out results/q0_episodes/<new> \
        --agent-config q0_harness/config/agent.gpt6astra.gateway.json
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHARED = ROOT/'third_party/xpolicylab_shared'
sys.path[:0] = [str(ROOT), str(ROOT/'scripts'), str(SHARED)]
os.environ.setdefault('MUJOCO_GL', 'osmesa')

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from q0_harness.adapters.mujoco_backend import NativeMujocoBackend  # noqa: E402
from q0_harness.adapters.model_loading import load_model  # noqa: E402
from q0_harness.adapters.so101_station import SO101Station  # noqa: E402
from q0_harness.runtime.episode import Episode  # noqa: E402
from q0_harness.runtime.task_context import task_step_limit  # noqa: E402
from q0_harness.runtime.io_utils import atomic  # noqa: E402

TASKS = {  # task -> (instruction, CAD package, part installed by this stage, parts fastened earlier)
    'so101_base_m1_insert': ('Assemble the orange servo motor (M1) into the blue Base as in the CAD assembly.', 'cad', 'motor', ()),
    'so101_m1_cap_install': ('Install the green retaining cap on the Base+M1 assembly as in the CAD assembly.', 'cad_step2', 'holder', ('motor',)),
    'so101_shoulder_fork_install': ("Mount the yellow shoulder fork on M1's output horns as in the CAD assembly.", 'cad_step3', 'shoulder_fork', ('motor', 'holder')),
    'so101_shoulder_seat_install': ('Install the printed shoulder seat in the shoulder fork as in the CAD assembly.', 'cad_step4', 'shoulder_seat', ('motor', 'holder', 'shoulder_fork')),
    'so101_motor2_install': ('Install the second servo motor (M2) in the shoulder fork as in the CAD assembly.', 'cad_step5', 'motor_2', ('motor', 'holder', 'shoulder_fork', 'shoulder_seat')),
    'so101_upper_arm_install': ("Mount the upper arm on M2's output horns as in the CAD assembly.", 'cad_step6', 'upper_print', ('motor', 'holder', 'shoulder_fork', 'shoulder_seat', 'motor_2')),
    'so101_motor3_install': ('Install the third servo motor (M3) in the upper arm as in the CAD assembly.', 'cad_step7', 'motor3', ('motor', 'holder', 'shoulder_fork', 'shoulder_seat', 'motor_2', 'upper_print')),
    'so101_forearm_install': ("Mount the forearm on M3's output horns as in the CAD assembly.", 'cad_step8', 'forearm', ('motor', 'holder', 'shoulder_fork', 'shoulder_seat', 'motor_2', 'upper_print', 'motor3')),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scene', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True, help='new directory')
    ap.add_argument('--agent-config', type=Path, required=True)
    ap.add_argument('--task', default='so101_base_m1_insert')
    ap.add_argument('--plans', type=int, default=100)
    ap.add_argument('--wall-seconds', type=int, default=10800)
    ap.add_argument('--turn-seconds', type=int, default=3600)
    ap.add_argument('--checkpoint-seconds', type=int, default=60)
    ap.add_argument('--control-hz', type=float, default=25.)
    ap.add_argument('--post-hold-s', type=float, default=2., help='evaluator hold before closing the episode')
    ap.add_argument('--expose-object-ground-truth', action='store_true',
                    help='add simulator object poses to observations (upstream Q0 exposes none)')
    ap.add_argument('--init-state', type=Path, help='mapped native state from the previous stage (scripts/map_step_state.py)')
    ap.add_argument('--render-on-pod', action='store_true',
                    help='render the Q0 cameras on the GPU pod (NVIDIA EGL) through one devbox-initiated ssh connection')
    ap.add_argument('--planner', choices=('mujoco', 'curobo'), default='mujoco',
                    help='curobo: upstream CuroboPlanner on the GPU pod (adapters/pod_curobo.py); mujoco: native stand-in')
    ap.add_argument('--curobo-gpu', default=None, help='pod GPU index for the CuRobo service (default $Q0_CUROBO_GPU or 7)')
    ap.add_argument('--agent-python', default=sys.executable)
    ap.add_argument('--resume-from', type=Path, default=None,
                    help='continue an episode that was cut off by an infrastructure failure: re-execute its recorded commands '
                         'from the same start up to the end of its last planning round (deterministic replay, verified against '
                         'the record), then let the policy go on with the remaining rounds; scene/task/init-state come from it')
    ap.add_argument('--resume-through-plan', type=int, default=None,
                    help='with --resume-from: re-execute only up to and including this planning round of the earlier episode '
                         '(default: all of it), e.g. to continue from before a mistake under a changed station rule')
    ap.add_argument('--resume-scene', type=Path, default=None,
                    help='with --resume-from: replay in this scene instead of the recorded one; it may only add inactive '
                         'equalities (e.g. scripts/add_base_clamp.py), so the replay is checked to reproduce the record')
    ap.add_argument('--resume-init-state', type=Path, default=None, help='with --resume-scene: the start state converted into it')
    ap.add_argument('--clamp-base', action='store_true',
                    help="activate the scene's base_table_clamp weld at the Base's current pose before the policy starts "
                         '(protocol change 2026-11-20: the Base is mounted on the table from step 8 on, like the real arm)')
    ap.add_argument('--fastening', choices=('action_end', 'step'), default='action_end',
                    help="when the station checks seating and fastens: 'action_end' (default) = after every finished action, "
                         "reported to the policy as observation['station']; 'step' = every control step, no signal (earlier runs)")
    ap.add_argument('--gate', nargs=5, action='append', default=[], metavar=('PART', 'LATERAL_MM', 'AXIAL_MIN_MM', 'AXIAL_MAX_MM', 'ANGLE_DEG'),
                    help='per-episode station gate for one part (recorded in LAUNCH.json), e.g. --gate shoulder_seat 0.4 -0.3 0.8 1.2')
    ap.add_argument('--keep-rule', choices=('held', 'gate'), default='held',
                    help="earlier parts at the end: 'held' = still fastened and moved < 0.2 mm / 0.2 deg during this stage "
                         "(default); 'gate' = still fastened and inside their own seating gate (episodes before 2026-11-19)")
    ap.add_argument('--no-station-queries', action='store_true',
                    help='do not offer /work/station_tools.py (planning queries: can an arm reach a target / route)')
    ap.add_argument('--no-experience', action='store_true',
                    help='do not stage /work/experience.md and do not record this episode (upstream-faithful: no cross-episode memory)')
    ap.add_argument('--smoke-agent', action='store_true', help='upstream API-free smoke_agent instead of runner')
    ap.add_argument('--smoke-two-group', action='store_true', help='upstream smoke_agent --two-group (checkpoint gate)')
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    cfg = json.loads(a.agent_config.read_text())
    atomic(out/'agent_config.json', cfg)

    previous = None
    if a.resume_from:
        previous = a.resume_from.resolve()
        pl = json.loads((previous/'LAUNCH.json').read_text())
        a.task, a.scene, a.control_hz = pl['task'], Path(pl['scene']), pl['control_hz']
        a.init_state = Path(pl['init_state']) if pl.get('init_state') else None
        if a.resume_scene:
            a.scene = a.resume_scene
            a.init_state = a.resume_init_state or a.init_state
    instruction, cad_name, part, keep = TASKS[a.task]
    model_repair = {}
    model = load_model(str(a.scene), model_repair)
    print('model: polygon normals repaired', json.dumps(model_repair), flush=True)
    data = mujoco.MjData(model)
    if a.init_state:
        z = np.load(a.init_state)
        if Path(str(z['scene'])).resolve() != a.scene.resolve():
            raise SystemExit('init state was mapped for a different scene')
        model.eq_data[:] = z['eq_data']
        mujoco.mj_setState(model, data, z['state'], int(z['spec']))
        data.eq_active[:] = z['eq_active']
    mujoco.mj_forward(model, data)
    limit = task_step_limit(a.task)
    dt = 1./a.control_hz
    rig = None
    if a.render_on_pod:
        from q0_harness.adapters.pod_render import PodCameraRig
        rig = PodCameraRig(model, a.scene)
        print('render:', rig.backend, flush=True)
    backend = NativeMujocoBackend(model, data, limit, rig=rig, control_dt=dt, post_planning_hold_steps=round(a.post_hold_s/dt),
                                  expose_object_ground_truth=a.expose_object_ground_truth, instruction=instruction,
                                  planner=a.planner,
                                  curobo_options={'gpu': a.curobo_gpu, 'log_path': out/'curobo_service.log'})
    if a.planner == 'curobo':
        print('planner: curobo', json.dumps({k: backend.curobo_service.info[k] for k in
                                             ('curobo', 'torch', 'gpu', 'yml_sha256', 'total_startup_s')}), flush=True)
    gates = {g[0]: {'lateral': float(g[1])*1e-3, 'axial_min': float(g[2])*1e-3, 'axial_max': float(g[3])*1e-3,
                    'angle_deg': float(g[4])} for g in a.gate}
    station = SO101Station(model, data, part=part, keep=keep, fasten=a.fastening, gates=gates, keep_rule=a.keep_rule)
    station.trace_path = out/'station_trace.jsonl'   # evaluator-side, outside the agent workspace
    backend.after_step_hooks.append(station.after_step)
    backend.after_group_hooks.append(station.after_action)
    if a.fastening == 'action_end':
        backend.station_status = station.status
    session = out/'session'
    resume = None
    if previous:
        # Same start + same commands -> the state at the end of the interrupted episode's last planning round.
        from q0_harness.adapters.replay import load_trace, replay_plans
        # Chained continuations: if `previous` itself continued an earlier episode, that earlier episode's commands
        # (up to where it was rewound) are replayed first, oldest first; then `previous`'s own commands.
        chain, run = [], previous
        while True:
            lr = json.loads((run/'LAUNCH.json').read_text())
            if not lr.get('resumed_from'):
                break
            parent = Path(lr['resumed_from'])
            through = int(lr['resume']['replay']['last_plan_number'])   # the parent's own last replayed round (rounds_used also counts its ancestors)
            chain.insert(0, (parent, through))
            run = parent
        ancestors = []
        for parent, through in chain:
            ptrace = load_trace(parent)
            pc = next((i for i, e in enumerate(ptrace) if e['event'] == 'plan' and int(e['plan_number']) > through), len(ptrace))
            station.fasten = json.loads((parent/'LAUNCH.json').read_text()).get('fastening', 'step')
            prep = replay_plans(backend, ptrace[:pc])
            ancestors.append({'episode': parent.name, 'through_plan': through, **{k: prep[k] for k in ('plans', 'control_steps', 'motion_step_mismatches')}})
            print('resume: replayed ancestor', json.dumps(ancestors[-1]), flush=True)
            if prep['motion_step_mismatches'] or prep['ended']:
                raise SystemExit('resume: an ancestor replay does not reproduce its record; not continuing')
        trace = load_trace(previous)
        cut_time = None
        if a.resume_through_plan is not None:
            cut = next((i for i, e in enumerate(trace) if e['event'] == 'plan' and int(e['plan_number']) > a.resume_through_plan), len(trace))
            cut_time = trace[cut]['time'] if cut < len(trace) else None
            trace = trace[:cut]
        station.fasten = pl.get('fastening', 'step')   # the interrupted part is re-executed under its own station rule
        rep = replay_plans(backend, trace)
        station.fasten = a.fastening
        rec = np.load(previous/'trajectory.npz')
        diff = float(np.max(np.abs(rec['qpos'][backend.steps-1] - data.qpos))) if backend.steps else 0.
        rep['max_abs_qpos_diff_vs_record'] = diff
        print('resume: replayed', json.dumps(rep), flush=True)
        if rep['motion_step_mismatches'] or rep['ended'] or diff > 1e-9:
            raise SystemExit('resume: the replay does not reproduce the interrupted episode; not continuing on a different state')
        rounds_used = int(rep['last_plan_number']) + 1 + sum(x['plans'] for x in ancestors)
        if a.plans - rounds_used < 1:
            raise SystemExit('resume: no planning rounds left')
        memories = [e['payload'].get('memory') for e in trace if e['event'] == 'plan' and e['payload'].get('memory')]
        last_group = [e for e in trace if e['event'] in ('group_completed', 'plan_cancelled')]
        resume = {'episode': previous.name, 'rounds_used': rounds_used, 'rounds_left': a.plans - rounds_used,
                  'episode_round': int(rep['last_plan_number']) + 1,
                  'reason': 'cut_off' if a.resume_through_plan is None else 'rewound', 'files_until_unix_s': cut_time,
                  'control_steps_used': backend.steps, 'control_steps_left': limit - backend.steps,
                  'memory': memories[-1] if memories else None,
                  'last_outcome': None if not last_group or last_group[-1]['event'] == 'group_completed'
                  else 'interrupted: ' + str(last_group[-1].get('reason')), 'replay': rep, 'ancestors': ancestors}
        a.plans -= rounds_used
    base_clamp = None
    if a.clamp_base:
        # Mount the Base on the table where it stands now (weld Base <-> world at the measured relative pose).
        eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'base_table_clamp')
        if eq < 0:
            raise SystemExit('--clamp-base: the scene has no base_table_clamp equality (scripts/add_base_clamp.py)')
        bid = model.body('base').id
        Tb = np.eye(4); Tb[:3, :3] = data.xmat[bid].reshape(3, 3); Tb[:3, 3] = data.xpos[bid]
        rel = np.linalg.inv(Tb)
        model.eq_data[eq, :3] = 0.
        model.eq_data[eq, 3:6] = rel[:3, 3]
        from scipy.spatial.transform import Rotation
        model.eq_data[eq, 6:10] = np.roll(Rotation.from_matrix(rel[:3, :3]).as_quat(), 1)
        data.eq_active[eq] = 1
        mujoco.mj_forward(model, data)
        base_clamp = {'activated_at_control_step': backend.steps, 'base_world_pos': Tb[:3, 3].round(6).tolist(),
                      'base_world_z_axis': Tb[:3, 2].round(6).tolist()}
        print('base clamp:', json.dumps(base_clamp), flush=True)
    episode = Episode(backend, session, {'task_name': a.task, 'max_plans': a.plans, 'wall_seconds': a.wall_seconds,
                                         'turn_seconds': a.turn_seconds, 'checkpoint_seconds': a.checkpoint_seconds})
    backend.attach_executor(episode.executor)   # honour group['compliance'] (force-limited pressing)
    # Task CAD (read-only reference) staged into the agent workspace before the Agent starts; runner unchanged.
    cad = ROOT/'q0_harness/assets'/cad_name
    if cad.is_dir():
        import shutil
        shutil.copytree(cad, session/'sandbox'/'cad')
        for f in (session/'sandbox'/'cad').iterdir():
            f.chmod(0o444)
    # Cross-episode experience (q0_harness/experience.py): what earlier episodes of this task tried and how they ended.
    experience_info = None
    if previous:   # the interrupted session's own files (saved transforms, scripts, outputs), read-only
        import shutil
        keep_out = {'vlm_geom.py', 'observation_io.py', 'geometry_reference.md', 'geometry_interfaces.md', 'geometry_evidence.md',
                    'experience.md', 'task_reference.json', 'python_environment.json'}
        dst = session/'sandbox'/'previous_session'; dst.mkdir(parents=True, exist_ok=True)
        for f in sorted((previous/'session/sandbox').iterdir()):
            if resume.get('files_until_unix_s') and f.stat().st_mtime > resume['files_until_unix_s']:
                continue   # written after the point this episode continues from
            if f.is_file() and f.name not in keep_out and not f.name.startswith('_kernel'):
                shutil.copy2(f, dst/f.name); (dst/f.name).chmod(0o444)
    if not a.no_experience or resume:
        from q0_harness import experience
        experience_info = experience.stage(a.task, session/'sandbox', resume)
    skills_info = None
    if not a.no_experience:   # skill library (q0_harness/skills.py): procedures distilled from earlier episodes
        from q0_harness import skills
        skills_info = skills.stage(a.task, session/'sandbox')
        print('skills:', json.dumps(skills_info), flush=True)
        print('experience:', json.dumps(experience_info), flush=True)
    if getattr(backend.rig, 'depth', False):
        backend.depth_dir = session/'sandbox'/'depth'   # observation() writes depth maps here (/work/depth)
    tools_src = ROOT/'q0_harness/assets/tools'
    if tools_src.is_dir():   # station tools for the policy's Python (read-only): pose estimation from colour + depth
        tdst = session/'sandbox'/'tools'; tdst.mkdir(parents=True, exist_ok=True)
        for f in tools_src.glob('*.py'):
            shutil.copy2(f, tdst/f.name); (tdst/f.name).chmod(0o444)
    queries_info, query_server = None, None
    if not a.no_station_queries:
        from q0_harness import station_queries
        queries_info = station_queries.stage(session/'sandbox')
        query_server = station_queries.QueryServer(backend, session/'sandbox', log_path=out/'station_queries.log')
        query_server.start()
        print('station queries:', json.dumps(queries_info), flush=True)
    env = os.environ.copy()
    env.update(PYTHONPATH=os.pathsep.join([str(ROOT), str(SHARED)] + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else [])), PYTHONDONTWRITEBYTECODE='1',
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', Q0_SANDBOX_MOUNT_PROC=os.environ.get('Q0_SANDBOX_MOUNT_PROC', '0'))
    env.setdefault(cfg.get('api_key_env', 'Q0_PREVIEW_API_KEY'), 'local-gateway')  # gateway ignores the value
    mode = 'smoke_agent' if (a.smoke_agent or a.smoke_two_group) else 'runner'
    cmd = [a.agent_python, '-u', '-m', f'q0_harness.harness.{mode}', '--run-dir', str(session)]
    if mode == 'runner':
        cmd += ['--config', str(out/'agent_config.json')] + (['--enable-openai'] if cfg.get('provider') == 'openai' else [])
    elif a.smoke_two_group:
        cmd.append('--two-group')
    with (out/'agent.log').open('w') as log:
        agent = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    atomic(out/'LAUNCH.json', {'agent_cmd': cmd, 'agent_pid': agent.pid, 'scene': str(a.scene), 'task': a.task,
                               'control_hz': a.control_hz, 'mujoco_gl': os.environ.get('MUJOCO_GL'),
                               'expose_object_ground_truth': a.expose_object_ground_truth, 'started_unix_s': time.time(),
                               'init_state': str(a.init_state) if a.init_state else None, 'cad_package': cad_name,
                               'polygon_normal_repair': model_repair, 'experience': experience_info, 'skills': skills_info,
                               'resumed_from': str(previous) if previous else None, 'resume': resume, 'plans': a.plans,
                               'fastening': a.fastening, 'gates': gates, 'keep_rule': a.keep_rule,
                               'render': getattr(backend.rig, 'backend', '') or os.environ.get('MUJOCO_GL'),
                               'render_info': getattr(backend.rig, 'info', None), 'planner': a.planner,
                               'planner_info': backend.curobo_service.info if backend.curobo_service else None,
                               'station_queries': queries_info, 'base_clamp': base_clamp,
                               'resume_scene_override': str(a.resume_scene) if a.resume_scene else None})
    try:
        episode.run()
        if query_server:
            query_server.stop_flag.set()
    finally:
        try:
            agent.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(agent.pid, signal.SIGTERM)
            agent.wait(timeout=30)
        result = station.assess(backend)
        result.update(end=json.loads((session/'END.json').read_text()) if (session/'END.json').exists() else None,
                      agent_returncode=agent.returncode, control_steps=backend.steps, sim_time_s=backend.sim_time())
        # Result and trajectory first: they must survive a lost render connection (pod job killed mid-episode, S6c).
        atomic(out/'RESULT.json', result)
        backend.save_record(out/'trajectory.npz')
        try:
            vision = backend.rig.capture(data)
            from PIL import Image
            for name, cam in vision.items():
                Image.fromarray(cam['color']).save(out/f'final_{name}.png')
        except Exception as e:   # noqa: BLE001 - final images are optional
            print('final images not captured:', repr(e), flush=True)
        state_type = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(model, state_type))
        mujoco.mj_getState(model, data, state, state_type)
        np.savez_compressed(out/'final_state.npz', state=state, eq_active=data.eq_active.copy(),
                            eq_data=model.eq_data.copy(), spec=int(state_type))
        backend.close()
        if not a.no_experience:
            try:
                from q0_harness import experience
                entry = experience.record(out, note=(f"Continuation of {previous.name}: its first {resume['rounds_used']} planning rounds "
                                                     'were re-executed exactly, then the policy went on from there'
                                                     + (' after an infrastructure cut-off.' if resume['reason'] == 'cut_off' else
                                                        ', now with the station fastening signal (the earlier episode had none).'))
                                          if previous else None)
                print('experience recorded:', entry['status'], f"{len(entry['planner_rejections'])} rejection kinds", flush=True)
            except Exception as exc:  # never lose an episode result over the experience log
                print('experience NOT recorded:', repr(exc), flush=True)
        print(json.dumps({k: result[k] for k in ('success', 'fastened_after_seating', 'final_error', 'control_steps', 'end')},
                         default=str), flush=True)


if __name__ == '__main__':
    main()
