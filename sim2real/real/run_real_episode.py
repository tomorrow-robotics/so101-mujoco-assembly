"""Run one Q0 episode (default: step 1, M1 into Base) on the real two-arm cell, or on the mock.

    # offline, no robot, no model API: whole pipeline with the mocked twin and the API-free smoke agent
    python real/run_real_episode.py --mock --smoke-agent --out runs/mock_smoke --yes
    # real cell, smoke agent (no model): the arms only hold / make no-op groups
    python real/run_real_episode.py --robot my_sdk_adapter:MyRobot --cameras real.realsense:RealSenseCameras \
        --smoke-agent --out runs/real_smoke
    # real cell, Astra
    Q0_PREVIEW_API_KEY=... python real/run_real_episode.py --robot my_sdk_adapter:MyRobot \
        --cameras real.realsense:RealSenseCameras --agent-config q0_harness/config/agent.gpt6astra.example.json \
        --out runs/step1_real_001

Layout mirrors scripts/run_q0_episode.py (the simulator launcher): this process runs the backend and the unchanged
upstream Episode; a child process runs the unchanged upstream Agent (model + sandboxed Python); they exchange
READY.json / observations/<id>/reply.json files. The operator answers the station prompt in this terminal.
"""
import argparse
import importlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHARED = ROOT/'third_party/xpolicylab_shared'
sys.path[:0] = [str(ROOT), str(SHARED)]
os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco  # noqa: E402

from q0_harness.runtime.episode import Episode  # noqa: E402
from q0_harness.runtime.io_utils import atomic  # noqa: E402
from q0_harness.runtime.task_context import task_step_limit  # noqa: E402
from real.backend import RealBackend  # noqa: E402
from real.station import OperatorStation  # noqa: E402

TASKS = {'so101_base_m1_insert': ('Assemble the orange servo motor (M1) into the blue Base as in the CAD assembly.', 'cad')}


def load_class(spec):
    mod, _, name = spec.partition(':')
    return getattr(importlib.import_module(mod), name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True, help='new directory')
    ap.add_argument('--robot', help='module:Class implementing real.interface.RobotIO (constructor takes no arguments, '
                                    'or --robot-args JSON as keyword arguments)')
    ap.add_argument('--robot-args', default='{}')
    ap.add_argument('--cameras', default='real.realsense:RealSenseCameras', help='module:Class implementing CameraIO')
    ap.add_argument('--camera-args', default='{}')
    ap.add_argument('--mock', action='store_true', help='simulated twin instead of hardware (offline test)')
    ap.add_argument('--twin', type=Path, default=ROOT/'twin/twin.xml')
    ap.add_argument('--agent-config', type=Path, default=ROOT/'q0_harness/config/agent.gpt6astra.example.json')
    ap.add_argument('--smoke-agent', action='store_true', help='upstream API-free smoke agent instead of the model')
    ap.add_argument('--task', default='so101_base_m1_insert', choices=sorted(TASKS))
    ap.add_argument('--plans', type=int, default=30, help='planning rounds (the sim step-1 episodes used 30)')
    ap.add_argument('--wall-seconds', type=int, default=10800)
    ap.add_argument('--turn-seconds', type=int, default=3600)
    ap.add_argument('--checkpoint-seconds', type=int, default=60)
    ap.add_argument('--control-hz', type=float, default=25.)
    ap.add_argument('--slowdown', type=float, default=2., help='real seconds per control step = slowdown / control-hz '
                    '(2 = half the simulator speed; the agent-visible timing is unchanged)')
    ap.add_argument('--post-hold-s', type=float, default=2.)
    ap.add_argument('--planner', choices=('mujoco', 'curobo'), default='mujoco',
                    help='mujoco: IK + collision-checked joint paths on the twin (CPU); curobo: local cuRobo 895c651 (GPU)')
    ap.add_argument('--station-ask', choices=('compliant', 'every', 'never'), default='compliant',
                    help='when the operator is asked whether M1 is seated')
    ap.add_argument('--no-station-queries', action='store_true')
    ap.add_argument('--no-experience', action='store_true', help='do not stage experience.md and skills')
    ap.add_argument('--agent-python', default=sys.executable)
    ap.add_argument('--yes', action='store_true', help='skip the operator confirmations (mock / unattended tests only)')
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    cfg = json.loads(a.agent_config.read_text())
    atomic(out/'agent_config.json', cfg)
    instruction, cad_name = TASKS[a.task]

    model = mujoco.MjModel.from_xml_path(str(a.twin))
    data = mujoco.MjData(model)
    if a.mock:
        from real.mock import MockCameras, MockRobot
        robot = MockRobot(a.twin)
        cameras = MockCameras(robot)
    else:
        if not a.robot:
            raise SystemExit('--robot module:Class is required (or --mock)')
        robot = load_class(a.robot)(**json.loads(a.robot_args))
        cameras = load_class(a.cameras)(**json.loads(a.camera_args))
    if not a.mock and not a.yes:
        print('\nSafety check before the arms move:\n  - workspace clear, Base and M1 on the table, e-stop in reach\n'
              '  - both arms at the start pose and holding\n  - real/check_setup.py passed (FK overlay on the camera images)')
        input('Press Enter to start the episode (Ctrl-C aborts) ')

    limit = task_step_limit(a.task)
    dt = 1./a.control_hz
    backend = RealBackend(model, data, robot, cameras, limit, control_dt=dt, slowdown=a.slowdown,
                          post_planning_hold_steps=round(a.post_hold_s/dt), instruction=instruction, planner=a.planner,
                          curobo_options={'log_path': out/'curobo_service.log'})
    station = OperatorStation(backend, out, ask='never' if a.yes else a.station_ask)
    backend.after_group_hooks.append(station.after_action)
    backend.station_status = station.status
    session = out/'session'
    episode = Episode(backend, session, {'task_name': a.task, 'max_plans': a.plans, 'wall_seconds': a.wall_seconds,
                                         'turn_seconds': a.turn_seconds, 'checkpoint_seconds': a.checkpoint_seconds})
    backend.attach_executor(episode.executor)
    sandbox = session/'sandbox'
    cad = ROOT/'q0_harness/assets'/cad_name
    shutil.copytree(cad, sandbox/'cad')
    for f in (sandbox/'cad').iterdir():
        f.chmod(0o444)
    experience_info = skills_info = None
    if not a.no_experience:
        from q0_harness import experience, skills
        experience_info = experience.stage(a.task, sandbox, None)
        skills_info = skills.stage(a.task, sandbox)
    backend.depth_dir = sandbox/'depth'
    tdst = sandbox/'tools'; tdst.mkdir(parents=True, exist_ok=True)
    for f in (ROOT/'q0_harness/assets/tools').glob('*.py'):
        shutil.copy2(f, tdst/f.name); (tdst/f.name).chmod(0o444)
    queries_info, query_server = None, None
    if not a.no_station_queries:
        from q0_harness import station_queries
        queries_info = station_queries.stage(sandbox)
        query_server = station_queries.QueryServer(backend, sandbox, log_path=out/'station_queries.log')
        query_server.start()
    env = os.environ.copy()
    env.update(PYTHONPATH=os.pathsep.join([str(ROOT), str(SHARED)] + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else [])),
               PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               Q0_SANDBOX_MOUNT_PROC=os.environ.get('Q0_SANDBOX_MOUNT_PROC', '0'))
    mode = 'smoke_agent' if a.smoke_agent else 'runner'
    if mode == 'runner' and not env.get(cfg.get('api_key_env', 'Q0_PREVIEW_API_KEY')):
        raise SystemExit(f"set {cfg.get('api_key_env', 'Q0_PREVIEW_API_KEY')} (model API key) or use --smoke-agent")
    cmd = [a.agent_python, '-u', '-m', f'q0_harness.harness.{mode}', '--run-dir', str(session)]
    if mode == 'runner':
        cmd += ['--config', str(out/'agent_config.json')] + (['--enable-openai'] if cfg.get('provider') == 'openai' else [])
    with (out/'agent.log').open('w') as log:
        agent = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    atomic(out/'LAUNCH.json', {'agent_cmd': cmd, 'agent_pid': agent.pid, 'twin': str(a.twin), 'task': a.task,
                               'mock': a.mock, 'robot': a.robot, 'cameras': 'mock' if a.mock else a.cameras,
                               'control_hz': a.control_hz, 'slowdown': a.slowdown, 'plans': a.plans,
                               'planner': a.planner, 'started_unix_s': time.time(), 'cad_package': cad_name,
                               'experience': experience_info, 'skills': skills_info, 'station_queries': queries_info,
                               'station': {'kind': 'operator', 'ask': station.ask}})
    print(f'episode running: {out}\n  agent log: {out/"agent.log"}\n  stop: touch {session/"STOP"}', flush=True)
    try:
        episode.run()
    except KeyboardInterrupt:
        print('interrupted by operator; stopping the arms', flush=True)
        backend.cancel()
    finally:
        if query_server:
            query_server.stop_flag.set()
        try:
            agent.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(agent.pid, signal.SIGTERM)
            agent.wait(timeout=30)
        try:
            robot.stop()
        except Exception as exc:  # noqa: BLE001
            print('robot.stop failed:', repr(exc), flush=True)
        result = station.assess(backend, interactive=not a.yes)
        result.update(end=json.loads((session/'END.json').read_text()) if (session/'END.json').exists() else None,
                      agent_returncode=agent.returncode, control_steps=backend.steps,
                      hardware_faults=backend.physics_events, compliance_fallback_steps=backend.compliance_fallback_steps)
        atomic(out/'RESULT.json', result)
        backend.save_record(out/'trajectory.npz')
        try:
            from PIL import Image
            for name, cam in backend.rig.capture(data).items():
                Image.fromarray(cam['color']).save(out/f'final_{name}.png')
        except Exception as exc:  # noqa: BLE001
            print('final images not captured:', repr(exc), flush=True)
        backend.close()
        print(json.dumps({k: result.get(k) for k in ('success', 'fastened_signal', 'control_steps', 'end')}, default=str),
              flush=True)


if __name__ == '__main__':
    main()
