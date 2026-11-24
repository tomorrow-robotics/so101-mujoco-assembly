"""Pod side of q0_harness/adapters/pod_curobo.py: upstream CuroboPlanner per arm behind a stdin/stdout JSON-line pipe.

    CUDA_VISIBLE_DEVICES=<gpu> /tmp/q0_curobo_venv/bin/python -u scripts/q0_curobo_server.py '<arms json>' [yml]
arms json: {"left": {"joint_names": [...], "table_height_in_base": z, "dt": 0.04}, "right": {...}}
Request : {"id": n, "arm": "left", "method": "plan_path", "args": [...], "kwargs": {...}}  (arrays as lists)
Response: {"id": n, "ok": true, "result": {...}, "seconds": t} or {"id": n, "ok": false, "error": "..."}
The first line written is a hello with versions, GPU, config hashes and per-arm startup time.
Only the protocol goes to stdout; CuRobo/warp logging is redirected to stderr.
"""
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

METHODS = ('solve_ik_to_joint', 'plan_joint', 'plan_path', 'plan_batch')


def encode(x):
    import numpy as np
    if isinstance(x, dict):
        return {k: encode(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [encode(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    return x


def main():
    proto = os.fdopen(os.dup(1), 'w', buffering=1)
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    arms = json.loads(sys.argv[1])
    t0 = time.monotonic()
    import numpy as np
    import torch
    import curobo
    from q0_harness.adapters.curobo_panthera import DEFAULT_YML, make_planner, tool_pose_in_base
    yml = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_YML
    planners, startup = {}, {}
    for arm, cfg in arms.items():
        t = time.monotonic()
        planners[arm] = make_planner(cfg['joint_names'], cfg['dt'], cfg['table_height_in_base'], yml)
        startup[arm] = time.monotonic()-t
    urdf = Path(yml).parent/'panthera_ht.urdf'
    hello = {'ready': True, 'curobo': curobo.__version__, 'torch': torch.__version__, 'cuda': torch.version.cuda,
             'gpu': torch.cuda.get_device_name(0), 'yml': str(yml),
             'yml_sha256': hashlib.sha256(Path(yml).read_bytes()).hexdigest(),
             'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
             'startup_s': startup, 'total_startup_s': time.monotonic()-t0,
             'arms': {a: {'ee_link': p.ee_link, 'tool_frames': p.tool_frames, 'frame_bias': list(p.frame_bias),
                          'cspace_joint_names': p.cspace_joint_names, 'table': p.scene_model['cuboid']['table']}
                      for a, p in planners.items()}}
    proto.write(json.dumps(hello)+'\n')
    for line in sys.stdin:
        if not line.strip():
            continue
        req = json.loads(line)
        t = time.monotonic()
        try:
            planner = planners[req['arm']]
            method, args, kwargs = req['method'], req.get('args', []), req.get('kwargs', {})
            if method == 'fk':
                qs = np.asarray(args[0], float).reshape(-1, len(planner.active_joints_name))
                out = [tool_pose_in_base(planner, q) for q in qs]
                result = {'position': [p for p, _ in out], 'quaternion_wxyz': [q for _, q in out]}
            elif method in METHODS:
                result = getattr(planner, method)(*args, **kwargs)
            else:
                raise ValueError(f'unknown method {method}')
            torch.cuda.synchronize()
            resp = {'id': req.get('id'), 'ok': True, 'result': encode(result), 'seconds': time.monotonic()-t}
        except Exception as exc:  # reported to the caller, which raises it
            resp = {'id': req.get('id'), 'ok': False, 'error': f'{type(exc).__name__}: {exc}',
                    'traceback': traceback.format_exc(), 'seconds': time.monotonic()-t}
        proto.write(json.dumps(resp)+'\n')


if __name__ == '__main__':
    main()
