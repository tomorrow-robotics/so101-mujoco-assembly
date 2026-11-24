"""Real CuRobo planning on the GPU pod for a Q0 episode simulated on the devbox.

One devbox-initiated ssh process runs ``scripts/q0_curobo_server.py`` in the pod's CuRobo venv
(scripts/pod_setup_curobo_env.sh); it holds one upstream ``CuroboPlanner`` per arm, built exactly as upstream
``LazyPlanner`` does (base-frame planning, robot_origin_pose = identity, dt = control_dt, the table cuboid top at
TABLE_SURFACE_Z - base_z) from the Panthera config in controller/calibration_panthera/curobo/.
``PodCuroboPlanner`` exposes the upstream call signatures and result dicts (solve_ik_to_joint, plan_joint,
plan_path, plan_batch); arrays travel as JSON lists over the pipe. No port is opened; nothing goes through HDFS.

Like upstream ``LazyPlanner``, the first use checks the CuRobo flange FK (+ GRASP_POINT_OFFSET_M along tool X)
against the backend's native FK (MuJoCo site <arm>_tcp) and refuses to plan on a mismatch.
"""
import hashlib
import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .pod_render import POD_ROOT, ssh_command
from ..controller.robot_geometry import GRASP_POINT_OFFSET_M

CALIBRATION = Path(__file__).resolve().parents[1]/'controller/calibration_panthera/curobo'
POD_PYTHON = '/tmp/q0_curobo_venv/bin/python'
FK_POSITION_TOL_M, FK_ROTATION_TOL_RAD = .0005, .002  # upstream LazyPlanner check


def quat_wxyz_matrix(q):
    """(...,4) wxyz -> (...,3,3); the devbox SciPy predates Rotation.from_quat(scalar_first=True)."""
    q = np.asarray(q, float)
    return Rotation.from_quat(np.roll(q.reshape(-1, 4), -1, axis=1)).as_matrix().reshape(q.shape[:-1]+(3, 3))


def _arrays(x):
    if isinstance(x, dict):
        return {k: _arrays(v) for k, v in x.items()}
    if isinstance(x, list) and x and all(isinstance(v, (int, float)) for v in x):
        return np.asarray(x, float)
    if isinstance(x, list) and x and all(isinstance(v, list) for v in x):
        try:
            return np.asarray(x, float)
        except (TypeError, ValueError):
            return [_arrays(v) for v in x]
    if isinstance(x, list) and x and all(isinstance(v, str) for v in x):
        return np.asarray(x, dtype=object)
    return x


def _jsonable(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


class PodCuroboService:
    """One ssh pipe, one CuroboPlanner per arm on the pod."""

    def __init__(self, arms, job=None, gpu=None, yml=None, log_path=None, command=None):
        """arms: {arm: {'joint_names': [...], 'table_height_in_base': z, 'dt': s}}.
        ``command`` replaces the ssh command (tests / running directly on a GPU host)."""
        self.yml = Path(yml or CALIBRATION/'curobo.yml').resolve()
        self.yml_sha256 = hashlib.sha256(self.yml.read_bytes()).hexdigest()
        gpu = os.environ.get('Q0_CUROBO_GPU', '7') if gpu is None else gpu
        job = job or os.environ['POD_JOB']
        spec = json.dumps(arms)
        if command is None:
            remote = (f'cd {POD_ROOT} && export CUDA_VISIBLE_DEVICES={gpu} PYTHONNOUSERSITE=1 && unset PYTHONPATH && '
                      f"exec {POD_PYTHON} -u scripts/q0_curobo_server.py '{spec}' {self.yml}")
            command = ssh_command(job)+[remote]
        self.log_path = Path(log_path or f'/tmp/q0_curobo_client_{os.getpid()}.log')
        self._log = self.log_path.open('ab')
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log,
                                     bufsize=0)
        self._lock = threading.Lock()
        self._next = 0
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError(f'CuRobo pod service did not start; see {self.log_path}')
        hello = json.loads(line)
        if hello.get('yml_sha256') != self.yml_sha256:
            raise RuntimeError(f'pod CuRobo config differs from the devbox copy: {hello.get("yml_sha256")}')
        self.info = hello
        self.latency = []  # (arm, method, wall seconds incl. pipe, server seconds)

    def call(self, arm, method, *args, **kwargs):
        with self._lock:
            self._next += 1
            req = {'id': self._next, 'arm': arm, 'method': method, 'args': _jsonable(list(args)),
                   'kwargs': _jsonable(kwargs)}
            t = time.monotonic()
            self.proc.stdin.write((json.dumps(req)+'\n').encode())
            self.proc.stdin.flush()
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError(f'CuRobo pod service closed; see {self.log_path}')
            resp = json.loads(line)
            self.latency.append((arm, method, time.monotonic()-t, resp.get('seconds')))
        if resp.get('id') != req['id']:
            raise RuntimeError('CuRobo pod service protocol out of sync')
        if not resp['ok']:
            raise RuntimeError(f'pod CuroboPlanner.{method} failed: {resp["error"]}')
        return resp['result']

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=30)
        except Exception:
            self.proc.kill()
            self.proc.wait()
        self.proc.stdout.close()
        self._log.close()


class PodCuroboPlanner:
    """Upstream CuroboPlanner call surface for one arm (the object behind upstream ``backend.planners[arm]``)."""

    def __init__(self, service, arm):
        self.service, self.arm = service, arm
        info = service.info['arms'][arm]
        self.frame_bias = np.asarray(info['frame_bias'], float)
        self.ee_link, self.tool_frames = info['ee_link'], info['tool_frames']

    def _status_result(self, result):
        return _arrays(result)

    def solve_ik_to_joint(self, curr_joint_pos, target_ee_pose, real_robot_pose, num_seeds=32):
        r = self._status_result(self.service.call(self.arm, 'solve_ik_to_joint', curr_joint_pos, target_ee_pose,
                                                  real_robot_pose, num_seeds=num_seeds))
        if r.get('status') == 'Success':
            cands = np.asarray(r['joint_candidates'], float).reshape(-1, len(np.ravel(curr_joint_pos)))
            r['joint_candidates'] = list(cands)
            r['joint_value'] = np.asarray(r['joint_value'], float)
        return r

    def plan_joint(self, start_joint_pos, goal_joint_pos):
        return self._status_result(self.service.call(self.arm, 'plan_joint', start_joint_pos, goal_joint_pos))

    def plan_path(self, curr_joint_pos, target_ee_pose, real_robot_pose, constraint_pose=None):
        return self._status_result(self.service.call(self.arm, 'plan_path', curr_joint_pos, target_ee_pose,
                                                     real_robot_pose, constraint_pose=constraint_pose))

    def plan_batch(self, curr_joint_pos, target_ee_pose_list, real_robot_pose, constraint_pose=None):
        return self._status_result(self.service.call(self.arm, 'plan_batch', curr_joint_pos, target_ee_pose_list,
                                                     real_robot_pose, constraint_pose=constraint_pose))

    def tool_fk_base(self, qs):
        """CuRobo link6 poses in the arm base frame for an (N,6) batch: (positions (N,3), quaternions wxyz (N,4))."""
        r = self.service.call(self.arm, 'fk', np.asarray(qs, float).reshape(-1, 6))
        return np.asarray(r['position'], float), np.asarray(r['quaternion_wxyz'], float)


def reference_fk(planner, origin_pose, qs):
    """Upstream Backend.reference_fk for a batch: CuRobo grasp point and rotation in world."""
    p, quat = planner.tool_fk_base(qs)
    origin = np.asarray(origin_pose, float).reshape(7)
    base_r = quat_wxyz_matrix(origin[3:])
    world_r = base_r @ quat_wxyz_matrix(quat)
    flange = origin[:3] + (base_r @ p.T).T
    return flange + GRASP_POINT_OFFSET_M*world_r[:, :, 0], world_r


def check_reference_fk(backend, arm, planner):
    """Upstream LazyPlanner: refuse to plan when the bundled URDF / CuRobo FK disagrees with the backend FK."""
    q = backend.joints()[arm]
    p, r = backend.fk(arm, q)
    ref_p, ref_r = reference_fk(planner, backend.robots[arm].entity_origin_pose, q[None])
    dp = float(np.linalg.norm(p-ref_p[0]))
    dr = float(Rotation.from_matrix(r.T @ ref_r[0]).magnitude())
    if dp > FK_POSITION_TOL_M or dr > FK_ROTATION_TOL_RAD:
        raise RuntimeError(f'Panthera URDF/CuRobo FK mismatch ({arm}: {dp*1000:.3f} mm, {math.degrees(dr):.3f} deg)')
    return dp, dr


def start_backend_planners(backend, job=None, gpu=None, yml=None, log_path=None, command=None):
    """Planner service for a NativeMujocoBackend: {arm: PodCuroboPlanner}, FK-checked like upstream."""
    arms = {}
    for a, v in backend.arm.items():
        origin = np.asarray(backend.robots[a].entity_origin_pose, float)
        base_z_axis = quat_wxyz_matrix(origin[3:])[:, 2]
        if base_z_axis[2] < 1-1e-6:
            raise RuntimeError(f'{a} base is tilted; upstream table height in base frame assumes an upright base')
        arms[a] = {'joint_names': [n.split('_', 1)[1] for n in backend.joint_limits[a].names],
                   # CuRobo expects the top surface in the arm's base frame (upstream LazyPlanner).
                   'table_height_in_base': float(backend.table_z-origin[2]), 'dt': backend.control_dt}
    service = PodCuroboService(arms, job=job, gpu=gpu, yml=yml, log_path=log_path, command=command)
    planners = {a: PodCuroboPlanner(service, a) for a in arms}
    try:
        for a, p in planners.items():
            check_reference_fk(backend, a, p)
    except Exception:
        service.close()
        raise
    return service, planners
