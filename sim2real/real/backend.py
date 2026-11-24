"""RealBackend: the Q0 harness backend for a real two-arm cell.

The harness (Episode, Executor, motion search, station queries) talks to a backend with the interface of
q0_harness/adapters/mujoco_backend.NativeMujocoBackend. This class keeps that interface and splits it:
- hardware: joint commands, gripper, the compliant press and the cameras go to your RobotIO / CameraIO;
- twin (twin/twin.xml, both arms only): forward kinematics, IK, joint-path collision checks against the table and
  the arm itself, the 2 mm inter-arm check, wrist-camera extrinsics. The twin is posed at the *measured* joints
  after every step and before every observation.
"""
from __future__ import annotations

import copy
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from q0_harness.adapters.mujoco_backend import NativeMujocoBackend, MujocoPlanner, PREFIX
from q0_harness.adapters.mujoco_cameras import DEFAULT_CAMERAS, camera_to_world
from q0_harness.controller import robot_geometry as rg
from q0_harness.controller.executor import Ended, StepBudgetExhausted
from q0_harness.controller.joint_limits import JointLimits
from q0_harness.runtime.protocol import ARMS

ROOT = Path(__file__).resolve().parents[1]


class TwinCameraRig:
    """CameraIO frames + extrinsics -> the vision payload the harness expects (same keys as the simulator rig)."""
    depth = True

    def __init__(self, backend, cameras, names=None):
        self.b, self.io = backend, cameras
        self.names = dict(DEFAULT_CAMERAS if names is None else names)
        self.ids = {k: backend.model.camera(v).id for k, v in self.names.items()}
        self.backend = type(cameras).__name__
        self.info = {'cameras': self.names, 'io': self.backend}

    def capture(self, data):
        frames = self.io.capture()
        vision = {}
        for name, cid in self.ids.items():
            f = frames[name]
            T = f.get('camera_to_world')
            vision[name] = {'color': np.asarray(f['color'], np.uint8),
                            'depth': np.asarray(f['depth'], np.float32),
                            'intrinsic_matrix': np.asarray(f['K'], float),
                            'extrinsic_matrix': np.asarray(T, float) if T is not None else camera_to_world(data, cid)}
        return vision

    def close(self):
        self.io.close()


class RealBackend(NativeMujocoBackend):
    def __init__(self, model, data, robot, cameras, max_control_steps, control_dt=.04, slowdown=2.,
                 post_planning_hold_steps=50, instruction='', planner='mujoco', curobo_options=None, log=print):
        if type(max_control_steps) is not int or max_control_steps <= 0:
            raise ValueError('An explicit positive task control-step limit is required')
        self.model, self.data = model, data
        self.scratch = mujoco.MjData(model)
        self.robot, self.log = robot, log
        self.control_dt, self.slowdown, self.substeps = float(control_dt), float(slowdown), 1
        self.max_control_steps, self.steps, self.actions_sent = max_control_steps, 0, 0
        self.step_phase = 'initial'
        self.on_step_acknowledged = lambda: None
        self.instruction = instruction
        self.expose_objects = False
        self.post_hold_limit, self.post_hold = int(post_planning_hold_steps), 0
        self._cancelled, self._error = False, None
        self.physics_events, self.after_step_hooks, self.after_group_hooks = [], [], []
        self.sim_invalid = False
        self.compliance = self.group_compliance = None
        self.station_status = None
        self.last_group_compliant = False
        self.compliance_fallback_steps = 0
        self._jacp, self._jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        self.record, self.observation_index = [], 0
        self.arm, self.joint_limits = {}, {}
        for a, s in PREFIX.items():
            joints = [model.joint(f'{s}_joint{i}') for i in range(1, 7)]
            kinds = ['revolute' if model.jnt_limited[j.id] else 'continuous' for j in joints]
            bounds = [tuple(model.jnt_range[j.id]) if model.jnt_limited[j.id] else (-math.inf, math.inf) for j in joints]
            self.joint_limits[a] = JointLimits([j.name for j in joints], kinds, bounds)
            bodies = {i for i in range(model.nbody) if model.body(i).name.startswith(s+'_')}
            geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies
                     and (model.geom_contype[g] or model.geom_conaffinity[g])]
            self.arm[a] = dict(prefix=s, qids=np.array([j.qposadr[0] for j in joints]),
                               vids=np.array([j.dofadr[0] for j in joints]),
                               grip=model.actuator(f'{s}_gripper').id, site=model.site(f'{s}_tcp').id,
                               finger_q=(model.joint(f'{s}_gripper_L_joint').qposadr[0],
                                         model.joint(f'{s}_gripper_R_joint').qposadr[0]),
                               bodies=bodies, geoms=geoms, base=model.body(f'{s}_base_link').id)
        self.table = model.geom('lab_table').id
        mujoco.mj_forward(model, data)
        normal = data.geom_xmat[self.table].reshape(3, 3)[:, 2]
        self.table_z = float((data.geom_xpos[self.table] + model.geom_size[self.table, 2]*normal)[2])
        rg.TABLE_SURFACE_Z = rg.TOOL_TABLE_GUARD_Z = self.table_z
        self.ctrl_max = {a: float(model.actuator_ctrlrange[v['grip'], 1]) for a, v in self.arm.items()}
        g0 = robot.read_gripper()
        self.cmd_g = {a: (1. if g0.get(a) is None else float(g0[a])) for a in ARMS}
        self._sync()
        self.targets = self.joints()
        self.held_width_mm = {a: None for a in ARMS}
        self._collision_ranges = None
        self.collision_context, self.last_collision = {}, None
        self.robots = {a: SimpleNamespace(entity_origin_pose=np.r_[data.xpos[v['base']], data.xquat[v['base']]])
                       for a, v in self.arm.items()}
        self.planner_kind, self.curobo_service = planner, None
        if planner == 'curobo':
            self.curobo_service, self.planners = _local_curobo(self, **(curobo_options or {}))
        elif planner == 'mujoco':
            self.planners = {a: MujocoPlanner(self, a) for a in ARMS}
        else:
            raise ValueError(f'unknown planner {planner!r}')
        self.path_diagnostics = {}
        self.origin_poses = {a: self.pose(a) for a in ARMS}   # 'home' = where the arms are at launch
        self.rig = TwinCameraRig(self, cameras)

    # --- twin <- hardware ---------------------------------------------------------------------------------------
    def _sync(self):
        q = self.robot.read_joints()
        meas = self.robot.read_gripper()
        d = self.data
        for a, v in self.arm.items():
            d.qpos[v['qids']] = np.asarray(q[a], float)
            g = self.cmd_g[a] if meas.get(a) is None else float(meas[a])
            closure = float(np.clip((rg.FULL_OPEN_GAP_MM-rg.jaw_gap_mm(g))/2000., 0, self.ctrl_max[a]))
            d.qpos[v['finger_q'][0]], d.qpos[v['finger_q'][1]] = -closure, closure
        d.qvel[:] = 0
        mujoco.mj_fwdPosition(self.model, d)

    def joints(self):
        self._sync()
        return {a: self.data.qpos[v['qids']].copy() for a, v in self.arm.items()}

    def sim_time(self):
        return self.steps*self.control_dt

    # --- actuation ------------------------------------------------------------------------------------------------
    def _fault(self, exc):
        self.physics_events.append({'step': self.steps, 'hardware_fault': repr(exc), 'unix_s': time.time()})
        self.sim_invalid = True
        self._cancelled = True
        try:
            self.robot.stop()
        finally:
            raise Ended()

    def step(self, q, g):
        if self._cancelled:
            raise Ended()
        if self.remaining_steps() == 0:
            raise StepBudgetExhausted('task_control_step_limit')
        if any(not np.isfinite(g[a]) or not 0 <= g[a] <= 1 for a in ARMS):
            raise ValueError('Invalid normalized gripper command')
        for a in ARMS:
            self.joint_limits[a].validate(q[a])
        self.actions_sent += 1
        self.cmd_g = dict(g)
        self.targets = {a: np.asarray(q[a], float).copy() for a in ARMS}
        dur = self.control_dt*self.slowdown
        t0 = time.monotonic()
        gc = self.group_compliance
        try:
            if gc is not None:
                arm = gc['arm']; other = 'left' if arm == 'right' else 'right'
                p_d, R_d = self.fk(arm, self.targets[arm])
                v = self.arm[arm]
                mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, v['site'])
                J = np.vstack([self._jacp[:, v['vids']], self._jacr[:, v['vids']]])
                try:
                    self.robot.command_compliant(arm, p_d, R_d, gc['axis'], gc['gains']['F_max'], self.targets[other],
                                                 dict(g), dur, J, self.pose(arm))
                except NotImplementedError:
                    if self.compliance_fallback_steps == 0:
                        self.log('WARNING: RobotIO.command_compliant is not implemented; the press runs as stiff '
                                 'position control (unsafe on hardware).')
                    self.compliance_fallback_steps += 1
                    self.robot.command(self.targets, dict(g), dur)
            else:
                self.robot.command(self.targets, dict(g), dur)
        except Ended:
            raise
        except Exception as exc:   # noqa: BLE001 - any hardware fault ends the episode
            self._fault(exc)
        self._sync()
        self.steps += 1
        meas = {a: self.data.qpos[v['qids']].copy() for a, v in self.arm.items()}
        self.record.append({'step': self.steps, 'phase': self.step_phase, 'wall_s': time.monotonic()-t0,
                            'unix_s': time.time(), 'compliant': gc is not None,
                            'q_left': self.targets['left'], 'q_right': self.targets['right'],
                            'qm_left': meas['left'], 'qm_right': meas['right'],
                            'g_left': float(g['left']), 'g_right': float(g['right'])})
        for hook in self.after_step_hooks:
            hook(self.steps)
        self.on_step_acknowledged()
        if self.step_phase == 'post_planning_hold':
            self.post_hold += 1
            if self.post_hold >= self.post_hold_limit:
                self._cancelled = True
                raise Ended()

    def attach_executor(self, executor):
        super().attach_executor(executor)          # group['compliance'] + after_group_hooks (station)
        inner = executor._execute_group

        def execute_group(group):
            self.last_group_compliant = group.get('compliance') is not None
            return inner(group)
        executor._execute_group = execute_group
        return executor

    def save_record(self, path):
        r = self.record
        if not r:
            return
        np.savez_compressed(path, step=np.array([x['step'] for x in r]), phase=np.array([x['phase'] for x in r]),
                            unix_s=np.array([x['unix_s'] for x in r]), wall_s=np.array([x['wall_s'] for x in r]),
                            compliant=np.array([x['compliant'] for x in r]),
                            q_target=np.stack([np.r_[x['q_left'], x['q_right']] for x in r]),
                            q_measured=np.stack([np.r_[x['qm_left'], x['qm_right']] for x in r]),
                            gripper_cmd=np.array([[x['g_left'], x['g_right']] for x in r]),
                            control_dt=self.control_dt, slowdown=self.slowdown)

    # --- observation ------------------------------------------------------------------------------------------------
    def observation(self):
        self._sync()
        obs, vision = super().observation()
        obs['sim_time_source'] = 'relative time from acknowledged action steps and supplied control frequency'
        return obs, vision

    def close(self):
        self._cancelled = True
        try:
            self.robot.stop()
        finally:
            self.rig.close()
            self.robot.close()
            if self.curobo_service is not None:
                self.curobo_service.close()


def _local_curobo(backend, gpu=None, log_path=None, python=None):
    """cuRobo on this machine's GPU (needs cuRobo 895c651 installed): the same server the simulator used on the pod."""
    import json
    import os
    from q0_harness.adapters.pod_curobo import start_backend_planners, quat_wxyz_matrix, CALIBRATION
    arms = {}
    for a, v in backend.arm.items():
        origin = np.asarray(backend.robots[a].entity_origin_pose, float)
        if quat_wxyz_matrix(origin[3:])[:, 2][2] < 1-1e-6:
            raise RuntimeError(f'{a} base is tilted; cuRobo table height assumes an upright base')
        arms[a] = {'joint_names': [n.split('_', 1)[1] for n in backend.joint_limits[a].names],
                   'table_height_in_base': float(backend.table_z-origin[2]), 'dt': backend.control_dt}
    if gpu is not None:
        os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu)
    cmd = [python or sys.executable, '-u', str(ROOT/'scripts/q0_curobo_server.py'), json.dumps(arms),
           str(CALIBRATION/'curobo.yml')]
    return start_backend_planners(backend, log_path=log_path, command=cmd)
