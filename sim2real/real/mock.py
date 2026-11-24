"""Mock hardware: the twin itself, simulated, behind RobotIO / CameraIO.

Use it to test the whole pipeline (agent, plans, executor, backend, station prompt, files) without the robot:
    python real/run_real_episode.py --mock --smoke-agent --out runs/mock_smoke
The arms are the twin's (no SO-101 parts), driven like the simulator: joint PD + bias torque at 1 kHz, and the
compliant press with the reference law in real/compliance.py. Cameras render the twin (needs MUJOCO_GL).
"""
from __future__ import annotations

import copy

import mujoco
import numpy as np

from q0_harness.adapters.mujoco_cameras import CameraRig, DEFAULT_CAMERAS
from .compliance import GAINS, impedance_torque
from .interface import ARMS, CameraIO, RobotIO

PREFIX = {'left': 'a', 'right': 'b'}
ARM_KP = np.array([500, 650, 600, 350, 200, 130.])   # config/physics.yaml servo gains of the simulator
ARM_KD = np.array([35, 45, 35, 20, 12, 8.])


class MockRobot(RobotIO):
    def __init__(self, twin_xml, initial_q=None):
        self.model = mujoco.MjModel.from_xml_path(str(twin_xml))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.v = {}
        for a, s in PREFIX.items():
            joints = [m.joint(f'{s}_joint{i}') for i in range(1, 7)]
            self.v[a] = dict(qids=np.array([j.qposadr[0] for j in joints]), vids=np.array([j.dofadr[0] for j in joints]),
                             act=np.array([m.actuator(f'{s}_joint{i}_torque').id for i in range(1, 7)]),
                             grip=m.actuator(f'{s}_gripper').id, site=m.site(f'{s}_tcp').id)
            if initial_q is not None:
                self.data.qpos[self.v[a]['qids']] = initial_q[a]
        mujoco.mj_forward(m, self.data)
        self.target = {a: self.data.qpos[v['qids']].copy() for a, v in self.v.items()}
        self.grip = {a: 1. for a in ARMS}
        self.jacp, self.jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))

    def read_joints(self):
        return {a: self.data.qpos[v['qids']].copy() for a, v in self.v.items()}

    def _apply_grippers(self, gripper):
        for a, v in self.v.items():
            self.grip[a] = float(gripper[a])
            self.data.ctrl[v['grip']] = (1-self.grip[a])*self.model.actuator_ctrlrange[v['grip'], 1]

    def _pd(self, a, q_ref):
        v, d = self.v[a], self.data
        return d.qfrc_bias[v['vids']] + ARM_KP*(q_ref-d.qpos[v['qids']]) - ARM_KD*d.qvel[v['vids']]

    def _run(self, duration_s, torque_fn):
        m, d = self.model, self.data
        n = max(1, round(duration_s/m.opt.timestep))
        for k in range(n):
            for a, v in self.v.items():
                lim = m.actuator_ctrlrange[v['act']]
                d.ctrl[v['act']] = np.clip(torque_fn(a, (k+1)/n), lim[:, 0], lim[:, 1])
            mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            raise RuntimeError('mock physics diverged')

    def command(self, q, gripper, duration_s):
        start = {a: self.target[a].copy() for a in ARMS}
        goal = {a: np.asarray(q[a], float) for a in ARMS}
        self._apply_grippers(gripper)
        self._run(duration_s, lambda a, u: self._pd(a, (1-u)*start[a] + u*goal[a]))
        self.target = goal

    def command_compliant(self, arm, p_d, R_d, press_axis, max_force_n, q_other, gripper, duration_s, jacobian, tcp):
        other = 'left' if arm == 'right' else 'right'
        if getattr(self, '_press_arm', None) != arm:
            self._press_arm, self._q_null = arm, self.data.qpos[self.v[arm]['qids']].copy()
        gains = dict(GAINS, F_max=float(max_force_n))
        self._apply_grippers(gripper)
        m, d = self.model, self.data

        def torque(a, u):
            v = self.v[a]
            if a != arm:
                return self._pd(a, np.asarray(q_other, float))
            mujoco.mj_jacSite(m, d, self.jacp, self.jacr, v['site'])
            J = np.vstack([self.jacp[:, v['vids']], self.jacr[:, v['vids']]])
            p, R = d.site_xpos[v['site']], d.site_xmat[v['site']].reshape(3, 3)
            return impedance_torque(J, d.qvel[v['vids']], d.qpos[v['qids']], self._q_null, p, R, p_d, R_d, press_axis,
                                    d.qfrc_bias[v['vids']], gains)
        self._run(duration_s, torque)
        self.target[arm] = d.qpos[self.v[arm]['qids']].copy()
        self.target[other] = np.asarray(q_other, float)

    def stop(self):
        self._press_arm = None


class MockCameras(CameraIO):
    def __init__(self, robot: MockRobot, width=640, height=480):
        self.robot = robot
        # The twin keeps only collision geometry (a hidden geom group); show it in the mock images.
        m = copy.copy(robot.model)
        arm = (m.geom_contype > 0) | (m.geom_conaffinity > 0)
        m.geom_group[arm & (m.geom_group > 2)] = 1
        m.geom_rgba[arm & (m.geom_bodyid > 0)] = (.75, .75, .78, 1.)
        self.rig = CameraRig(m, DEFAULT_CAMERAS, width, height, depth=True)

    def capture(self):
        mujoco.mj_forward(self.robot.model, self.robot.data)
        out = {}
        for name, cam in self.rig.capture(self.robot.data).items():
            dep = cam['depth'].copy()
            dep[dep > 5.] = 0.   # like a real depth camera: no return = 0
            out[name] = {'color': cam['color'], 'depth': dep, 'K': cam['intrinsic_matrix']}
        return out

    def close(self):
        self.rig.close()
