"""Template: connect your own (modified) Panthera-HT SDK to the harness. Copy, fill in the TODOs, then run

    python real/check_setup.py --robot real.adapter_template:MyRobot --out runs/check_001

Keep the conventions of real/interface.py: arms 'left'/'right', joints joint1..6 in rad in the twin's URDF convention,
gripper 1 = open / 0 = closed.
"""
import threading
import time

import numpy as np

from real.interface import ARMS, RobotIO


class MyRobot(RobotIO):
    def __init__(self, left_config='TODO/left.yaml', right_config='TODO/right.yaml', rate_hz=200.):
        # TODO: open both arms with your SDK, e.g. self.sdk = {'left': YourArm(left_config), 'right': YourArm(right_config)}
        self.sdk = {}
        self.rate = float(rate_hz)
        self.lock = threading.Lock()
        q = self.read_joints()
        self.goal = {a: q[a].copy() for a in ARMS}       # what the control loop holds between harness calls
        self.grip = {a: 1. for a in ARMS}
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # --- what the harness calls ---------------------------------------------------------------------------------
    def read_joints(self):
        # TODO: return {a: np.asarray(self.sdk[a].get_joint_positions(), float) for a in ARMS}
        raise NotImplementedError

    def command(self, q, gripper, duration_s):
        start = {a: self.goal[a].copy() for a in ARMS}
        t0 = time.monotonic()
        while True:                                     # hand the loop an interpolated goal over duration_s
            u = min(1., (time.monotonic()-t0)/duration_s)
            with self.lock:
                for a in ARMS:
                    self.goal[a] = (1-u)*start[a] + u*np.asarray(q[a], float)
                    self.grip[a] = float(gripper[a])
            if u >= 1.:
                break
            time.sleep(1./self.rate)

    # def command_compliant(self, arm, p_d, R_d, press_axis, max_force_n, q_other, gripper, duration_s, jacobian, tcp):
    #     TODO (required before the insertion on hardware): torque-mode Cartesian impedance in your loop, see
    #     real/compliance.py (impedance_torque) and real/mock.py (MockRobot.command_compliant) for the reference.

    def stop(self):
        with self.lock:                                 # hold where the arms are now
            q = self.read_joints()
            self.goal = {a: q[a].copy() for a in ARMS}

    def close(self):
        self.running = False
        self.thread.join(timeout=1.)

    # --- your high-rate loop --------------------------------------------------------------------------------------
    def _loop(self):
        dt = 1./self.rate
        while self.running:
            t = time.monotonic()
            with self.lock:
                goal = {a: self.goal[a].copy() for a in ARMS}
                grip = dict(self.grip)
            for a in ARMS:
                # TODO: joint PD + gravity feed-forward toward goal[a], e.g.
                #   self.sdk[a].send(pos=goal[a], vel=0, tau=self.sdk[a].gravity(q), kp=KP, kd=KD)
                #   self.sdk[a].gripper(OPEN_POS*grip[a], max_torque=GRIP_TORQUE)
                # TODO: safety: stop if |q - goal| is large, joint speed or current is too high, or a read fails
                pass
            time.sleep(max(0., dt-(time.monotonic()-t)))
