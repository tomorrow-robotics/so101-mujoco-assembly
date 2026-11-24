"""The control interface between the Q0 harness and a real two-arm Panthera-HT cell.

Implement `RobotIO` (and `CameraIO`, unless the default RealSense one fits) on top of your own SDK and pass the
classes to `real/run_real_episode.py --robot pkg.module:Class --cameras pkg.module:Class`. Everything else -- the
agent, the plan protocol, the executor, IK / path planning, the 2 mm inter-arm check, observations, depth files for
the agent's pose tools, the fastening signal -- is already wired in `real/backend.py`.

Conventions (the same as the simulator, so the agent's experience and skills carry over):
- arms are 'left' and 'right' (left = arm "a" with wrist camera 1868, right = arm "b" with wrist camera 1416);
- joint vectors are the 6 arm joints joint1..joint6 in radians, with the zero and sign convention of the SDK URDF
  that `twin/twin.xml` was built from (check with `real/check_setup.py`, which compares your FK with the twin's);
- world = table frame of `twin/twin.xml` (metres, z up, table top at z = 0);
- gripper commands are normalised: 1.0 = fully open, 0.0 = fully closed (closing on a part = 0.0 with your
  grasp force limit).
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

ARMS = ('left', 'right')


class RobotIO(ABC):
    """Two arms + two parallel grippers.

    The harness calls `command` once per control step (25 Hz nominal) with the next joint target of a planned,
    speed-limited trajectory (joint speed <= 2 rad/s before `slowdown`), so consecutive targets are close together.
    Between steps -- for example while the model is thinking, which can take minutes -- the arms must keep holding
    the last target: implement holding in your SDK loop (gravity compensation + joint PD), not by waiting for calls.
    """

    @abstractmethod
    def read_joints(self) -> dict[str, np.ndarray]:
        """Measured joint angles {arm: (6,) rad}."""

    def read_gripper(self) -> dict[str, float | None]:
        """Measured opening {arm: 0..1, 1 = open} if your gripper reports it, else None (the command is used)."""
        return {a: None for a in ARMS}

    @abstractmethod
    def command(self, q: dict[str, np.ndarray], gripper: dict[str, float], duration_s: float) -> None:
        """Track the joint targets q {arm: (6,) rad} and gripper commands {arm: 0..1} and return after duration_s.

        Interpolate from the previous target inside your own control loop (e.g. 200-1000 Hz) so the motion is
        smooth, then keep holding q. Raise an exception (or call `stop()` yourself) on any safety fault: tracking
        error, over-current, joint limit, communication loss. The episode then ends.
        """

    def command_compliant(self, arm: str, p_d: np.ndarray, R_d: np.ndarray, press_axis: np.ndarray,
                          max_force_n: float, q_other: np.ndarray, gripper: dict[str, float], duration_s: float,
                          jacobian: np.ndarray, tcp: tuple[np.ndarray, np.ndarray]) -> None:
        """Force-limited Cartesian motion of one arm for one control step (the plan's `compliance` groups; step 1
        uses it for the final push of M1 into the Base, max_force_n = 10 N).

        Follow the TCP reference (p_d, R_d) in world softly: along press_axis never push harder than max_force_n,
        sideways yield to contact so the part can centre itself, keep the orientation firm. The other arm keeps the
        joint target q_other. `jacobian` (6x6, world, TCP, [linear; angular]) and `tcp` (measured position, rotation)
        come from the twin at the current joints, for convenience.

        The simulator implements this as joint-torque Cartesian impedance (see REPORT.md section 2 for the law and
        gains). Default here: NotImplementedError -> the backend falls back to stiff position control and logs it,
        which is NOT safe for a real press; implement this before running the insertion on hardware.
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop safely and hold (or brake if your hardware has a safe brake). Called on faults and at the end."""

    def close(self) -> None:
        """Release the connection. The arms should stay held until the operator decides otherwise."""


class CameraIO(ABC):
    """The three RGB-D cameras the agent sees: cam_head, cam_left_wrist, cam_right_wrist."""

    @abstractmethod
    def capture(self) -> dict[str, dict]:
        """{camera: {'color': (H,W,3) uint8 RGB, 'depth': (H,W) float32 metres along the optical axis, aligned to
        the colour image, 0 where invalid, 'K': (3,3) OpenCV intrinsics of the colour image}}.

        Optional per camera: 'camera_to_world' (4x4, OpenGL axes: +X right, +Y up, -Z forward) if you calibrate the
        cameras yourself; otherwise the backend uses the twin: the head camera is fixed in the table frame, the
        wrist cameras ride on link6 with the calibrated mount, posed at the measured joints.
        """

    def close(self) -> None:
        pass
