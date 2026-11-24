"""Reference force-limited Cartesian impedance: the law the simulator used for every successful step-1 press.

Port this into your SDK's high-rate loop for `RobotIO.command_compliant` (joint torque mode with gravity
compensation). `real/mock.py` runs exactly this function on the twin, so the mock and the simulator agree.

    F_a = clip(K_a e_a - D_a v_a, -F_max, F_max)        along the press axis (force-limited)
    F   = F_a axis + K_l e_lat - D_l v_lat              sideways: soft, the part centres itself
    M   = K_r rotvec(R_d R^T) - D_r w                   orientation: firm
    tau = J^T [F; M] + g(q) - 1.0 qdot + (I - J+ J) 5 (q_null - q)

e = p_d - p (TCP position error, world), v and w = TCP linear / angular velocity (J qdot), q_null = the joints when
the press began. No force sensor is used: only commanded torque and joint state.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Gains of the simulator's group compliance (q0_harness/adapters/mujoco_backend.py: IMPEDANCE + GROUP_COMPLIANCE_AXIAL).
# SI units: N/m, N s/m, N m/rad, N m s/rad. F_max is replaced by the plan's max_force_n (10 N in step 1).
GAINS = dict(K_a=20000., D_a=200., F_max=10., K_l=250., D_l=25., K_r=150., D_r=1.1)


def impedance_torque(J, qdot, q, q_null, p, R, p_d, R_d, axis, gravity, gains=GAINS, joint_damping=1.0, null_kp=5.):
    """J: 6x6 TCP Jacobian in world ([linear; angular]); qdot, q, q_null: (6,); p, R: measured TCP; p_d, R_d: the
    reference; axis: unit press direction (world); gravity: (6,) gravity torque g(q). Returns joint torques (6,)."""
    g, ax = gains, np.asarray(axis, float)/np.linalg.norm(axis)
    vel = J @ qdot
    e = np.asarray(p_d, float) - np.asarray(p, float)
    e_a, v_a = float(e @ ax), float(vel[:3] @ ax)
    F_a = float(np.clip(g['K_a']*e_a - g['D_a']*v_a, -g['F_max'], g['F_max']))
    F = F_a*ax + g['K_l']*(e - e_a*ax) - g['D_l']*(vel[:3] - v_a*ax)
    M = g['K_r']*Rotation.from_matrix(np.asarray(R_d) @ np.asarray(R).T).as_rotvec() - g['D_r']*vel[3:]
    Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4*np.eye(6))
    return (J.T @ np.r_[F, M] + np.asarray(gravity, float) - joint_damping*qdot
            + (np.eye(6) - Jp @ J) @ (null_kp*(np.asarray(q_null, float) - q)))
