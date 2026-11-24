"""Panthera command geometry (legacy X5 planner is not used); no motion skills or simulator imports."""
import math
import numpy as np
from typing import Any

ARMS = ("left", "right")
import json
from pathlib import Path
# Measured on the compiled native Panthera model by scripts/q0_calibrate_panthera.py
# (replaces upstream's X5 constants; same names and semantics).
_CAL = json.loads((Path(__file__).with_name('calibration_panthera')/'gripper.json').read_text())
# p0 is the -jaw finger, p1 the +jaw finger, for either arm (upstream: link8, link7).
POSE_FINGER_LINKS = tuple(_CAL['pose_finger_links'])
#: Distance from link6 (flange) to the native TCP site along the tool axis.
GRASP_POINT_OFFSET_M = float(_CAL['tcp_to_flange_m'])
#: The pad front edge reaches this far past the grasp point along the tool axis.
FINGERTIP_REACH_M = float(_CAL['fingertip_reach_m'])
FRONT_TIP_OFFSET_LOCAL_M = tuple(_CAL['front_tip_offset_local_m'])
DISTAL_PAD_OFFSET_LOCAL_M = tuple(_CAL['distal_pad_bbox_center_offset_local_m'])
FULL_OPEN_GAP_MM = float(_CAL['full_open_gap_mm'])
EMPTY_CLOSED_GAP_MM = float(_CAL['empty_closed_gap_mm'])
_GAP_COMMANDS = np.array([x['command'] for x in _CAL['dynamic_empty_gripper']])
_GAP_MM = np.array([x['steady_gap_mm'] for x in _CAL['dynamic_empty_gripper']])
# The table plane is scene-specific; the native backend reads the compiled lab_table and sets these.
TABLE_SURFACE_Z = None
TOOL_TABLE_GUARD_Z = None


def table_collision_cuboid(surface_z, dims=(3.0, 3.0, 0.05)):
    return {'dims': list(dims), 'pose': [0., 0., float(surface_z)-dims[2]/2, 1., 0., 0., 0.]}


def jaw_gap_mm(gripper: float) -> float:
    """Measured empty-gripper inner pad gap for a normalized command (1 open, 0 closed)."""
    return float(np.interp(float(np.clip(gripper, 0.0, 1.0)), _GAP_COMMANDS, _GAP_MM))


def gripper_for_gap(gap_mm: float) -> float:
    return float(np.clip(np.interp(float(gap_mm), _GAP_MM, _GAP_COMMANDS), 0.0, 1.0))


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat_wxyz(m: np.ndarray, reference_quaternion=None) -> np.ndarray:
    """Convert a rotation, choosing its quaternion sign near an optional reference.

    q and -q are the SAME pose. Hemisphere selection must never swap finger points
    or rotate the physical jaw by 180 degrees. Without a reference, use w >= 0.
    """
    m = np.asarray(m, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.asarray(q) / np.linalg.norm(q)
    if reference_quaternion is None:
        return q * (1.0 if q[0] >= 0 else -1.0)
    reference = np.asarray(reference_quaternion, dtype=float)
    if reference.shape != (4,) or not np.isfinite(reference).all() or np.linalg.norm(reference) < 1e-12:
        raise ValueError('reference quaternion must be a finite nonzero wxyz vector')
    return -q if float(q @ reference) < 0 else q


def grasp_frame(flange_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Grasp point and link6 rotation from a flange pose [x, y, z, qw, qx, qy, qz]."""
    pose = np.asarray(flange_pose, dtype=float).reshape(-1)
    rotation = quat_wxyz_to_matrix(pose[3:7])
    return pose[:3] + GRASP_POINT_OFFSET_M * rotation[:, 0], rotation


def flange_pose(grasp_point: np.ndarray, rotation: np.ndarray, reference_quaternion=None) -> np.ndarray:
    """Flange pose [x, y, z, qw, qx, qy, qz] that puts the grasp point at ``grasp_point``."""
    position = np.asarray(grasp_point, dtype=float) - GRASP_POINT_OFFSET_M * rotation[:, 0]
    return np.concatenate([position, matrix_to_quat_wxyz(rotation, reference_quaternion)])


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))
