"""Upstream CuroboPlanner (controller/curobo_planner.py, byte-identical to Q0_Preview @6baccd5) for Panthera-HT.

Why a subclass at all: upstream ``CuroboPlanner.__init__`` hard-codes the robot files
(``yml['robot_cfg']['kinematics']['urdf_path'] = <its dir>/calibration/X5A.urdf`` and ``asset_root_path``)
right before it calls ``_build_robot_and_scene_cfg``. Everything else is already parameterised (yml_path, frames,
table height, dt). The only override below restores the urdf/asset paths named by the Panthera yml; the
config translation, table cuboid, table-mount filter, solver settings, warmup and every public method
(plan_path, plan_joint, plan_batch, solve_ik_to_joint) are the unchanged upstream code.
Imports CuRobo: pod side only (scripts/q0_curobo_server.py).
"""
from pathlib import Path

import yaml

from ..controller.curobo_planner import CuroboPlanner

CALIBRATION = Path(__file__).resolve().parents[1]/'controller/calibration_panthera/curobo'
DEFAULT_YML = CALIBRATION/'curobo.yml'


class PantheraCuroboPlanner(CuroboPlanner):
    def _build_robot_and_scene_cfg(self, yml_data, table_height):
        own = yaml.safe_load(Path(self.yml_path).read_text())['robot_cfg']['kinematics']
        base = Path(self.yml_path).resolve().parent
        kinematics = yml_data['robot_cfg']['kinematics']
        kinematics['urdf_path'] = str(base/own['urdf_path'])
        kinematics['asset_root_path'] = str(base/own.get('asset_root_path', '.'))
        return super()._build_robot_and_scene_cfg(yml_data, table_height)


def make_planner(joint_names, control_dt, table_height_in_base, yml_path=DEFAULT_YML):
    """Exactly upstream LazyPlanner's construction: base-frame planning (robot_origin_pose = identity)."""
    return PantheraCuroboPlanner(robot_origin_pose=[0, 0, 0, 1, 0, 0, 0], active_joints_name=list(joint_names),
                                 all_joints=list(joint_names), dt=control_dt, yml_path=str(yml_path),
                                 table_height=float(table_height_in_base))


def tool_pose_in_base(planner, q):
    """Upstream Backend.reference_fk before the base transform: CuRobo link6 pose in the arm base frame."""
    import numpy as np
    kin = planner.motion_planner.compute_kinematics(planner._build_joint_state(q))
    tool = kin.tool_poses.to_dict()[planner.ee_link]
    p = np.asarray(tool.position.detach().cpu(), float).reshape(-1)[:3] - np.asarray(planner.frame_bias)
    quat = np.asarray(tool.quaternion.detach().cpu(), float).reshape(-1)[:4]
    return p, quat
