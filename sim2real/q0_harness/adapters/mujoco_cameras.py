"""Native MuJoCo cameras -> Q0 observation vision payload (RGB, K, camera-to-world).

Contract: q0_harness/docs/OBSERVATION_INTERFACE.md (unchanged upstream document).
- RGB: uint8 (H,W,3), rendered from the current MjData at capture time.
- K: pinhole intrinsics in pixels, OpenCV convention (integer pixel index = pixel centre).
- extrinsic_matrix: camera-to-world in USD/OpenGL camera axes (+X right, +Y up, -Z forward).
  MuJoCo camera frames already use these axes, so T = [cam_xmat | cam_xpos].
Wrist cameras are re-read from MjData at every capture, never cached.
A wrist camera does not see its own housing: CameraRig renders from a render-only model copy in which, per camera,
the housing visuals (group-2, non-colliding geoms on the camera's own arm) get alpha 0, which MuJoCo skips.
"""
import copy
import os
import numpy as np
import mujoco

# Q0 camera name -> native camera name in the bimanual lab scene (arm a = left, arm b = right).
DEFAULT_CAMERAS = {'cam_head': 'agent', 'cam_left_wrist': 'wrist_1868', 'cam_right_wrist': 'wrist_1416'}


def intrinsics(model, camera_id, width, height):
    """Pixel intrinsics as MuJoCo renders them (verified by tests/test_q0_cameras.py)."""
    i = camera_id
    if model.cam_sensorsize[i][0] > 0:
        fx, fy, px, py = model.cam_intrinsic[i]
        sx, sy = model.cam_sensorsize[i]
        # MuJoCo principal offset is measured opposite to image u/v; continuous centre = W/2 - px.
        fxp, fyp = fx/sx*width, fy/sy*height
        cx, cy = width/2 - px/sx*width, height/2 - py/sy*height
    else:
        fyp = height/2/np.tan(np.radians(model.cam_fovy[i])/2)
        fxp, cx, cy = fyp, width/2, height/2
    # Continuous image coordinates put pixel centres at index+0.5; OpenCV puts them at index.
    return np.array([[fxp, 0, cx-.5], [0, fyp, cy-.5], [0, 0, 1.]])


def camera_to_world(data, camera_id):
    T = np.eye(4)
    T[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3)
    T[:3, 3] = data.cam_xpos[camera_id]
    return T


def own_housing_geoms(model, camera_id):
    """Group-2 visual-only geoms on the camera body or its ancestors (world excluded): the camera's own housing."""
    chain, b = set(), int(model.cam_bodyid[camera_id])
    while b > 0:
        chain.add(b); b = int(model.body_parentid[b])
    return [g for g in range(model.ngeom) if model.geom_group[g] == 2 and int(model.geom_bodyid[g]) in chain
            and model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0]


class CameraRig:
    def __init__(self, model, cameras=None, width=640, height=480, depth=False):
        self.model, self.width, self.height = model, int(width), int(height)
        self.depth = bool(depth)   # also return 'depth' (float32 metres along the camera's viewing axis)
        self.cameras = dict(DEFAULT_CAMERAS if cameras is None else cameras)
        self.ids = {k: model.camera(v).id for k, v in self.cameras.items()}
        if model.vis.global_.offwidth < self.width or model.vis.global_.offheight < self.height:
            raise ValueError('Offscreen buffer smaller than requested image size')
        self.hidden = {k: own_housing_geoms(model, i) for k, i in self.ids.items()}
        self.render_model = copy.copy(model)  # render-only; physics model, collision and calibration untouched
        self.renderer = mujoco.Renderer(self.render_model, self.height, self.width)
        self.backend = os.environ.get('MUJOCO_GL', '')

    def capture(self, data):
        vision = {}
        rgba = self.render_model.geom_rgba
        for name, i in self.ids.items():
            hide = self.hidden[name]
            rgba[hide, 3] = 0
            self.renderer.update_scene(data, camera=i)
            rgb = self.renderer.render().copy()
            if self.depth:
                self.renderer.enable_depth_rendering()
                dep = self.renderer.render().astype(np.float32).copy()
                self.renderer.disable_depth_rendering()
            rgba[hide, 3] = self.model.geom_rgba[hide, 3]
            vision[name] = {'color': rgb, 'intrinsic_matrix': intrinsics(self.model, i, self.width, self.height),
                            'extrinsic_matrix': camera_to_world(data, i)}
            if self.depth:
                vision[name]['depth'] = dep
        return vision

    def close(self):
        self.renderer.close()
