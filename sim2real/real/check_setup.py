"""Check the real cell before an episode: joints, cameras, and whether the twin lines up with the images.

    python real/check_setup.py --robot my_sdk_adapter:MyRobot --out runs/check_001          # read-only
    python real/check_setup.py --robot my_sdk_adapter:MyRobot --out runs/check_002 --move   # + small test motions
    python real/check_setup.py --mock --out runs/check_mock

What it writes into --out:
- overlay_<camera>.png: every camera image with the twin's points of both arms drawn on it (TCP = red, finger pads =
  yellow, link origins = cyan). If the joint convention, the arm base poses and the camera extrinsics are right, the
  dots sit on the real grippers and links. A consistent offset means a calibration error; dots on the wrong part of
  the arm usually mean a joint zero or sign differs from the twin's URDF.
- check.json: joint readings and limits, camera intrinsics, valid-depth fraction, the --move tracking errors.
"""
import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'third_party/xpolicylab_shared')]
os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from real.backend import RealBackend  # noqa: E402
from real.run_real_episode import load_class  # noqa: E402
from q0_harness.controller import robot_geometry as rg  # noqa: E402


def project(K, T_cam_world_gl, pts):
    """World points -> pixel (u, v) with OpenCV K and an OpenGL camera-to-world; None if behind the camera."""
    T = np.linalg.inv(T_cam_world_gl)
    out = []
    for p in pts:
        c = T[:3, :3] @ p + T[:3, 3]
        c = np.array([c[0], -c[1], -c[2]])        # OpenGL -> OpenCV camera axes
        out.append(None if c[2] <= 1e-3 else (K[0, 0]*c[0]/c[2]+K[0, 2], K[1, 1]*c[1]/c[2]+K[1, 2]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--robot'); ap.add_argument('--robot-args', default='{}')
    ap.add_argument('--cameras', default='real.realsense:RealSenseCameras'); ap.add_argument('--camera-args', default='{}')
    ap.add_argument('--mock', action='store_true')
    ap.add_argument('--twin', type=Path, default=ROOT/'twin/twin.xml')
    ap.add_argument('--move', action='store_true', help='small test motions: joint 1 of each arm +-0.05 rad, grippers')
    ap.add_argument('--slowdown', type=float, default=2.)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(a.twin)); data = mujoco.MjData(model)
    if a.mock:
        from real.mock import MockCameras, MockRobot
        robot = MockRobot(a.twin); cameras = MockCameras(robot)
    else:
        robot = load_class(a.robot)(**json.loads(a.robot_args))
        cameras = load_class(a.cameras)(**json.loads(a.camera_args))
    b = RealBackend(model, data, robot, cameras, 7500, slowdown=a.slowdown)
    report = {'joints': {}, 'cameras': {}}
    q = b.joints()
    ok = True
    for arm in ('left', 'right'):
        lo, hi = b.joint_limits[arm].bounds.T
        inside = bool(np.all((q[arm] >= lo-1e-3) & (q[arm] <= hi+1e-3)))
        ok &= inside
        report['joints'][arm] = {'q_rad': q[arm].round(4).tolist(), 'within_twin_limits': inside,
                                 'tcp_world_m': b.pose(arm)[0].round(4).tolist()}
        print(f'{arm:5s} q = {np.round(q[arm], 3)}  within twin limits: {inside}  tcp = {np.round(b.pose(arm)[0], 3)}')
    vision = b.rig.capture(b.data)
    pts = {}
    for arm, v in b.arm.items():
        p, R = b.pose(arm)
        pads = [p + rg.FINGERTIP_REACH_M*R[:, 0] + s*.02*R[:, 1] for s in (-1, 1)]
        links = [b.data.xpos[i] for i in sorted(v['bodies'])]
        pts[arm] = {'tcp': [p], 'pads': pads, 'links': links}
    from PIL import Image, ImageDraw
    for name, cam in vision.items():
        dep = cam['depth']
        report['cameras'][name] = {'shape': list(cam['color'].shape), 'K': cam['intrinsic_matrix'].round(2).tolist(),
                                   'valid_depth_fraction': float(((dep > .02) & (dep < 3)).mean())}
        img = Image.fromarray(cam['color']); dr = ImageDraw.Draw(img)
        for arm in pts:
            for kind, colour, r in (('links', (0, 255, 255), 3), ('pads', (255, 255, 0), 4), ('tcp', (255, 0, 0), 5)):
                for uv in project(cam['intrinsic_matrix'], cam['extrinsic_matrix'], pts[arm][kind]):
                    if uv is not None:
                        dr.ellipse([uv[0]-r, uv[1]-r, uv[0]+r, uv[1]+r], outline=colour, width=2)
        img.save(a.out/f'overlay_{name}.png')
        print(f'{name:16s} {cam["color"].shape}  valid depth {report["cameras"][name]["valid_depth_fraction"]:.2f}  '
              f'-> {a.out/("overlay_" + name + ".png")}')
    if a.move:
        if not a.mock:
            input('About to move joint 1 of each arm by +-0.05 rad and open/close both grippers. Enter to continue ')
        errs = []
        for arm in ('left', 'right'):
            for delta in (.05, -.05, 0.):
                q1 = {k: v.copy() for k, v in q.items()}; q1[arm][0] += delta
                n = 25
                for k in range(n):
                    u = (k+1)/n
                    b.step({k2: (1-u)*b.targets[k2] + u*q1[k2] for k2 in q1}, {'left': 1., 'right': 1.})
                for _ in range(10):
                    b.step(q1, {'left': 1., 'right': 1.})
                errs.append(float(np.max(np.abs(b.joints()[arm]-q1[arm]))))
        for g in (0., 1.):
            for _ in range(25):
                b.step(q, {'left': g, 'right': g})
        report['move'] = {'max_settled_tracking_error_rad': max(errs), 'per_move': errs}
        print('test motions: max settled tracking error %.4f rad' % max(errs))
    report['ok'] = bool(ok)
    (a.out/'check.json').write_text(json.dumps(report, indent=1))
    b.close()
    print('check.json written;', 'joints OK' if ok else 'JOINTS OUTSIDE TWIN LIMITS: check zero/sign convention')


if __name__ == '__main__':
    main()
