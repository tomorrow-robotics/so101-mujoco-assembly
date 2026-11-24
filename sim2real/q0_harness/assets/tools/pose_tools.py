"""Pose estimation from the cameras' colour + depth images and the CAD meshes (read-only station tool).

    import sys; sys.path.insert(0, '/work/tools')
    import pose_tools as pt
    obs = pt.load('/work/observations/obs_00012')                 # colour, depth, K, camera-to-world of every camera
    T, info = pt.estimate_pose(obs, 'upper_print', color=(26, 191, 204))   # part pose in the world, 4x4 (metres)
    # info: {'rms_mm', 'inlier_fraction', 'points_used', 'per_camera_points', ...}
    rel, twist_deg = pt.in_hand(obs, 'right', T)                  # part pose in the gripper frame, twist about the jaw axis

How it works: in every camera, pixels whose colour is close to `color` are back-projected with the depth image into
world points; the CAD mesh (/work/cad/<part>.npz, part body frame) is sampled with normals and fitted to all those
points at once by robust point-to-plane ICP (outlier points beyond a few millimetres are ignored). Without `init` it
tries a set of starting orientations around the observed points and keeps the best fit. Only the visible side of a
part is seen, so give `init` (e.g. the previous estimate, or the CAD pose relative to a measured part) when you have
one: it converges in a few iterations and is then accurate to a fraction of a millimetre on well-seen parts.
Check `info['rms_mm']` (good fits: under ~0.5 mm) and `info['inlier_fraction']`.

Conventions: K is OpenCV (x right, y down, z forward); camera_to_world uses OpenGL camera axes (x right, y up, z back),
so a point (x, y, z) from K and depth is (x, -y, -z) in that camera frame. pose_tools handles this.
"""
import json
import os

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

FLIP = np.diag([1., -1., -1.])
_MESH_CACHE = {}


def load(obs_dir):
    """-> {camera: {'rgb', 'depth', 'K', 'T'}} for one observation folder."""
    meta = json.load(open(os.path.join(obs_dir, 'observation.json')))
    files = (meta.get('depth') or {}).get('files') or {}
    out = {}
    for cam, cal in meta['calibration'].items():
        rgb = np.load(os.path.join(obs_dir, cam + '.rgb.npy'))
        dep = np.load(files[cam]) if cam in files else None
        out[cam] = {'rgb': rgb, 'depth': dep, 'K': np.array(cal['K']), 'T': np.array(cal['camera_to_world'])}
    out['_state'] = meta.get('state', {})
    return out


def color_mask(rgb, color, tol=0.10, min_brightness=12):
    """Pixels whose chromaticity is within `tol` of `color` (robust to shading): normalised RGB distance."""
    x = rgb[..., :3].astype(np.float32)
    s = x.sum(-1, keepdims=True) + 1e-6
    c = np.asarray(color, np.float32); c = c / c.sum()
    return (np.linalg.norm(x / s - c, axis=-1) < tol) & (x.max(-1) > min_brightness)


def world_points(cam, mask, max_depth=3.0, stride=1):
    """Back-project masked pixels of one camera to world points (metres)."""
    d = cam['depth']
    v, u = np.nonzero(mask[::stride, ::stride]); v, u = v*stride, u*stride
    z = d[v, u]; keep = (z > 0.02) & (z < max_depth); v, u, z = v[keep], u[keep], z[keep]
    K = cam['K']
    p = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], 1)
    p = (FLIP @ p.T).T
    return (cam['T'][:3, :3] @ p.T).T + cam['T'][:3, 3]


def cad_samples(part, n=6000, cad_dir='/work/cad'):
    """Area-weighted surface samples and normals of the CAD mesh (part body frame)."""
    key = (part, n)
    if key not in _MESH_CACHE:
        m = np.load(os.path.join(cad_dir, part + '.npz')); V, F = m['vertices'], m['faces']
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        cr = np.cross(b - a, c - a); area = np.linalg.norm(cr, axis=1)
        nrm = cr / (area[:, None] + 1e-12)
        rng = np.random.default_rng(0)
        idx = rng.choice(len(F), n, p=area/area.sum())
        r1, r2 = rng.random(n), rng.random(n); s1 = np.sqrt(r1)
        pts = (1-s1)[:, None]*a[idx] + (s1*(1-r2))[:, None]*b[idx] + (s1*r2)[:, None]*c[idx]
        _MESH_CACHE[key] = (pts, nrm[idx])
    return _MESH_CACHE[key]


def icp(model_pts, model_nrm, scene_pts, T0, iters=60, trim_start_mm=10., trim_end_mm=2., w_point=0.3):
    """Robust ICP of the model (part frame) onto scene points (world). -> T, rms_mm, inlier_frac.
    Each observed point is matched to its nearest model point (so the unseen side of the part does not matter). The
    step minimises point-to-plane distance plus a smaller point-to-point term (edges then fix sliding along flat
    faces); the outlier cut shrinks gradually from trim_start_mm to trim_end_mm over the first two thirds."""
    T = np.asarray(T0, float).copy()
    n_shrink = max(1, int(iters*2/3))
    for it in range(iters):
        P = (T[:3, :3] @ model_pts.T).T + T[:3, 3]
        N = (T[:3, :3] @ model_nrm.T).T
        dist, j = cKDTree(P).query(scene_pts)
        thr = (trim_start_mm + (trim_end_mm - trim_start_mm)*min(1., it/n_shrink))*1e-3
        k = dist < thr
        if k.sum() < 30:
            break
        s, p, n = scene_pts[k], P[j[k]], N[j[k]]
        A1 = np.hstack([np.cross(p, n), n]); b1 = np.einsum('ij,ij->i', s - p, n)
        # point-to-point rows: [ -[p]x  I ] x = s - p
        px = np.zeros((len(p), 3, 3)); px[:, 0, 1], px[:, 0, 2], px[:, 1, 2] = -p[:, 2], p[:, 1], -p[:, 0]
        px[:, 1, 0], px[:, 2, 0], px[:, 2, 1] = p[:, 2], -p[:, 1], p[:, 0]
        A2 = np.concatenate([-px, np.broadcast_to(np.eye(3), (len(p), 3, 3))], axis=2).reshape(-1, 6)
        b2 = (s - p).reshape(-1)
        A = np.vstack([A1, w_point*A2]); bvec = np.concatenate([b1, w_point*b2])
        x, *_ = np.linalg.lstsq(A, bvec, rcond=None)
        D = np.eye(4); D[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix(); D[:3, 3] = x[3:]
        T = D @ T
        if it > n_shrink and np.linalg.norm(x) < 1e-8:
            break
    P = (T[:3, :3] @ model_pts.T).T + T[:3, 3]
    dist, _ = cKDTree(P).query(scene_pts)
    k = dist < trim_end_mm*1e-3
    rms = float(np.sqrt(np.mean(dist[k]**2))*1e3) if k.any() else float('inf')
    return T, rms, float(k.mean())


def largest_cluster(pts, radius=0.006, near=None, near_radius=0.08):
    """Points of one connected 3D cluster (neighbours within `radius`): the one nearest `near` if given, else the
    largest. Removes other objects of a similar colour."""
    if len(pts) == 0:
        return pts
    if near is not None:
        pts = pts[np.linalg.norm(pts - near, axis=1) < near_radius] if len(pts) else pts
        if len(pts) == 0:
            return pts
    tree = cKDTree(pts)
    pairs = tree.query_pairs(radius, output_type='ndarray')
    parent = np.arange(len(pts))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    roots = np.array([find(i) for i in range(len(pts))])
    labels, counts = np.unique(roots, return_counts=True)
    return pts[roots == labels[np.argmax(counts)]]


def estimate_pose(obs, part, color, init=None, cameras=None, tol=0.10, n_model=6000, hypotheses=24):
    """Fit the CAD of `part` to the colour-masked depth points of all (or the given) cameras. -> (T_world_part, info)."""
    cams = [c for c in (cameras or obs) if not c.startswith('_') and obs[c]['depth'] is not None]
    pts, per = [], {}
    for c in cams:
        w = world_points(obs[c], color_mask(obs[c]['rgb'], color, tol))
        per[c] = len(w); pts.append(w)
    scene = np.vstack(pts) if pts else np.zeros((0, 3))
    if len(scene) < 50:
        return None, {'error': 'too few points of that colour', 'per_camera_points': per}
    if len(scene) > 20000:
        scene = scene[np.random.default_rng(0).choice(len(scene), 20000, replace=False)]
    mp, mn = cad_samples(part, n_model)
    near = None if init is None else (np.asarray(init, float)[:3, :3] @ mp.mean(0) + np.asarray(init, float)[:3, 3])
    scene = largest_cluster(scene, near=near)
    if len(scene) < 50:
        return None, {'error': 'no connected cluster of that colour (near init)' if init is not None else 'no connected cluster',
                      'per_camera_points': per}
    starts = []
    if init is not None:
        T0 = np.asarray(init, float)
        starts = [T0]
        # nearby starts: flat parts have false minima a few millimetres from the true pose
        for dx in (-.005, .005):
            for ax in range(3):
                T = T0.copy(); T[ax, 3] += dx; starts.append(T)
        for ang in (-6, 6):
            for ax in range(3):
                rv = np.zeros(3); rv[ax] = np.radians(ang)
                T = T0.copy(); c0 = T0[:3, :3] @ mp.mean(0) + T0[:3, 3]
                Rr = Rotation.from_rotvec(rv).as_matrix()
                T[:3, :3] = Rr @ T0[:3, :3]; T[:3, 3] = c0 - T[:3, :3] @ mp.mean(0); starts.append(T)
    else:
        c = np.median(scene, 0)
        # principal axes of the model and of the observed cluster, matched with every sign/axis permutation
        _, _, vm = np.linalg.svd(mp - mp.mean(0), full_matrices=False)
        _, _, vs = np.linalg.svd(scene - scene.mean(0), full_matrices=False)
        cands = []
        for R in Rotation.create_group('O').as_matrix():
            Rp = vs.T @ R @ vm
            if np.linalg.det(Rp) > 0:
                cands.append(Rp)
        cands += list(Rotation.create_group('O').as_matrix()[:hypotheses])
        for R in cands:
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = c - R @ mp.mean(0); starts.append(T)
    best = None
    for T0 in starts:
        T, rms, frac = icp(mp, mn, scene, T0)
        score = rms + 2.0*(1.0 - frac)
        if best is None or score < best[0]:
            best = (score, T, rms, frac)
    _, T, rms, frac = best
    return T, {'rms_mm': rms, 'inlier_fraction': frac, 'points_used': int(len(scene)), 'per_camera_points_before_clustering': per,
               'starts_tried': len(starts)}


def gripper_frame(obs, arm):
    """Gripper frame in the world from the observation state: origin = grasp point, x = tool axis, y = jaw axis."""
    s = obs['_state'][arm]
    x = np.asarray(s['tool_axis'], float); y = np.asarray(s['jaw_axis'], float)
    y = y - x*(x @ y); y /= np.linalg.norm(y)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, np.cross(x, y); T[:3, 3] = s['grasp_point']
    return T


def in_hand(obs, arm, T_world_part, reference=None):
    """Part pose in the gripper frame; with `reference` (an earlier in-hand pose) also the twist about the jaw axis
    since then, in degrees (the way a long part turns in the fingers). -> (T_gripper_part, twist_deg or None)."""
    G = gripper_frame(obs, arm)
    rel = np.linalg.solve(G, T_world_part)
    twist = None
    if reference is not None:
        d = Rotation.from_matrix(rel[:3, :3] @ np.asarray(reference)[:3, :3].T).as_rotvec()
        twist = float(np.degrees(d[1]))     # component about the gripper's y (jaw) axis
    return rel, twist


def _scene_points(obs, color, cameras=None, tol=0.10, near=None):
    cams = [c for c in (cameras or obs) if not c.startswith('_') and obs[c]['depth'] is not None]
    pts = [world_points(obs[c], color_mask(obs[c]['rgb'], color, tol)) for c in cams]
    scene = np.vstack(pts) if pts else np.zeros((0, 3))
    if len(scene) > 20000:
        scene = scene[np.random.default_rng(0).choice(len(scene), 20000, replace=False)]
    return largest_cluster(scene, near=near) if len(scene) >= 50 else scene


def flip_check(obs, part, color, T, cameras=None, tol=0.10, n_model=12000):
    """Is the fitted pose `T` the right way round? Box-like parts (the servos especially) look almost the same turned
    180 degrees about a body axis: M3 turned about its body X differs from itself by only 0.55 mm on average, so a
    fit can settle on the turned pose with a good rms. This refits from `T` and from `T` turned 180 degrees about each
    body axis (about the part's centre) and scores each fit by how many observed points lie within 0.7 mm of the CAD
    (`tight_fraction`) and their median distance. -> list of dicts, best first: {'turned_about': None|'x'|'y'|'z',
    'T', 'rms_mm', 'tight_fraction', 'median_mm'}; plus a verdict string. If the best two are within ~0.02 in
    tight_fraction, the depth cannot tell: look at a distinguishing feature (for M3: only its drive horn sticks out
    beyond the case, by 3 mm, on body +Z at body X +12.5 mm) from a camera that sees it, or turn the part to show it."""
    T = np.asarray(T, float)
    mp, mn = cad_samples(part, n_model)
    c = mp.mean(0)
    scene = _scene_points(obs, color, cameras, tol, near=T[:3, :3] @ c + T[:3, 3])
    if len(scene) < 50:
        return [], 'too few points of that colour near T'
    rows = []
    for ax in (None, 'x', 'y', 'z'):
        T0 = T.copy()
        if ax is not None:
            R = np.diag([1., 1., 1.]); i = 'xyz'.index(ax); R[[j for j in range(3) if j != i], [j for j in range(3) if j != i]] = -1
            T0[:3, :3] = T[:3, :3] @ R; T0[:3, 3] = T[:3, :3] @ c + T[:3, 3] - T0[:3, :3] @ c
        Tf, rms, frac = icp(mp, mn, scene, T0, iters=40, trim_start_mm=4.)
        d, _ = cKDTree((Tf[:3, :3] @ mp.T).T + Tf[:3, 3]).query(scene)
        rows.append({'turned_about': ax, 'T': Tf, 'rms_mm': rms, 'tight_fraction': float(np.mean(d < 7e-4)),
                     'median_mm': float(np.median(d)*1e3)})
    rows.sort(key=lambda r: (-r['tight_fraction'], r['median_mm']))
    gap = rows[0]['tight_fraction'] - rows[1]['tight_fraction']
    verdict = (f"best: {'as given' if rows[0]['turned_about'] is None else 'turned 180 about body ' + rows[0]['turned_about']}"
               f" (tight fraction {rows[0]['tight_fraction']:.3f} vs {rows[1]['tight_fraction']:.3f}; "
               + ('clear' if gap > 0.02 else 'NOT clear - check a distinguishing feature') + ')')
    return rows, verdict
