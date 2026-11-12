"""Task-independent exposed-surface gate for a convex-piece collider against a watertight CAD reference.

Standard (user decision 2026-11-11; replaces "<= 0.2 mm vs v1" and task-chosen "contact faces"):
  allowed collider C satisfies   CAD (-) tol  <=  C  <=  closing_r(CAD) (+) tol
  * closing_r(CAD) = CAD plus every place a ball of radius r (the probe) cannot reach from outside. A probe that is
    rounder than r cannot tell C from CAD; a sharp 90 deg edge sees at most ~0.41 r in a filled concave corner.
  * exposed CAD surface (touchable by the probe from outside): |collider offset along the normal| <= tol
  * nowhere: CAD material uncovered by more than tol (missing), extra material in probe-reachable space (protrusion)
No region list, no stage/trajectory knowledge: the probe decides what is exposed, so flips/regrasps/RL are covered.

Numerics (all in mm, reference frame of the CAD):
  * exact signed distance to the tessellated reference (open3d, BRepMesh deflection << tol)
  * reachable probe centers R' = {grid g: sdf(g) >= r - h*sqrt(3)/2}, flood-filled from the grid boundary; R' is a
    superset of the discretised true center set, so grid distances to R' are conservative (flag more, never less)
  * protrusion candidates are confirmed by an exact center search; unconfirmed ones are reported, not hidden
  * collider occupancy uses exact half-space tests of every convex piece
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import ConvexHull, cKDTree
import trimesh

SQ3 = np.sqrt(3.0)
CACHE_DIR = __import__("os").environ.get("SO101_BENCH_CACHE", __import__("os").path.expanduser("~/.cache/so101_bench/envelope"))


# ----------------------------------------------------------------------------------------------- reference
def tessellate_step(step_path: Path, deflection: float = 0.004, angular: float = 0.04) -> trimesh.Trimesh:
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from audit_joint1_candidate import read_reference

    def all_faces_meshed(sh):
        e = TopExp_Explorer(sh, TopAbs_FACE)
        while e.More():
            if BRep_Tool.Triangulation_s(TopoDS.Face_s(e.Current()), TopLoc_Location()) is None:
                return False
            e.Next()
        return True

    for lin, ang in ((deflection, angular), (0.006, 0.06), (0.01, 0.1), (0.005, 0.05), (0.008, 0.08)):
        solid = read_reference(step_path)        # fresh solid: no stale triangulation from a failed pass
        flags = BRepMesh_IncrementalMesh(solid, lin, False, ang, False).GetStatusFlags()
        if flags == 0 or all_faces_meshed(solid):  # a warning flag is fine when every face got a triangulation
            break
    else:
        raise ValueError("BRepMesh failed at every deflection")
    tessellate_step.used = (lin, ang)
    verts, faces, off = [], [], 0
    ex = TopExp_Explorer(solid, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        ex.Next()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            raise ValueError("face without triangulation")
        trsf = loc.Transformation()
        v = np.array([tri.Node(i).Transformed(trsf).Coord() for i in range(1, tri.NbNodes() + 1)])
        f = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
        if face.Orientation() == TopAbs_REVERSED:
            f = f[:, ::-1]
        verts.append(v)
        faces.append(f + off)
        off += len(v)
    mesh = trimesh.Trimesh(np.concatenate(verts), np.concatenate(faces), process=False)
    mesh.merge_vertices(digits_vertex=7)
    return close_cracks(mesh)


def close_cracks(mesh: trimesh.Trimesh, max_span=0.5) -> trimesh.Trimesh:
    """Drop degenerate triangles and cap tiny boundary loops left by BRepMesh (T-junction slivers, < max_span mm)."""
    f = mesh.faces
    good = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2]) & (mesh.area_faces > 1e-14)
    f = f[good]
    directed = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    key = {tuple(e) for e in directed.tolist()}
    nxt = {b: a for a, b in directed.tolist() if (b, a) not in key}     # reversed boundary edges -> cap winding
    new, seen = [], set()
    for start in list(nxt):
        if start in seen:
            continue
        loop, v = [], start
        while v not in seen:
            seen.add(v)
            loop.append(v)
            v = nxt[v]
        if v != start or len(loop) < 3:
            raise ValueError("boundary is not a set of simple loops")
        if np.ptp(mesh.vertices[loop], axis=0).max() > max_span:
            raise ValueError(f"open boundary of {np.ptp(mesh.vertices[loop], axis=0).max():.3f} mm is not a crack")
        new += [[loop[0], loop[i], loop[i + 1]] for i in range(1, len(loop) - 1)]
    out = trimesh.Trimesh(mesh.vertices, np.concatenate([f, np.asarray(new, dtype=f.dtype).reshape(-1, 3)]),
                          process=False)
    out.remove_unreferenced_vertices()
    close_cracks.capped = len(new)
    return out


class Reference:
    def __init__(self, mesh: trimesh.Trimesh):
        if not mesh.is_watertight:
            # closed but non-manifold (an edge shared by 4 faces where two lumps touch along a line) is still a
            # valid closed surface for ray-parity signs; anything with odd edge use is not
            _, cnt = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
            if np.any(cnt % 2) or not mesh.is_winding_consistent:
                raise ValueError("reference mesh is not closed; signed distance would be meaningless")
        self.mesh = mesh
        t = o3d.t.geometry.TriangleMesh()
        t.vertex.positions = o3d.core.Tensor(mesh.vertices.astype(np.float32))
        t.triangle.indices = o3d.core.Tensor(mesh.faces.astype(np.int32))
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(t)
        self.nthreads = 0          # 0 = all cores; pool workers set 1

    def sdf(self, p, chunk=20_000_000, nsamples=1):
        """Signed distance, positive outside. nsamples>1: majority vote of random ray directions for the sign
        (needed on grids aligned with CAD planes, where a single axis ray grazes edges)."""
        p = np.asarray(p, dtype=np.float32).reshape(-1, 3)
        out = np.empty(len(p), dtype=np.float32)
        for s in range(0, len(p), chunk):
            out[s:s + chunk] = self.scene.compute_signed_distance(o3d.core.Tensor(p[s:s + chunk]),
                                                                  nthreads=self.nthreads, nsamples=nsamples).numpy()
        return out

    def closest(self, p):
        r = self.scene.compute_closest_points(o3d.core.Tensor(np.asarray(p, dtype=np.float32)), nthreads=self.nthreads)
        return r["points"].numpy().astype(np.float64), r["primitive_ids"].numpy()


# ----------------------------------------------------------------------------------------------- collider
class Hulls:
    def __init__(self, vertex_sets, names=None):
        self.A, self.b, self.lo, self.hi, self.tris, self.names = [], [], [], [], [], []
        for k, v in enumerate(vertex_sets):
            h = ConvexHull(np.asarray(v, dtype=np.float64))
            e = h.equations
            self.A.append(e[:, :3])
            self.b.append(e[:, 3])
            pts = h.points[h.vertices]
            self.lo.append(pts.min(0))
            self.hi.append(pts.max(0))
            self.tris.append(h.points[h.simplices])
            self.names.append(names[k] if names else str(k))
        self.lo, self.hi = np.array(self.lo), np.array(self.hi)
        self.n = len(self.A)

    @classmethod
    def from_obj_dir(cls, folder: Path, center_mm):
        paths = sorted(Path(folder).glob("*.obj"))
        return cls([trimesh.load(p, process=False).vertices + center_mm for p in paths], [p.name for p in paths])

    @classmethod
    def from_mjb(cls, mjb: Path, body: str, center_mm):
        import mujoco
        from scipy.spatial.transform import Rotation
        m = mujoco.MjModel.from_binary_path(str(mjb))
        bid = m.body(body).id
        sets, names = [], []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                raise ValueError(f"non-mesh collider {m.geom(g).name}")
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            rot = Rotation.from_quat(np.roll(m.geom_quat[g], -1)).as_matrix()
            sets.append((v @ rot.T + m.geom_pos[g]) * 1000.0 + center_mm)
            names.append(m.geom(g).name)
        return cls(sets, names)

    def occupancy(self, origin, h, shape):
        occ = np.zeros(shape, dtype=bool)
        for A, b, lo, hi in zip(self.A, self.b, self.lo, self.hi):
            i0 = np.maximum(np.floor((lo - origin) / h).astype(int), 0)
            i1 = np.minimum(np.ceil((hi - origin) / h).astype(int) + 1, shape)
            if np.any(i1 <= i0):
                continue
            ax = [origin[d] + h * np.arange(i0[d], i1[d]) for d in range(3)]
            Y, Z = np.meshgrid(ax[1], ax[2], indexing="ij")
            YZ = np.stack([Y.ravel(), Z.ravel()], 1)
            for xi, x in enumerate(ax[0]):         # one x-slab at a time: (voxels x faces) stays small for big pieces
                val = YZ @ A[:, 1:].T + (x * A[:, 0] + b)
                ins = np.all(val <= 1e-9, axis=1).reshape(Y.shape)
                occ[i0[0] + xi, i0[1]:i1[1], i0[2]:i1[2]] |= ins
        return occ

    def inside(self, p, eps=1e-9):
        p = np.asarray(p, dtype=np.float64)
        res = np.zeros(len(p), dtype=bool)
        tree = cKDTree(p)
        c = 0.5 * (self.lo + self.hi)
        rad = np.linalg.norm(0.5 * (self.hi - self.lo), axis=1) + 1e-6
        for A, b, cc, rr in zip(self.A, self.b, c, rad):
            ids = np.asarray(tree.query_ball_point(cc, rr), dtype=int)
            ids = ids[~res[ids]]
            if ids.size:
                res[ids] = np.all(p[ids] @ A.T + b <= eps, axis=1)
        return res

    def surface_samples(self, n, rng):
        tris = np.concatenate(self.tris)
        area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
        ids = rng.choice(len(tris), size=n, p=area / area.sum())
        w = rng.dirichlet(np.ones(3), size=n)
        p = np.einsum("nk,nkd->nd", w, tris[ids])
        # union boundary only: drop points strictly inside some piece
        strictly = self.inside(p, eps=-1e-4)
        return p[~strictly], float(area.sum())

    def normal_offsets(self, p, nrm, limit=3.0):
        """Exact signed collider offset along the CAD normal at each point (ray vs convex pieces, merged).
        +t: p is covered, collider boundary t mm outward; -t: p uncovered, nearest covered point t mm inward;
        -limit: nothing within the limit inward."""
        tree = cKDTree(p)
        c = 0.5 * (self.lo + self.hi)
        rad = np.linalg.norm(0.5 * (self.hi - self.lo), axis=1) + limit
        ivs = [[] for _ in range(len(p))]
        for A, b, cc, rr in zip(self.A, self.b, c, rad):
            all_ids = np.asarray(tree.query_ball_point(cc, rr), dtype=int)
            for s0 in range(0, len(all_ids), 8000):       # chunked: (points x faces) temporaries stay small
                self._offsets_chunk(all_ids[s0:s0 + 8000], p, nrm, A, b, limit, ivs)
        return self._merge_offsets(ivs, limit, len(p))

    @staticmethod
    def _offsets_chunk(ids, p, nrm, A, b, limit, ivs):
            if not ids.size:
                return
            vel = nrm[ids] @ A.T                       # d/dt of (A x + b)
            room = -(p[ids] @ A.T + b)                 # need vel * t <= room
            with np.errstate(divide="ignore", invalid="ignore"):
                q = room / vel
            par = np.abs(vel) < 1e-12
            bad = np.any(par & (room < -1e-9), axis=1)
            lo = np.max(np.where(vel < -1e-12, q, -np.inf), axis=1)
            hi = np.min(np.where(vel > 1e-12, q, np.inf), axis=1)
            ok = ~bad & (lo <= hi) & (hi >= -limit) & (lo <= limit)
            for i, l, u in zip(ids[ok], lo[ok], hi[ok]):
                ivs[i].append((l, u))

    @staticmethod
    def _merge_offsets(ivs, limit, n):
        out = np.empty(n)
        for i, iv in enumerate(ivs):
            merged = []
            for l, u in sorted(iv):
                if merged and l <= merged[-1][1] + 1e-9:
                    merged[-1][1] = max(merged[-1][1], u)
                else:
                    merged.append([l, u])
            val = -limit
            for l, u in merged:
                if l <= 1e-9 <= u + 2e-9 or (l <= 0 <= u):
                    val = u
                    break
                if u < 0:
                    val = max(val, u)
            out[i] = min(val, limit)
        return out


# ----------------------------------------------------------------------------------------------- envelope
class Envelope:
    def __init__(self, ref: Reference, r: float, h: float, bbox_lo, bbox_hi):
        self.ref, self.r, self.h = ref, r, h
        pad = r + 3 * h + 1.0
        # irrational sub-voxel shift: grid points never lie on axis-aligned CAD planes
        self.origin = np.floor((np.asarray(bbox_lo) - pad) / h) * h + h * np.array([0.3183, 0.2718, 0.1414])
        top = np.asarray(bbox_hi) + pad
        self.shape = tuple((np.ceil((top - self.origin) / h)).astype(int) + 1)
        t = time.time()
        ax = [self.origin[d] + h * np.arange(self.shape[d], dtype=np.float64) for d in range(3)]
        self.axes = ax
        key = hashlib.sha256(np.ascontiguousarray(ref.mesh.vertices).tobytes() + ref.mesh.faces.tobytes()
                             + np.asarray([r, h, *self.origin, *self.shape], float).tobytes()).hexdigest()[:20]
        cache = Path(CACHE_DIR) / f"envelope_{key}.npz"
        if cache.exists():
            z = np.load(cache)
            self.sdf, self.reach, self.D = z["sdf"], z["reach"], z["D"]
            self.enclosed_center_voxels = int(z["enclosed"])
            self.eps_grid = h * SQ3 / 2
            self.t_sdf = self.t_total = time.time() - t
            return
        sdf = np.empty(self.shape, dtype=np.float32)
        YZ = np.stack(np.meshgrid(ax[1], ax[2], indexing="ij"), -1).reshape(-1, 2)
        step = max(1, int(2e7 // len(YZ)))
        for i in range(0, self.shape[0], step):
            xs = ax[0][i:i + step]
            P = np.concatenate([np.column_stack([np.full(len(YZ), x), YZ]) for x in xs])
            sdf[i:i + step] = ref.sdf(P, nsamples=3).reshape(len(xs), self.shape[1], self.shape[2])
        self.sdf = sdf
        self.t_sdf = time.time() - t
        self.eps_grid = h * SQ3 / 2
        cand = sdf >= r - self.eps_grid
        lab, _ = ndimage.label(cand)
        border = np.unique(np.concatenate([lab[[0, -1]].ravel(), lab[:, [0, -1]].ravel(), lab[:, :, [0, -1]].ravel()]))
        border = border[border > 0]
        self.reach = np.isin(lab, border)
        self.enclosed_center_voxels = int((cand & ~self.reach).sum())
        # distance (mm) from every grid point to the nearest reachable center voxel
        self.D = (ndimage.distance_transform_edt(~self.reach) * h).astype(np.float32)
        Path(CACHE_DIR).mkdir(parents=True, exist_ok=True)
        np.savez(cache, sdf=self.sdf, reach=self.reach, D=self.D, enclosed=self.enclosed_center_voxels)
        self.t_total = time.time() - t

    def index(self, p):
        return (np.asarray(p) - self.origin) / self.h

    def interp(self, field, p):
        return ndimage.map_coordinates(field, self.index(p).T, order=1, mode="nearest")

    def reachable_center(self, c):
        """Exact-ish: center c is a valid probe center if exact sdf >= r - 0.01 and a neighbouring voxel is reachable."""
        idx = np.floor(self.index(c)).astype(int)
        ok = np.zeros(len(c), dtype=bool)
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    j = idx + [dx, dy, dz]
                    j = np.clip(j, 0, np.array(self.shape) - 1)
                    ok |= self.reach[j[:, 0], j[:, 1], j[:, 2]]
        return ok & (self.ref.sdf(c) >= self.r - 0.01)


def fibonacci_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = np.pi * (1 + 5 ** 0.5) * i
    return np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])


def confirm_protrusion(env: Envelope, q, tol, ndir=96):
    """For collider points q outside CAD: True if an exact reachable probe ball contains q deeper than tol."""
    dirs = fibonacci_sphere(ndir)
    hit = np.zeros(len(q), dtype=bool)
    rr = env.r - tol
    for s in range(0, len(q), 4000):
        qq = q[s:s + 4000]
        C = (qq[:, None, :] + rr * dirs[None]).reshape(-1, 3)
        ok = env.reachable_center(C).reshape(len(qq), ndir)
        hit[s:s + 4000] = ok.any(1)
    return hit


# ----------------------------------------------------------------------------------------------- gate
def run_gate(ref: Reference, hulls: Hulls, r: float, tol: float, h: float, n_cad=200_000, n_col=400_000, seed=0,
             env: Envelope | None = None):
    rng = np.random.default_rng(seed)
    lo = np.minimum(ref.mesh.bounds[0], hulls.lo.min(0))
    hi = np.maximum(ref.mesh.bounds[1], hulls.hi.max(0))
    if env is None:
        env = Envelope(ref, r, h, lo, hi)
    t0 = time.time()
    occ = hulls.occupancy(env.origin, h, env.shape)
    vox = h ** 3
    sdf = env.sdf
    missing = (sdf < -tol) & ~occ
    outside = occ & (sdf > tol)
    flagged = outside & (env.D < r - tol + env.eps_grid)
    fill = outside & ~flagged
    res = dict(probe_r_mm=r, tol_mm=tol, grid_h_mm=h, pieces=hulls.n,
               cad_volume_mm3=float(ref.mesh.volume), collider_volume_grid_mm3=float(occ.sum() * vox),
               enclosed_probe_center_voxels=env.enclosed_center_voxels,
               volume=dict(missing_mm3=float(missing.sum() * vox),
                           missing_max_depth_mm=float(-sdf[missing].min()) if missing.any() else 0.0,
                           extra_total_mm3=float(outside.sum() * vox),
                           extra_unreachable_fill_mm3=float(fill.sum() * vox),
                           extra_probe_reachable_flagged_mm3=float(flagged.sum() * vox)))
    # connected violation blobs (grid, conservative)
    blobs = []
    lab, nb = ndimage.label(flagged)
    if nb:
        sizes = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, nb + 1))
        order = np.argsort(-sizes)[:15]
        sl = ndimage.find_objects(lab)
        for k in order:
            s = sl[k]
            m = lab[s] == k + 1
            prot = (r + env.eps_grid - env.D[s][m]).max()
            blobs.append(dict(volume_mm3=float(sizes[k] * vox),
                              bbox_lo=[float(env.origin[d] + h * s[d].start) for d in range(3)],
                              bbox_hi=[float(env.origin[d] + h * s[d].stop) for d in range(3)],
                              max_outside_cad_mm=float(sdf[s][m].max()), max_probe_depth_upper_mm=float(prot)))
    res["flagged_blobs_top15"] = blobs
    mb = []
    lab, nb = ndimage.label(missing)
    if nb:
        sizes = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, nb + 1))
        sl = ndimage.find_objects(lab)
        for k in np.argsort(-sizes)[:10]:
            s = sl[k]
            m = lab[s] == k + 1
            mb.append(dict(volume_mm3=float(sizes[k] * vox), max_depth_mm=float(-sdf[s][m].min()),
                           bbox_lo=[float(env.origin[d] + h * s[d].start) for d in range(3)],
                           bbox_hi=[float(env.origin[d] + h * s[d].stop) for d in range(3)]))
    res["missing_blobs_top10"] = mb

    # exact CAD-surface test
    pts, fid = trimesh.sample.sample_surface(ref.mesh, n_cad, seed=seed)
    nrm = ref.mesh.face_normals[fid]
    centers = pts + r * nrm
    exposed = env.reachable_center(centers)
    off = hulls.normal_offsets(pts, nrm, limit=3.0)
    ex_off = off[exposed]
    a = np.abs(ex_off)
    res["cad_surface"] = dict(
        samples=int(len(pts)), exposed_fraction=float(exposed.mean()),
        exposed_abs_offset_mm=dict(median=float(np.median(a)), p95=float(np.quantile(a, .95)),
                                   p99=float(np.quantile(a, .99)), max=float(a.max())),
        exposed_outward_gt_tol=float((ex_off > tol).mean()), exposed_inward_gt_tol=float((ex_off < -tol).mean()),
        unexposed_uncovered_gt_tol=float((off[~exposed] < -tol).mean()) if (~exposed).any() else 0.0,
        exposed_area_fraction_within_tol=float((a <= tol).mean()))
    # exact collider-surface protrusion test
    q, area = hulls.surface_samples(n_col, rng)
    d = ref.sdf(q)
    cand = d > tol
    Dq = env.interp(env.D, q[cand])
    suspect = Dq < r - tol + 2 * env.eps_grid
    qs = q[cand][suspect]
    conf = confirm_protrusion(env, qs, tol) if len(qs) else np.zeros(0, bool)
    dd = d[cand][suspect]
    res["collider_surface"] = dict(
        samples=int(len(q)), area_mm2=area, outside_cad_gt_tol_fraction=float(cand.mean()),
        protrusion_suspects=int(len(qs)), protrusion_confirmed=int(conf.sum()),
        protrusion_confirmed_fraction=float(conf.sum() / len(q)),
        protrusion_confirmed_max_mm=float(dd[conf].max()) if conf.any() else 0.0,
        protrusion_confirmed_p95_mm=float(np.quantile(dd[conf], .95)) if conf.any() else 0.0,
        protrusion_unconfirmed_suspects=int((~conf).sum()))
    res["witnesses"] = dict(
        protrusion=[dict(p=x.tolist(), outside_cad_mm=float(y)) for x, y in
                    sorted(zip(qs[conf], dd[conf]), key=lambda t: -t[1])[:20]],
        exposed_cad_outward=[dict(p=x.tolist(), offset_mm=float(y)) for x, y in
                             sorted(zip(pts[exposed][ex_off > tol], ex_off[ex_off > tol]), key=lambda t: -t[1])[:20]],
        exposed_cad_inward=[dict(p=x.tolist(), offset_mm=float(y)) for x, y in
                            sorted(zip(pts[exposed][ex_off < -tol], ex_off[ex_off < -tol]), key=lambda t: t[1])[:20]])
    cs, cl = res["cad_surface"], res["collider_surface"]
    res["pass"] = dict(
        missing=res["volume"]["missing_mm3"] == 0.0 and cs["unexposed_uncovered_gt_tol"] == 0.0,
        exposed_surface=cs["exposed_abs_offset_mm"]["max"] <= tol,
        no_reachable_protrusion=cl["protrusion_confirmed"] == 0 and res["volume"]["extra_probe_reachable_flagged_mm3"] == 0.0)
    res["pass"]["all"] = all(res["pass"].values())
    res["t_gate_s"] = time.time() - t0
    res["t_envelope_s"] = env.t_total
    return res, env


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reference", type=Path, required=True, help="STEP (tessellated) or watertight mesh, CAD frame mm")
    ap.add_argument("--hulls", type=Path, help="dir of body-frame mm OBJ pieces")
    ap.add_argument("--mjb", type=Path)
    ap.add_argument("--body")
    ap.add_argument("--center-mm", type=float, nargs=3, default=(0, 0, 0), help="body origin in CAD frame, mm")
    ap.add_argument("--r", type=float, nargs="+", default=[0.5])
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--h", type=float, default=0.2)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    cache = a.out / "reference_mesh.ply"
    if a.reference.suffix.lower() in (".step", ".stp"):
        if not cache.exists():
            tessellate_step(a.reference).export(cache)
        mesh = trimesh.load(cache, process=False)
    else:
        mesh = trimesh.load(a.reference, process=False)
    ref = Reference(mesh)
    center = np.asarray(a.center_mm)
    hulls = Hulls.from_obj_dir(a.hulls, center) if a.hulls else Hulls.from_mjb(a.mjb, a.body, center)
    for r in a.r:
        res, _ = run_gate(ref, hulls, r, a.tol, a.h)
        res["reference"] = str(a.reference)
        res["reference_sha256"] = hashlib.sha256(a.reference.read_bytes()).hexdigest()
        res["collider"] = str(a.hulls or f"{a.mjb}:{a.body}")
        (a.out / f"gate_r{r:g}.json").write_text(json.dumps(res, indent=1) + "\n")
        print(json.dumps({k: res[k] for k in ("probe_r_mm", "pieces", "volume", "cad_surface", "collider_surface", "pass",
                                             "t_envelope_s", "t_gate_s")}, indent=1), flush=True)


if __name__ == "__main__":
    main()
