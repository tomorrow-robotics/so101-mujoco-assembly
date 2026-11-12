#!/usr/bin/env python
"""Virtual parallel-jaw width probe: "if a gripper closes on this part here, does it stop at the right width?"

    python benchmark/jaw_probe.py --scene scene.xml --references DIR --out jaw_probe.json

Parts: every free-joint body of the scene that has a CAD reference mesh ``DIR/<body>_reference_mesh.ply`` (body frame,
mm).  ``--reference BODY=PATH`` adds or overrides the reference of one body.  For each part:

1. The three closing axes are the principal axes of the reference vertices (PCA).
2. For each axis, candidate positions lie on a regular grid over the part's projection onto the plane normal to the
   axis.  At each position two square pads (4 mm x 4 mm) close along the axis from both sides.  Each pad is a 5 x 5
   bundle of parallel rays cast from outside the part; a pad stops at the first hit over its bundle (the hit closest
   to the pad) and the jaw width is the distance between the two pads.
3. A position is a valid probe only where a real jaw would get a well-defined grip on the CAD: the centre ray hits
   the CAD (both opposite faces exist) and each pad stops on the surface under its centre, i.e. the pad's stop is
   within 1 mm of the centre ray's first hit.  Holes, edges and lower ground inside the footprint are fine; positions
   where a taller feature at the rim of the footprint (a wall, a rib, a far component) stops the pad are dropped.
   The same rule applies to every part.  The grid pitch is tuned so that 40-80 valid probes (target 60) remain per
   axis; the pitch is never below 1 mm, so an axis along which the part offers few such grips gets fewer probes
   (``axes[].n_probes``).
4. The same probe is run on the collider (every convex-hull triangle of the body's collision geoms).
   ``width_error = collider_width - cad_width`` (mm): positive = the collider is fatter than the CAD there.
   If a collider pad stops more than 10 mm from where the CAD pad stops (or touches nothing), the collider has no
   surface locally: the probe is counted in ``n_no_local_contact`` and excluded from the error statistics.

Reported per part: number of probes, ``n_no_local_contact``, the fraction of scored probes with |error| <= 0.1 mm and
<= 0.5 mm, median/p95/max |error|, signed min/max, the worst 5 probes and every probe.

Because every ray is parallel to the closing axis, ray casting is done exactly in the projected plane (2-D
point-in-triangle), which needs neither embree nor rtree and is deterministic across machines.

Each part also gets a frame sanity check, ``collider_vertex_to_cad``: a random subset of the collider's hull vertices
is classified as on the CAD surface (within 0.05 mm), buried inside the CAD solid (internal cut faces of a convex
decomposition) or outside it.  With the right frame and units nearly all vertices are on or inside the CAD; a low
``frac_on_or_inside`` means the reference and the scene do not share a body frame, or it is the wrong reference.
"""
from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from _common import body_collision_triangles, default_workers, header, load_model, write_json

PAD_MM = 4.0            # pad edge length
BUNDLE = 5              # rays per pad edge
FLAT_MM = 1.0           # a CAD pad must stop within this distance of the first hit of its centre ray
LOCAL_MM = 10.0         # a collider pad stopping further than this from the CAD pad has no local contact
TOLS_MM = (0.1, 0.5)
TARGET, LOW, HIGH = 60, 40, 80
SUFFIX = "_reference_mesh.ply"


class AxisCaster:
    """First hits of rays parallel to +w/-w on a triangle soup given in a (u, v, w) frame.

    Triangles are bucketed on a uniform (u, v) grid so that a query only looks at triangles near the rays.
    """

    CELL_MM = 4.0

    def __init__(self, triangles: np.ndarray):
        self.tri = triangles
        self.lo = triangles[:, :, :2].min(axis=1)
        self.hi = triangles[:, :, :2].max(axis=1)
        self.origin = self.lo.min(axis=0) if len(triangles) else np.zeros(2)
        first = np.floor((self.lo - self.origin) / self.CELL_MM).astype(np.int64)
        last = np.floor((self.hi - self.origin) / self.CELL_MM).astype(np.int64)
        self.shape = last.max(axis=0) + 1 if len(triangles) else np.array([1, 1])
        span = last - first + 1
        count = span[:, 0] * span[:, 1]                             # cells covered by each triangle
        owner = np.repeat(np.arange(len(triangles)), count)
        local = np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)
        cell_u = first[owner, 0] + local // span[owner, 1]
        cell_v = first[owner, 1] + local % span[owner, 1]
        cell = cell_u * self.shape[1] + cell_v
        order = np.argsort(cell, kind="stable")
        self.members = owner[order]
        self.start = np.searchsorted(cell[order], np.arange(self.shape[0] * self.shape[1] + 1))

    def _near(self, pmin: np.ndarray, pmax: np.ndarray) -> np.ndarray:
        """Indices of triangles whose (u, v) bounding box overlaps the box [pmin, pmax]."""
        c0 = np.clip(np.floor((pmin - self.origin) / self.CELL_MM).astype(np.int64), 0, self.shape - 1)
        c1 = np.clip(np.floor((pmax - self.origin) / self.CELL_MM).astype(np.int64), 0, self.shape - 1)
        if np.any(pmax < self.origin) or np.any(pmin > self.origin + self.shape * self.CELL_MM):
            return np.empty(0, dtype=np.int64)
        cells = [u * self.shape[1] + v for u in range(c0[0], c1[0] + 1) for v in range(c0[1], c1[1] + 1)]
        near = np.unique(np.concatenate([self.members[self.start[c]:self.start[c + 1]] for c in cells]))
        return near[(self.lo[near, 0] <= pmax[0]) & (self.hi[near, 0] >= pmin[0]) &
                    (self.lo[near, 1] <= pmax[1]) & (self.hi[near, 1] >= pmin[1])]

    def extremes(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """For rays through ``points`` [n,2] (all inside a small neighbourhood): (highest w hit, lowest w hit) per
        ray, NaN where the ray misses."""
        top = np.full(len(points), np.nan)
        bottom = np.full(len(points), np.nan)
        near = self._near(points.min(axis=0), points.max(axis=0))
        if not len(near):
            return top, bottom
        a, b, c = (self.tri[near, k] for k in range(3))
        e1, e2 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
        det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        ok = np.abs(det) > 1e-12                                   # skip triangles seen edge-on
        a, b, c, e1, e2, det = a[ok], b[ok], c[ok], e1[ok], e2[ok], det[ok]
        rel = points[:, None, :] - a[None, :, :2]                  # [ray, tri, 2]
        s = (rel[..., 0] * e2[:, 1] - rel[..., 1] * e2[:, 0]) / det
        t = (rel[..., 1] * e1[:, 0] - rel[..., 0] * e1[:, 1]) / det
        eps = 1e-9
        hit = (s >= -eps) & (t >= -eps) & (s + t <= 1 + eps)
        w = a[:, 2] + s * (b[:, 2] - a[:, 2]) + t * (c[:, 2] - a[:, 2])
        any_hit = hit.any(axis=1)
        top[any_hit] = np.where(hit, w, -np.inf).max(axis=1)[any_hit]
        bottom[any_hit] = np.where(hit, w, np.inf).min(axis=1)[any_hit]
        return top, bottom


def pad_offsets() -> np.ndarray:
    line = np.linspace(-PAD_MM / 2, PAD_MM / 2, BUNDLE)
    return np.array([[u, v] for u in line for v in line])


def pad_stops(caster: AxisCaster, centre: np.ndarray):
    """Close two pads on ``centre``.  Returns (upper pad stop, lower pad stop, centre ray hit, stop-to-centre gap).

    The upper pad comes down from +w and stops at the highest first hit of its bundle; the lower pad comes up from -w
    and stops at the lowest.  Stops are NaN if no ray of the bundle hits anything.  The gap is the larger of the two
    pads' distances between the pad stop and the first hit of the pad's centre ray (inf if the centre ray misses).
    """
    top, bottom = caster.extremes(centre + pad_offsets())
    if np.isnan(top).all():
        return float("nan"), float("nan"), False, float("inf")
    mid = len(top) // 2                                             # centre ray of the odd-sized bundle
    hit = not np.isnan(top[mid])
    gap = max(np.nanmax(top) - top[mid], bottom[mid] - np.nanmin(bottom)) if hit else float("inf")
    return float(np.nanmax(top)), float(np.nanmin(bottom)), hit, float(gap)


def probe_grid(caster: AxisCaster, lo: np.ndarray, hi: np.ndarray):
    """Valid probe positions over the projection box [lo, hi]: ``(positions [n,2], CAD stops [n,2], pitch)``.

    Valid = the centre ray hits the CAD and each pad stops within FLAT_MM of its centre ray's first hit.  The pitch starts at
    sqrt(box area / TARGET) and is rescaled until LOW <= count <= HIGH (at most 10 rounds, pitch >= 1 mm); the
    result closest to TARGET is returned.  Deterministic: no random sampling.
    """
    def valid(pitch):
        axes = [np.arange(l + ((h - l) % pitch) / 2, h + 1e-9, pitch) for l, h in zip(lo, hi)]   # centred grid
        keep, stops = [], []
        for p in ([u, v] for u in axes[0] for v in axes[1]):
            p = np.array(p)
            top, bottom, hit, gap = pad_stops(caster, p)
            if hit and gap <= FLAT_MM and top - bottom > 1e-6:
                keep.append(p)
                stops.append((top, bottom))
        return np.array(keep).reshape(-1, 2), np.array(stops).reshape(-1, 2)

    pitch = float(np.sqrt(np.prod(hi - lo) / TARGET))
    best = None
    for _ in range(10):
        pts, stops = valid(pitch)
        if best is None or abs(len(pts) - TARGET) < abs(len(best[0]) - TARGET):
            best = (pts, stops, pitch)
        if LOW <= len(pts) <= HIGH or pitch <= 1.0:
            break
        pitch = max(1.0, pitch * float(np.clip(np.sqrt(max(len(pts), 1) / TARGET), 0.4, 2.5)))
    return best


def vertex_to_surface(points: np.ndarray, mesh, samples: int = 200_000) -> np.ndarray:
    """Exact distance (mm) from points to a triangle mesh, without a spatial-index dependency.

    The nearest of ``samples`` surface samples gives an upper bound; the exact value is then the minimum
    point-triangle distance over all triangles whose bounding box lies within that bound of the point.
    """
    import trimesh
    from scipy.spatial import cKDTree

    surface, _ = trimesh.sample.sample_surface(mesh, samples, seed=0)
    bound, _ = cKDTree(surface).query(points)
    tri = np.asarray(mesh.triangles)
    lo, hi = tri.min(axis=1), tri.max(axis=1)
    out = np.empty(len(points))
    for i, (p, r) in enumerate(zip(points, bound * (1 + 1e-9) + 1e-9)):
        near = np.flatnonzero((lo[:, 0] <= p[0] + r) & (hi[:, 0] >= p[0] - r))
        near = near[(lo[near, 1] <= p[1] + r) & (hi[near, 1] >= p[1] - r) &
                    (lo[near, 2] <= p[2] + r) & (hi[near, 2] >= p[2] - r)]
        closest = trimesh.triangles.closest_point(tri[near], np.repeat(p[None], len(near), axis=0))
        out[i] = np.linalg.norm(closest - p, axis=1).min()
    return out


def inside_mesh(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Point-in-solid test for a closed triangle mesh by crossing parity of a +z ray (points get a tiny fixed xy
    offset so rays do not run exactly through mesh vertices or edges)."""
    caster = AxisCaster(triangles)
    inside = np.zeros(len(points), dtype=bool)
    for i, p in enumerate(points + np.array([1.3e-4, 0.7e-4, 0.0])):
        near = caster._near(p[:2], p[:2])
        a, b, c = (triangles[near, k] for k in range(3))
        e1, e2, rel = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2], p[:2] - a[:, :2]
        det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        ok = np.abs(det) > 1e-12
        s = (rel[ok, 0] * e2[ok, 1] - rel[ok, 1] * e2[ok, 0]) / det[ok]
        t = (rel[ok, 1] * e1[ok, 0] - rel[ok, 0] * e1[ok, 1]) / det[ok]
        hit = (s >= 0) & (t >= 0) & (s + t <= 1)
        w = a[ok, 2] + s * (b[ok, 2] - a[ok, 2]) + t * (c[ok, 2] - a[ok, 2])
        inside[i] = (np.count_nonzero(hit & (w > p[2])) % 2) == 1
    return inside


def error_stats(errors) -> dict:
    """Statistics of signed width errors (mm); ``None`` entries are probes without local collider contact."""
    err = np.array([e for e in errors if e is not None], dtype=float)
    out = dict(n_scored=int(len(err)), n_no_local_contact=int(len(errors) - len(err)))
    for tol in TOLS_MM:
        out[f"frac_within_{tol}_mm"] = float((np.abs(err) <= tol).mean()) if len(err) else None
    out.update(median_abs_error_mm=float(np.median(np.abs(err))) if len(err) else None,
               p95_abs_error_mm=float(np.percentile(np.abs(err), 95)) if len(err) else None,
               max_abs_error_mm=float(np.abs(err).max()) if len(err) else None,
               min_signed_error_mm=float(err.min()) if len(err) else None,
               max_signed_error_mm=float(err.max()) if len(err) else None)
    return out


def probe_part(task):
    scene, body, ref_path = task
    import trimesh

    start = time.perf_counter()
    model = load_model(scene)
    ref = trimesh.load(ref_path, process=False)
    col_tri, col_vert = body_collision_triangles(model, body)
    if len(col_tri) == 0:
        return body, dict(skipped="body has no collision geoms")
    ref_vert = np.asarray(ref.vertices, dtype=np.float64)
    ref_tri = ref_vert[np.asarray(ref.faces)]

    rng = np.random.default_rng(0)
    subset = col_vert[rng.choice(len(col_vert), min(2000, len(col_vert)), replace=False)]
    dist = vertex_to_surface(subset, ref)
    far = dist > 0.05
    buried = np.zeros(len(subset), dtype=bool)                 # hull vertices on internal cuts of the decomposition
    buried[far] = inside_mesh(subset[far], ref_tri)
    outside = np.where(buried, 0.0, dist)                      # how far a vertex sticks out of the CAD solid
    frame_check = dict(n=int(len(subset)), frac_on_surface_0p05_mm=float((~far).mean()),
                       frac_inside_cad=float(buried.mean()), frac_on_or_inside=float((~far | buried).mean()),
                       frac_outside_beyond_0p05_mm=float((far & ~buried).mean()),
                       max_outside_mm=float(outside.max()), p95_surface_distance_mm=float(np.percentile(dist, 95)))

    centre = ref_vert.mean(axis=0)
    _, _, axes = np.linalg.svd(ref_vert - centre, full_matrices=False)         # rows: principal axes, major first
    probes, per_axis = [], []
    for k in range(3):
        w, u, v = axes[k], axes[(k + 1) % 3], axes[(k + 2) % 3]
        frame = np.stack([u, v, w])                                            # body -> (u, v, w)
        cad = AxisCaster((ref_tri - centre) @ frame.T)
        col = AxisCaster((col_tri - centre) @ frame.T)
        proj = (ref_vert - centre) @ frame[:2].T
        grid, cad_stops, pitch = probe_grid(cad, proj.min(axis=0), proj.max(axis=0))
        for p, (cad_top, cad_bottom) in zip(grid, cad_stops):
            col_top, col_bottom, _, _ = pad_stops(col, p)
            local = (not np.isnan(col_top) and abs(col_top - cad_top) <= LOCAL_MM
                     and abs(col_bottom - cad_bottom) <= LOCAL_MM)
            position = centre + p[0] * u + p[1] * v
            probes.append(dict(axis=k, position_body_mm=[round(float(x), 4) for x in position],
                               cad_width_mm=float(cad_top - cad_bottom),
                               collider_width_mm=None if np.isnan(col_top) else float(col_top - col_bottom),
                               error_mm=float((col_top - col_bottom) - (cad_top - cad_bottom)) if local else None,
                               pad_offsets_mm=None if np.isnan(col_top) else
                               [float(col_top - cad_top), float(cad_bottom - col_bottom)]))
        per_axis.append(dict(axis=k, direction_body=[float(x) for x in w], grid_pitch_mm=pitch, n_probes=len(grid)))

    result = dict(n_probes=len(probes), **error_stats([p["error_mm"] for p in probes]),
                  worst5=sorted((p for p in probes if p["error_mm"] is not None), key=lambda p: -abs(p["error_mm"]))[:5],
                  no_local_contact_probes=[p for p in probes if p["error_mm"] is None][:5],
                  axes=per_axis, collider_vertex_to_cad=frame_check, n_collider_triangles=int(len(col_tri)),
                  n_reference_triangles=int(len(ref_tri)), reference=str(ref_path),
                  wall_s=time.perf_counter() - start, probes=probes)
    return body, result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="candidate scene (.xml or .mjb)")
    ap.add_argument("--references", required=True, help=f"directory with <body>{SUFFIX} (body frame, mm)")
    ap.add_argument("--reference", action="append", default=[], metavar="BODY=PATH",
                    help="extra or overriding reference mesh for one body (repeatable)")
    ap.add_argument("--parts", nargs="*", help="restrict to these bodies (default: free bodies with a reference)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=default_workers())
    args = ap.parse_args()

    import mujoco

    start = time.perf_counter()
    model = load_model(args.scene)
    free = [model.body(model.jnt_bodyid[j]).name for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    refs = {body: Path(args.references) / (body + SUFFIX) for body in free}
    refs.update((item.split("=", 1)[0], Path(item.split("=", 1)[1])) for item in args.reference)
    names = [n for n in (args.parts or refs) if n in refs and refs[n].exists()]
    missing = sorted(n for n in refs if not refs[n].exists())
    tasks = [(args.scene, n, str(refs[n])) for n in names]
    with Pool(min(args.workers, max(len(tasks), 1))) as pool:
        parts = dict(pool.map(probe_part, tasks, chunksize=1))

    scored = {n: r for n, r in parts.items() if "skipped" not in r}
    overall = dict(n_parts=len(scored), n_probes=sum(r["n_probes"] for r in scored.values()),
                   **error_stats([p["error_mm"] for r in scored.values() for p in r["probes"]]))
    write_json(args.out, dict(header("jaw_probe", args.scene, references={n: str(refs[n]) for n in names},
                                     pad_mm=PAD_MM, rays_per_pad=BUNDLE * BUNDLE, flat_mm=FLAT_MM,
                                     local_contact_mm=LOCAL_MM, bodies_without_reference=missing),
                              overall=overall, parts=parts, wall_s=time.perf_counter() - start))
    keys = ("n_probes", "n_no_local_contact", "frac_within_0.1_mm", "frac_within_0.5_mm", "median_abs_error_mm",
            "p95_abs_error_mm", "max_abs_error_mm", "min_signed_error_mm", "max_signed_error_mm")
    print("part", *keys, "| hull verts on/inside CAD, max outside mm")
    for n, r in scored.items():
        c = r["collider_vertex_to_cad"]
        print(f"{n:16s}", " ".join(f"{r[k]:.3f}" if isinstance(r[k], float) else str(r[k]) for k in keys),
              f"| {c['frac_on_or_inside']:.3f} {c['max_outside_mm']:.3f}")
    print("overall", json.dumps(overall))
    print(f"jaw_probe: {time.perf_counter() - start:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()
