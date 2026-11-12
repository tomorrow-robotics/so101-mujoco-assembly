#!/usr/bin/env python
"""Static interference of a candidate scene at its assembly interfaces and in the fully assembled state.

For every interface in the interfaces file the model is put at the interface's ``qpos`` (moving part at its CAD target,
predecessors assembled) and the signed distance between every collision geom of the moving part and every collision
geom of the fixed parts is evaluated (negative = penetration depth).  The same is done for every pair of parts in the
fully assembled state (``qpos`` of ``--assembled-interface``, default ``end_frame_jaw``).

    python benchmark/static_interference.py --scene scene.xml --interfaces interfaces.json --out static_interference.json

Distance methods
----------------
Three numbers are computed for each geom pair and all three are reported:

``mujoco``   ``mujoco.mj_geomDistance`` (MuJoCo's own convex distance).  In MuJoCo 3.9.0 it returns *exactly 0.0* for
             a few percent of nearby mesh pairs that are in fact 0.2-2 mm apart, and is occasionally wrong for
             touching/penetrating pairs, so its raw minimum is not usable on its own (``n_exact_zero`` counts them).
``fcl``      python-fcl convex-convex signed distance on the same hulls.  Agrees with MuJoCo to < 0.01 mm for
             separated pairs; its penetration depths are unreliable for thin pieces.
``resolved`` the headline value.  Where the two agree on a clearly separated pair it is that distance.  Every other
             pair (either method <= ``--agree-above-mm``, MuJoCo's exact zero, or disagreement) is recomputed exactly
             from the Minkowski difference of the two hulls: if the origin is inside, the penetration depth is the
             distance to the nearest face (minimum translation that separates the pieces); otherwise the distance
             is the distance from the origin to the hull.  Pairs too large for that (``--exact-max-points``) keep
             the fcl value and are counted in ``n_unresolved``.

Definitions
-----------
* A part = the named body plus its descendant bodies (e.g. a servo and its output horn).
* Geom pairs whose world AABBs are further apart than ``--distmax-mm`` are skipped; a reported minimum distance equal
  to ``distmax_mm`` therefore means "at least that far" (``capped: true``).
* ``resolved`` covers every geom pair (pure geometry); ``resolved_collidable`` only pairs MuJoCo itself would collide
  (contype/conaffinity compatible, not removed by an ``<exclude>``).
* All lengths are in mm.  Penetration depth is ``-distance`` for pairs with ``distance < 0``; ``--pen-tol-mm`` (default
  1e-6) is the numerical noise floor below which a pair counts as touching, not penetrating.
"""
from __future__ import annotations

import argparse
import itertools
import json
import time
from multiprocessing import Pool

import mujoco
import numpy as np

from _common import body_collision_geoms, default_workers, geom_local_hull, header, load_model, write_json

DEPTH_BINS_MM = (0.01, 0.05, 0.1, 0.5)          # penetrating pairs are also counted at depth > each of these
_STATE: dict = {}


def _world_aabbs(model, data, geoms):
    """World-axis-aligned boxes (lo, hi) of geoms, from the geom-frame ``geom_aabb`` (centre, half-size)."""
    centre, half = model.geom_aabb[geoms, :3], model.geom_aabb[geoms, 3:]
    rot = data.geom_xmat[geoms].reshape(-1, 3, 3)
    c = data.geom_xpos[geoms] + np.einsum("nij,nj->ni", rot, centre)
    h = np.einsum("nij,nj->ni", np.abs(rot), half)
    return c - h, c + h


def _excluded_body_pairs(model):
    """Body pairs listed in ``<contact><exclude>`` (signature = (body1 << 16) + body2)."""
    pairs = set()
    for sig in model.exclude_signature:
        a, b = int(sig) >> 16, int(sig) & 0xFFFF
        pairs.add((min(a, b), max(a, b)))
    return pairs


def _collidable(model, excluded, g1, g2):
    """Would MuJoCo's default (dynamic) collision pipeline consider this geom pair?"""
    b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
    if (min(b1, b2), max(b1, b2)) in excluded:
        return False
    return bool((model.geom_contype[g1] & model.geom_conaffinity[g2]) or
                (model.geom_contype[g2] & model.geom_conaffinity[g1]))


def exact_signed_distance(points_a: np.ndarray, points_b: np.ndarray) -> float:
    """Exact signed distance (same units as the input) between conv(points_a) and conv(points_b).

    The Minkowski difference ``B - A`` is the convex hull of all pairwise vertex differences.  The two hulls overlap
    iff it contains the origin; the penetration depth is then the distance from the origin to its nearest face.
    Otherwise the distance between the hulls equals the distance from the origin to the Minkowski hull.
    """
    import trimesh
    from scipy.spatial import ConvexHull, QhullError

    diff = (points_b[None, :, :] - points_a[:, None, :]).reshape(-1, 3)
    try:
        hull = ConvexHull(diff)
    except QhullError:
        hull = ConvexHull(diff, qhull_options="QJ Pp")
    offsets = hull.equations[:, 3]                  # n.x + offset <= 0 inside; at the origin this is the offset
    if offsets.max() <= 0:
        return float(offsets.max())                 # inside: -(depth to the nearest face)
    tri = diff[hull.simplices]
    closest = trimesh.triangles.closest_point(tri, np.zeros((len(tri), 3)))
    return float(np.linalg.norm(closest, axis=1).min())


class Shapes:
    """Per-worker cache of geom hulls (geom frame, mm) and their fcl convex shapes."""

    def __init__(self, model):
        self.model, self.local, self.convex = model, {}, {}

    def hull(self, geom):
        if geom not in self.local:
            self.local[geom] = geom_local_hull(self.model, geom)
        return self.local[geom]

    def world_points(self, data, geom):
        return self.hull(geom)[0] @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom] * 1000.0

    def fcl_object(self, data, geom):
        import fcl

        if geom not in self.convex:
            verts, faces = self.hull(geom)
            if len(faces) == 0:
                self.convex[geom] = None
            else:
                encoding = np.c_[np.full(len(faces), 3), faces].ravel().astype(np.int32)
                self.convex[geom] = fcl.Convex(verts, len(faces), encoding)
        if self.convex[geom] is None:
            return None
        return fcl.CollisionObject(self.convex[geom], fcl.Transform(data.geom_xmat[geom].reshape(3, 3),
                                                                    data.geom_xpos[geom] * 1000.0))


def _stats(dist_mm, distmax_mm, pen_tol):
    """Summary of an array of signed pair distances (mm)."""
    bins = {str(b): 0 for b in DEPTH_BINS_MM}
    if len(dist_mm) == 0:
        return dict(n_pairs=0, min_distance_mm=distmax_mm, capped=True, n_penetrating=0, max_penetration_mm=0.0,
                    n_penetrating_deeper_than_mm=bins)
    low = float(dist_mm.min())
    depth = -dist_mm[dist_mm < -pen_tol]
    return dict(n_pairs=int(len(dist_mm)), min_distance_mm=min(low, distmax_mm), capped=bool(low >= distmax_mm),
                n_penetrating=int(len(depth)), max_penetration_mm=float(depth.max()) if len(depth) else 0.0,
                n_penetrating_deeper_than_mm={str(b): int((depth > b).sum()) for b in DEPTH_BINS_MM})


def _init(scene):
    model = load_model(scene)
    _STATE.update(model=model, data=mujoco.MjData(model), excluded=_excluded_body_pairs(model), shapes=Shapes(model))


def _check(task):
    """Distance check between part set A and part set B at one qpos.  Returns a JSON-ready dict."""
    import fcl

    name, qpos, parts_a, parts_b, opt = task
    model, data, excluded, shapes = (_STATE[k] for k in ("model", "data", "excluded", "shapes"))
    distmax_mm, agree_above, exact_max, pen_tol = opt
    start = time.perf_counter()
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    ga = np.concatenate([body_collision_geoms(model, p, subtree=True) for p in parts_a])
    gb = np.concatenate([body_collision_geoms(model, p, subtree=True) for p in parts_b])
    lo_a, hi_a = _world_aabbs(model, data, ga)
    lo_b, hi_b = _world_aabbs(model, data, gb)
    request = fcl.DistanceRequest(enable_signed_distance=True)
    objects = {}

    def fcl_object(geom):
        if geom not in objects:
            objects[geom] = shapes.fcl_object(data, geom)
        return objects[geom]

    rows = []            # geomA, geomB, mujoco mm, fcl mm, resolved mm, method
    fromto = np.zeros(6)
    counts = dict(n_agree=0, n_exact=0, n_unresolved=0, n_mujoco_exact_zero=0)
    for i, g1 in enumerate(ga):
        g1 = int(g1)
        gap = np.maximum(lo_b - hi_a[i], lo_a[i] - hi_b).max(axis=1)        # AABB separation lower bound
        for g2 in gb[gap <= distmax_mm / 1000.0]:
            g2 = int(g2)
            d_mj = mujoco.mj_geomDistance(model, data, g1, g2, distmax_mm / 1000.0, fromto) * 1000.0
            o1, o2 = fcl_object(g1), fcl_object(g2)
            d_fcl = fcl.distance(o1, o2, request, fcl.DistanceResult()) if o1 is not None and o2 is not None else None
            if d_fcl is not None and d_mj >= distmax_mm and d_fcl >= distmax_mm:
                continue                                                    # both: further than distmax
            counts["n_mujoco_exact_zero"] += d_mj == 0.0
            if d_fcl is not None and min(d_mj, d_fcl) > agree_above and abs(d_mj - d_fcl) <= 0.005:
                resolved, method = d_mj, "agree"
                counts["n_agree"] += 1
            else:
                pa, pb = shapes.world_points(data, g1), shapes.world_points(data, g2)
                if len(pa) * len(pb) <= exact_max:
                    resolved, method = exact_signed_distance(pa, pb), "exact"
                    counts["n_exact"] += 1
                else:
                    resolved, method = (d_fcl if d_fcl is not None else d_mj), "unresolved"
                    counts["n_unresolved"] += 1
            rows.append((g1, g2, d_mj, d_fcl, resolved, method))

    resolved = np.array([r[4] for r in rows])
    coll = np.array([_collidable(model, excluded, r[0], r[1]) for r in rows], dtype=bool)
    out = dict(name=name, a=list(parts_a), b=list(parts_b), n_geoms_a=int(len(ga)), n_geoms_b=int(len(gb)),
               n_geom_pairs_total=int(len(ga) * len(gb)), **{k: int(v) for k, v in counts.items()},
               resolved=_stats(resolved, distmax_mm, pen_tol),
               resolved_collidable=_stats(resolved[coll] if len(rows) else resolved, distmax_mm, pen_tol),
               mujoco=_stats(np.array([r[2] for r in rows]), distmax_mm, pen_tol),
               fcl=_stats(np.array([r[3] for r in rows if r[3] is not None]), distmax_mm, pen_tol))
    root_name = lambda g: model.body(model.body_rootid[model.geom_bodyid[g]]).name
    per = {}
    for g1, g2, _, _, d, _ in rows:
        e = per.setdefault(root_name(g2), dict(min_distance_mm=distmax_mm, n_penetrating=0))
        e["min_distance_mm"] = min(e["min_distance_mm"], d)
        e["n_penetrating"] += int(d < -pen_tol)
    out["per_b_part"] = per
    out["worst_pairs"] = [dict(geom_a=model.geom(g1).name or g1, body_a=model.body(model.geom_bodyid[g1]).name,
                               geom_b=model.geom(g2).name or g2, body_b=model.body(model.geom_bodyid[g2]).name,
                               distance_mm=d, method=how, mujoco_mm=d_mj, fcl_mm=d_fcl,
                               collidable=_collidable(model, excluded, g1, g2))
                          for g1, g2, d_mj, d_fcl, d, how in sorted(rows, key=lambda r: r[4])[:5]]
    out["wall_s"] = time.perf_counter() - start
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="candidate scene (.xml or .mjb)")
    ap.add_argument("--interfaces", required=True, help="interfaces JSON: {interfaces: {name: {moving, fixed, qpos}}}")
    ap.add_argument("--out", required=True)
    ap.add_argument("--assembled-interface", default="end_frame_jaw",
                    help="interface whose qpos is the fully assembled state (falls back to the last one in the file)")
    ap.add_argument("--distmax-mm", type=float, default=2.0, help="distances beyond this are not resolved")
    ap.add_argument("--agree-above-mm", type=float, default=0.05,
                    help="mujoco and fcl are trusted only when both exceed this and agree; else the exact method")
    ap.add_argument("--exact-max-points", type=int, default=600_000,
                    help="largest Minkowski point set (verts A x verts B) handled by the exact method")
    ap.add_argument("--pen-tol-mm", type=float, default=1e-6, help="noise floor for counting a pair as penetrating")
    ap.add_argument("--workers", type=int, default=default_workers())
    args = ap.parse_args()

    start = time.perf_counter()
    spec = json.loads(open(args.interfaces).read())["interfaces"]
    model = load_model(args.scene)
    for name, item in spec.items():
        if len(item["qpos"]) != model.nq:
            raise SystemExit(f"interface {name}: qpos has {len(item['qpos'])} entries, the scene has nq={model.nq}")

    opt = (args.distmax_mm, args.agree_above_mm, args.exact_max_points, args.pen_tol_mm)
    tasks = [(name, item["qpos"], [item["moving"]], list(item["fixed"]), opt) for name, item in spec.items()]
    final = args.assembled_interface if args.assembled_interface in spec else list(spec)[-1]
    parts = list(dict.fromkeys(p for item in spec.values() for p in [*item["fixed"], item["moving"]]))
    pair_tasks = [(f"{a}|{b}", spec[final]["qpos"], [a], [b], opt) for a, b in itertools.combinations(parts, 2)]
    every = tasks + pair_tasks
    order = sorted(range(len(every)), key=lambda i: -(len(every[i][2]) + len(every[i][3])))   # big jobs first
    with Pool(args.workers, initializer=_init, initargs=(args.scene,)) as pool:
        done = dict(zip(order, pool.map(_check, [every[i] for i in order], chunksize=1)))
    results = [done[i] for i in range(len(every))]
    interfaces = {r["name"]: {k: v for k, v in r.items() if k != "name"} for r in results[:len(tasks)]}
    pairs = results[len(tasks):]

    def merge(key):
        """Aggregate one statistics block over the part pairs of the assembled state."""
        blocks = [p[key] for p in pairs]
        return dict(n_pairs=sum(b["n_pairs"] for b in blocks),
                    min_distance_mm=min(b["min_distance_mm"] for b in blocks),
                    n_penetrating=sum(b["n_penetrating"] for b in blocks),
                    max_penetration_mm=max(b["max_penetration_mm"] for b in blocks),
                    n_penetrating_deeper_than_mm={str(b): sum(x["n_penetrating_deeper_than_mm"][str(b)] for x in blocks)
                                                  for b in DEPTH_BINS_MM},
                    n_part_pairs_penetrating=sum(b["n_penetrating"] > 0 for b in blocks))

    assembled = dict(qpos_from_interface=final, parts=parts, n_part_pairs=len(pairs),
                     **{k: merge(k) for k in ("resolved", "resolved_collidable", "mujoco", "fcl")},
                     **{k: sum(p[k] for p in pairs) for k in ("n_agree", "n_exact", "n_unresolved",
                                                              "n_mujoco_exact_zero")},
                     part_pairs={p["name"]: dict(min_distance_mm=p["resolved"]["min_distance_mm"],
                                                 n_penetrating=p["resolved"]["n_penetrating"],
                                                 max_penetration_mm=p["resolved"]["max_penetration_mm"],
                                                 mujoco_min_distance_mm=p["mujoco"]["min_distance_mm"],
                                                 fcl_min_distance_mm=p["fcl"]["min_distance_mm"],
                                                 worst_pairs=p["worst_pairs"][:2])
                                 for p in pairs if not p["resolved"]["capped"]})
    block = lambda key, field: {n: r[key][field] for n, r in interfaces.items()}
    summary = dict(
        interfaces_min_distance_mm=block("resolved", "min_distance_mm"),
        interfaces_min_distance_mujoco_mm=block("mujoco", "min_distance_mm"),
        interfaces_min_distance_fcl_mm=block("fcl", "min_distance_mm"),
        interfaces_n_penetrating=block("resolved", "n_penetrating"),
        interfaces_max_penetration_mm=block("resolved", "max_penetration_mm"),
        worst_interface_max_penetration_mm=max(block("resolved", "max_penetration_mm").values()),
        assembled_min_distance_mm=assembled["resolved"]["min_distance_mm"],
        assembled_n_penetrating=assembled["resolved"]["n_penetrating"],
        assembled_max_penetration_mm=assembled["resolved"]["max_penetration_mm"],
        assembled_n_part_pairs_penetrating=assembled["resolved"]["n_part_pairs_penetrating"])
    write_json(args.out, dict(header("static_interference", args.scene, interfaces_file=args.interfaces,
                                     distmax_mm=args.distmax_mm, agree_above_mm=args.agree_above_mm,
                                     pen_tol_mm=args.pen_tol_mm, exact_max_points=args.exact_max_points,
                                     depth_bins_mm=list(DEPTH_BINS_MM), workers=args.workers,
                                     n_exclude=int(model.nexclude)),
                              summary=summary, interfaces=interfaces, assembled=assembled,
                              wall_s=time.perf_counter() - start))
    print(json.dumps(summary, indent=1))
    print(f"static_interference: {time.perf_counter() - start:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()
