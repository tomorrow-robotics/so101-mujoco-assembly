#!/usr/bin/env python3
"""Continuous (swept) clearance of the SO-101 assembly interfaces for any candidate MuJoCo scene.

Standalone: needs only a scene (``.xml`` or ``.mjb``) and an interface data file. Dependencies: mujoco, numpy,
scipy, python-fcl.

    python benchmark/interface_sweep.py --scene scene.xml --interfaces interfaces.json --out out.json [--workers N]

Interface file
--------------
``{"interfaces": {name: {"moving": body, "fixed": [body, ...], "qpos": [...], "withdrawal_world": [x, y, z],
"travel_m": float, "required": bool}}}``

``qpos`` is the full model qpos at the check: after ``mj_kinematics`` the moving part sits at its CAD target and all
its predecessors (``fixed``) are assembled. The candidate scene must therefore keep the reference body/joint layout
(same ``nq``, same body names); only its collision geoms may differ.

Method
------
Every active collision geom (``contype | conaffinity != 0``) whose root body is the moving body or one of the fixed
bodies is a convex piece (mesh geoms are used as the convex hull of their vertices, exactly as MuJoCo collides them;
boxes by their 8 corners; other geom types are rejected). For a convex piece K translated along a straight line, the
swept solid is exactly ``conv(K ∪ (K + span))`` with ``span = travel_m * withdrawal_world``. The interface clearance
is the minimum FCL convex-convex distance between any swept moving piece and any fixed piece, capped at 2 mm.

An interface passes iff ``clearance > 0.005 mm`` and no piece near the sweep has a degenerate (qhull-invalid) hull.

All lengths inside this module are millimetres.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing
import os
import time
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path

import mujoco
import numpy as np

CAP_MM = 2.0             # reported clearance is capped here; also the margin that defines "near" fixed pieces
PASS_MM = 0.005          # an interface passes iff its clearance is strictly larger than this
MAX_LISTED_PAIRS = 50    # colliding pairs listed by name in the output (the count is always exact)
_BOX_CORNERS = np.array(list(product((-1.0, 1.0), repeat=3)))


def load_model(scene: str | os.PathLike) -> mujoco.MjModel:
    """Load a compiled (.mjb) or XML scene."""
    scene = str(scene)
    return mujoco.MjModel.from_binary_path(scene) if scene.endswith(".mjb") else mujoco.MjModel.from_xml_path(scene)


def load_interfaces(path: str | os.PathLike) -> dict:
    """Return ``{name: spec}`` from an interface data file."""
    return json.loads(Path(path).read_text())["interfaces"]


def _convex(points: np.ndarray):
    """FCL convex collision object of the hull of ``points`` (raises scipy QhullError if degenerate)."""
    import fcl
    from scipy.spatial import ConvexHull

    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    triangles = points[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    flip = np.einsum("ij,ij->i", normals, hull.equations[:, :3]) < 0     # make every face wind outwards
    faces[flip] = faces[flip, ::-1]
    encoding = np.c_[np.full(len(faces), 3), faces].ravel().astype(np.int32)
    return fcl.CollisionObject(fcl.Convex(points, len(faces), encoding))


def rigid_transform(rotation: np.ndarray | None = None, translation_mm=(0.0, 0.0, 0.0), pivot_mm=(0.0, 0.0, 0.0)):
    """4x4 world transform (mm): rotate about ``pivot_mm``, then translate by ``translation_mm``."""
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, float)
    pivot = np.asarray(pivot_mm, float)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = pivot - rotation @ pivot + np.asarray(translation_mm, float)
    return matrix


class InterfaceSweep:
    """Geometry of one interface: world-frame convex pieces of the moving part and of its fixed predecessors.

    ``sweep()`` evaluates the straight-line withdrawal, optionally after rigidly displacing the moving part
    (used by the tolerance envelope). Fixed-piece hulls are built lazily and cached across calls.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, spec: dict):
        qpos = np.asarray(spec["qpos"], float)
        if qpos.shape != (model.nq,):
            raise ValueError(f"interface qpos has {qpos.size} entries but the scene has nq={model.nq}: "
                             "the candidate scene must keep the reference body/joint layout")
        data.qpos[:] = qpos
        mujoco.mj_kinematics(model, data)

        def body(name):
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if index < 0:
                raise ValueError(f"scene has no body named {name!r}")
            return index

        roots = model.body_rootid[model.geom_bodyid]
        active = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
        moving_ids = np.flatnonzero((roots == body(spec["moving"])) & active)
        fixed_ids = np.flatnonzero(np.isin(roots, [body(name) for name in spec["fixed"]]) & active)
        if not len(moving_ids) or not len(fixed_ids):
            raise ValueError(f"no active collision geoms for moving={spec['moving']!r} or fixed={spec['fixed']!r}")

        def vertices(geom):
            kind = model.geom_type[geom]
            if kind == mujoco.mjtGeom.mjGEOM_BOX:
                points = _BOX_CORNERS * model.geom_size[geom]
            elif kind == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[geom]
                start = model.mesh_vertadr[mesh]
                points = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
            else:
                raise ValueError(f"geom {model.geom(geom).name!r}: only mesh and box collision geoms are supported")
            return (points @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]) * 1000

        def label(geom):
            return model.geom(geom).name or f"geom#{geom}"

        self.spec = spec
        self.span = np.asarray(spec["withdrawal_world"], float) * float(spec["travel_m"]) * 1000
        self.moving_names = [label(g) for g in moving_ids]
        self.moving_points = [vertices(g) for g in moving_ids]
        self.fixed_names = [label(g) for g in fixed_ids]
        self.fixed_points = [vertices(g) for g in fixed_ids]
        self.fixed_lo = np.array([p.min(axis=0) for p in self.fixed_points])
        self.fixed_hi = np.array([p.max(axis=0) for p in self.fixed_points])
        self._fixed_shapes: dict[int, object] = {}       # index -> fcl object, or None if qhull-invalid

    def pivot_mm(self) -> np.ndarray:
        """AABB centre of the moving part's collision pieces at the target pose."""
        points = np.concatenate(self.moving_points)
        return (points.min(axis=0) + points.max(axis=0)) / 2

    def _fixed_shape(self, index: int):
        if index not in self._fixed_shapes:
            from scipy.spatial import QhullError
            try:
                self._fixed_shapes[index] = _convex(self.fixed_points[index])
            except QhullError:
                self._fixed_shapes[index] = None
        return self._fixed_shapes[index]

    def sweep(self, transform: np.ndarray | None = None, cap_mm: float = CAP_MM) -> dict:
        """Minimum distance between the swept moving pieces and the fixed pieces.

        transform: optional 4x4 world transform (mm) applied to the moving part before the sweep.
        cap_mm:    distances above this are not resolved (<= CAP_MM); the pass/fail decision is unaffected as long
                   as ``cap_mm > PASS_MM``.
        """
        import fcl
        from scipy.spatial import QhullError

        assert PASS_MM < cap_mm <= CAP_MM
        swept = []
        for points in self.moving_points:
            if transform is not None:
                points = points @ transform[:3, :3].T + transform[:3, 3]
            swept.append(np.vstack((points, points + self.span)))
        everything = np.concatenate(swept)
        lower, upper = everything.min(axis=0) - CAP_MM, everything.max(axis=0) + CAP_MM
        near = np.flatnonzero(~(np.any(self.fixed_hi < lower, axis=1) | np.any(self.fixed_lo > upper, axis=1)))
        invalid = set()
        shapes = {}
        for index in near:
            shape = self._fixed_shape(int(index))
            if shape is None:
                invalid.add(self.fixed_names[index])
            else:
                shapes[int(index)] = shape
        usable = np.array(sorted(shapes), dtype=int)
        fixed_lo, fixed_hi = self.fixed_lo[usable], self.fixed_hi[usable]

        request = fcl.DistanceRequest()
        minimum, closest, colliding, calls = cap_mm, None, [], 0
        for name, points in zip(self.moving_names, swept):
            if not len(usable):
                break
            lo, hi = points.min(axis=0), points.max(axis=0)
            gap = np.maximum(np.maximum(lo - fixed_hi, fixed_lo - hi), 0.0)
            aabb_sq = np.einsum("ij,ij->i", gap, gap)                 # lower bound of the true squared distance
            candidates = np.flatnonzero(aabb_sq < max(minimum, 0.001) ** 2)
            if not len(candidates):
                continue
            try:
                moving_shape = _convex(points)
            except QhullError:
                invalid.add(name)
                continue
            for candidate in candidates:
                if aabb_sq[candidate] >= max(minimum, 0.001) ** 2:
                    continue
                distance = fcl.distance(moving_shape, shapes[int(usable[candidate])], request, fcl.DistanceResult())
                calls += 1
                pair = [name, self.fixed_names[usable[candidate]]]
                if distance < minimum:
                    minimum, closest = distance, pair
                if distance <= 0:
                    colliding.append(pair)
        return dict(clearance_mm=float(minimum) if minimum >= 0 else None,
                    passed=bool(minimum > PASS_MM and not invalid),
                    colliding_pair_count=len(colliding),
                    closest_pair=closest,
                    colliding_pairs=colliding[:MAX_LISTED_PAIRS],
                    invalid_near_hulls=sorted(invalid),
                    convex_pairs_checked=calls,
                    moving_pieces=len(self.moving_names),
                    fixed_pieces=len(self.fixed_names),
                    near_fixed_pieces=int(len(near)))


# ---------------------------------------------------------------------------------------------------------------------
# Process-pool plumbing: the model is loaded once in the parent and inherited by forked workers.
_MODEL: mujoco.MjModel | None = None
_DATA: mujoco.MjData | None = None
_INTERFACES: dict = {}
_CONTEXTS: dict[str, InterfaceSweep] = {}


def init_globals(scene, interfaces) -> None:
    global _MODEL, _INTERFACES
    _MODEL, _INTERFACES = load_model(scene), load_interfaces(interfaces)


def context(name: str) -> InterfaceSweep:
    """Per-process cached InterfaceSweep of one interface."""
    global _DATA
    if name not in _CONTEXTS:
        if _DATA is None:
            _DATA = mujoco.MjData(_MODEL)
        _CONTEXTS[name] = InterfaceSweep(_MODEL, _DATA, _INTERFACES[name])
    return _CONTEXTS[name]


def interface_names() -> list[str]:
    return list(_INTERFACES)


def make_pool(workers: int | None, tasks: int) -> ProcessPoolExecutor:
    workers = workers or len(os.sched_getaffinity(0))
    return ProcessPoolExecutor(max(1, min(workers, tasks)), mp_context=multiprocessing.get_context("fork"))


def sha256(path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 24), b""):
            digest.update(block)
    return digest.hexdigest()


def _run_interface(name: str) -> tuple[str, dict]:
    start = time.time()
    spec = _INTERFACES[name]
    result = context(name).sweep()
    return name, dict(moving=spec["moving"], fixed=list(spec["fixed"]), required=bool(spec.get("required", False)),
                      withdrawal_world=list(spec["withdrawal_world"]), travel_mm=float(spec["travel_m"]) * 1000,
                      **result, seconds=round(time.time() - start, 2))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scene", required=True, help="candidate scene (.xml or .mjb)")
    parser.add_argument("--interfaces", required=True, help="interface data file (json)")
    parser.add_argument("--out", required=True, help="output json")
    parser.add_argument("--workers", type=int, default=None, help="processes (default: available CPUs)")
    args = parser.parse_args()

    start = time.time()
    init_globals(args.scene, args.interfaces)
    names = interface_names()
    with make_pool(args.workers, len(names)) as pool:
        results = dict(pool.map(_run_interface, names))
    for name in names:
        row = results[name]
        clearance = "collision" if row["clearance_mm"] is None else f"{row['clearance_mm']:.6f} mm"
        print(f"{name:18s} {'PASS' if row['passed'] else 'FAIL'}  clearance {clearance:>12s}  "
              f"colliding {row['colliding_pair_count']:4d}  closest {row['closest_pair']}", flush=True)
    required = [name for name in names if results[name]["required"]]
    output = dict(tool="interface_sweep", scene=str(args.scene), scene_sha256=sha256(args.scene),
                  interfaces_file=str(args.interfaces), interfaces_sha256=sha256(args.interfaces),
                  mujoco=mujoco.__version__, distance_cap_mm=CAP_MM, pass_threshold_mm=PASS_MM,
                  passed_required=all(results[name]["passed"] for name in required),
                  passed=all(row["passed"] for row in results.values()),
                  wall_seconds=round(time.time() - start, 1), interfaces=results)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(output, indent=1) + "\n")
    print(f"passed_required={output['passed_required']} passed={output['passed']} "
          f"wall={output['wall_seconds']} s -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
