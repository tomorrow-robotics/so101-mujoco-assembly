"""Shared helpers for the SO-101 collision-geometry benchmark.

Everything here depends only on mujoco, numpy, scipy and trimesh, and only reads the candidate scene, so any scene with
the same body names can be scored.

Conventions
-----------
* A geom is a *collision geom* when ``contype | conaffinity != 0``.
* The collision geometry of a body is the set of collision geoms attached to that body.  ``subtree=True`` adds the
  geoms of all descendant bodies (e.g. a servo's output horn), which is what the interface checks use.
* MuJoCo recentres and realigns mesh assets at compile time; ``geom_pos``/``geom_quat`` (relative to the body) absorb
  that transform.  Body-frame vertices are therefore ``R(geom_quat) @ mesh_vert + geom_pos``.
* MuJoCo collides with the *convex hull* of a mesh geom, so hull triangles are what the benchmark measures.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import mujoco
import numpy as np


def load_model(scene: str | os.PathLike) -> mujoco.MjModel:
    """Load a scene from MJCF (``.xml``) or a native binary (``.mjb``)."""
    scene = str(scene)
    if scene.endswith(".mjb"):
        return mujoco.MjModel.from_binary_path(scene)
    return mujoco.MjModel.from_xml_path(scene)


def collision_mask(model: mujoco.MjModel) -> np.ndarray:
    """Boolean mask over geoms: True where the geom takes part in collision."""
    return (model.geom_contype | model.geom_conaffinity) != 0


def subtree_bodies(model: mujoco.MjModel, body: str) -> np.ndarray:
    """Ids of ``body`` and all its descendants."""
    root = model.body(body).id
    keep = [root]
    for b in range(root + 1, model.nbody):          # MuJoCo orders bodies so that parents precede children
        if model.body_parentid[b] in keep:
            keep.append(b)
    return np.asarray(keep)


def body_collision_geoms(model: mujoco.MjModel, body: str, subtree: bool = False) -> np.ndarray:
    """Ids of the collision geoms of ``body`` (optionally including descendant bodies)."""
    bodies = subtree_bodies(model, body) if subtree else np.array([model.body(body).id])
    return np.flatnonzero(np.isin(model.geom_bodyid, bodies) & collision_mask(model))


def _quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.empty(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=np.float64))
    return mat.reshape(3, 3)


def _primitive_vertices(model: mujoco.MjModel, geom: int) -> np.ndarray:
    """Vertices (geom frame, m) whose convex hull approximates a primitive geom."""
    import trimesh

    kind, size = model.geom_type[geom], model.geom_size[geom]
    G = mujoco.mjtGeom
    if kind == G.mjGEOM_BOX:
        return np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * size
    if kind == G.mjGEOM_SPHERE:
        return trimesh.creation.icosphere(subdivisions=3, radius=size[0]).vertices
    if kind == G.mjGEOM_ELLIPSOID:
        return trimesh.creation.icosphere(subdivisions=3, radius=1.0).vertices * size
    if kind == G.mjGEOM_CYLINDER:
        return trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=64).vertices
    if kind == G.mjGEOM_CAPSULE:
        return trimesh.creation.capsule(radius=size[0], height=2 * size[1], count=[32, 32]).vertices
    raise ValueError(f"geom {model.geom(geom).name!r}: type {int(kind)} has no finite hull (plane/hfield/sdf)")


def geom_local_hull(model: mujoco.MjModel, geom: int) -> tuple[np.ndarray, np.ndarray]:
    """Convex hull of one geom in the GEOM frame, mm: ``(hull vertices [n,3], faces [k,3] wound outwards)``.

    Only hull vertices are returned (interior mesh vertices are dropped).  Flat or otherwise degenerate pieces are
    hulled with qhull's joggle option; if that fails too the piece has no faces (``faces`` is empty).
    """
    from scipy.spatial import ConvexHull, QhullError

    if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        verts = np.asarray(model.mesh_vert[start:start + model.mesh_vertnum[mesh]], dtype=np.float64) * 1000.0
    else:
        verts = _primitive_vertices(model, geom) * 1000.0
    hull = None
    for options in (None, "QJ Pp"):
        try:
            hull = ConvexHull(verts, qhull_options=options)
            break
        except QhullError:
            continue
    if hull is None:
        return verts, np.empty((0, 3), dtype=np.int64)
    remap = np.full(len(verts), -1)
    remap[hull.vertices] = np.arange(len(hull.vertices))
    faces = hull.simplices.copy()
    tri = verts[faces]
    flip = np.einsum("ij,ij->i", np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), hull.equations[:, :3]) < 0
    faces[flip] = faces[flip, ::-1]
    return verts[hull.vertices], remap[faces]


def geom_hull(model: mujoco.MjModel, geom: int) -> tuple[np.ndarray, np.ndarray]:
    """Convex hull of one geom as ``(vertices [n,3] in the BODY frame, mm; faces [k,3])``."""
    verts, faces = geom_local_hull(model, geom)
    return verts @ _quat_to_mat(model.geom_quat[geom]).T + model.geom_pos[geom] * 1000.0, faces


def body_collision_triangles(model: mujoco.MjModel, body: str) -> tuple[np.ndarray, np.ndarray]:
    """All collision-hull triangles of ``body``: ``(triangles [n,3,3] mm, hull vertices [m,3] mm)``, body frame."""
    tris, verts = [], []
    for geom in body_collision_geoms(model, body):
        v, f = geom_hull(model, geom)
        verts.append(v)
        if len(f):
            tris.append(v[f])
    if not verts:
        return np.empty((0, 3, 3)), np.empty((0, 3))
    return (np.concatenate(tris) if tris else np.empty((0, 3, 3))), np.concatenate(verts)


def default_workers(cap: int = 32) -> int:
    """Worker count: CPUs this process may use, capped."""
    return max(1, min(cap, len(os.sched_getaffinity(0))))


def write_json(path: str | os.PathLike, payload: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1))


def header(tool: str, scene: str | os.PathLike, **extra) -> dict:
    """Common provenance block written at the top of every result file."""
    import hashlib

    scene = Path(scene)
    return dict(tool=tool, scene=str(scene), scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                mujoco=mujoco.__version__, created=time.strftime("%Y-%m-%d %H:%M:%S"), **extra)
