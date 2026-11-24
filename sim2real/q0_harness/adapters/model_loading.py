"""Single entry point for loading a scene for dynamics: MuJoCo model + repair of its per-polygon mesh normals.

Why: MuJoCo 3.9.0 stores one normal per polygon of every mesh's convex hull (`mesh_polynormal`) and its multi-contact
code (multiccd, on by default) uses it. For needle-thin hull faces whose first vertices are almost collinear the
stored normal is not unit length (1e14..1e17 instead of 1; 787 polygons in 119 collider pieces of the step-2 scene).
A contact on such a face is returned 1e6..1e10 m away from the part, and a few newtons at that lever arm throw the
part away within one time step (the "numerical blow-ups" / MuJoCo resets of step 1 and step 2). Evidence and the
40-seed statistics: scripts/step2_pinpoint.py, results/step2_pinpoint/.

The repair rebuilds only the defective normals from the polygon's own vertices; meshes, hulls and every healthy
polygon are untouched. NativeMujocoBackend refuses a model that still has defective normals.
"""
import mujoco
import numpy as np

UNIT_TOL = 1e-6


def polygon_normal_defects(model):
    """Indices (into mesh_polynormal) of polygons whose stored normal is not finite or not unit length."""
    n = np.linalg.norm(model.mesh_polynormal, axis=1)
    return np.flatnonzero(~np.isfinite(n) | (np.abs(n - 1.) > UNIT_TOL))


def repair_polygon_normals(model):
    """Rebuild the defective polygon normals in place (Newell's method over the polygon's vertices, oriented away
    from the mesh centre; a polygon with no area keeps MuJoCo's direction, rescaled). -> report dict."""
    bad = polygon_normal_defects(model)
    mesh_of = np.searchsorted(model.mesh_polyadr, bad, side='right') - 1
    for k, mid in zip(bad, mesh_of):
        v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]].astype(float)
        idx = model.mesh_polyvert[model.mesh_polyvertadr[k]:model.mesh_polyvertadr[k]+model.mesh_polyvertnum[k]]
        p = v[idx]
        q = np.roll(p, -1, axis=0)
        n = np.array([np.sum((p[:, 1]-q[:, 1])*(p[:, 2]+q[:, 2])), np.sum((p[:, 2]-q[:, 2])*(p[:, 0]+q[:, 0])),
                      np.sum((p[:, 0]-q[:, 0])*(p[:, 1]+q[:, 1]))])
        if not np.isfinite(n).all() or np.linalg.norm(n) < 1e-18:
            n = np.nan_to_num(model.mesh_polynormal[k].astype(float))
            if np.linalg.norm(n) == 0.:
                n = p.mean(0) - v.mean(0)
        n /= np.linalg.norm(n)
        if n @ (p.mean(0) - v.mean(0)) < 0:
            n = -n
        model.mesh_polynormal[k] = n
    left = polygon_normal_defects(model)
    if len(left):
        raise RuntimeError(f'{len(left)} mesh polygon normals are still defective after the repair')
    return {'polygons_repaired': int(len(bad)), 'meshes_repaired': int(len(set(mesh_of.tolist()))),
            'mujoco': mujoco.__version__}


def require_unit_polygon_normals(model):
    bad = polygon_normal_defects(model)
    if len(bad):
        raise RuntimeError(f'model has {len(bad)} defective mesh polygon normals (MuJoCo multi-contact would return '
                           'garbage contacts); load the scene with q0_harness.adapters.model_loading.load_model')


def load_model(path, report=None):
    """MjModel from an MJCF/.mjb path with the polygon normals repaired. `report` (dict) receives the repair counts."""
    path = str(path)
    model = mujoco.MjModel.from_binary_path(path) if path.endswith('.mjb') else mujoco.MjModel.from_xml_path(path)
    r = repair_polygon_normals(model)
    if report is not None:
        report.update(r)
    return model
