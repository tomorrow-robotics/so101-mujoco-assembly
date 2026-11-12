#!/usr/bin/env python
"""Model size and compile cost of a candidate scene.

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python benchmark/model_stats.py --scene scene.xml --out model_stats.json

Reports the cold compile time of the MJCF (one run, fresh process, so MuJoCo's in-process asset cache is empty), geom and
mesh counts, the size of the saved native binary (``.mjb``), its load time and the peak resident memory of this process.
Run it once per process: peak RSS is a process-lifetime maximum (it includes the Python interpreter and imports, about
``rss_after_import_mb``).  The OS page cache is not dropped, so "cold" refers to MuJoCo, not to the disk.
"""
from __future__ import annotations

import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse
import json
import resource
import tempfile
import time

import mujoco
import numpy as np

from _common import collision_mask, header, write_json


def _peak_rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0        # Linux reports kB


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="candidate scene MJCF (.xml)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    if not args.scene.endswith(".xml"):
        raise SystemExit("model_stats measures compilation, so --scene must be the MJCF (.xml)")

    rss_import = _peak_rss_mb()
    wall, cpu = time.perf_counter(), time.process_time()
    model = mujoco.MjModel.from_xml_path(args.scene)
    compile_wall, compile_cpu = time.perf_counter() - wall, time.process_time() - cpu
    rss_compile = _peak_rss_mb()

    t = time.perf_counter()
    data = mujoco.MjData(model)
    mjdata_s = time.perf_counter() - t
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "scene.mjb")
        mujoco.mj_saveModel(model, path, None)
        mjb_bytes = os.path.getsize(path)
        t = time.perf_counter()
        mujoco.MjModel.from_binary_path(path)
        mjb_load_s = time.perf_counter() - t

    active = collision_mask(model)
    mesh_geom = model.geom_type == mujoco.mjtGeom.mjGEOM_MESH
    stats = dict(
        compile_wall_s=compile_wall, compile_cpu_s=compile_cpu, mjb_load_wall_s=mjb_load_s, mjdata_alloc_wall_s=mjdata_s,
        ngeom=int(model.ngeom), n_collision_geom=int(active.sum()), n_collision_mesh_geom=int((active & mesh_geom).sum()),
        nmesh=int(model.nmesh), nmeshvert=int(model.nmeshvert), nmeshface=int(model.nmeshface),
        n_collision_meshvert=int(model.mesh_vertnum[np.unique(model.geom_dataid[active & mesh_geom])].sum()),
        nbody=int(model.nbody), nq=int(model.nq), nv=int(model.nv), nexclude=int(model.nexclude),
        mjb_size_mb=mjb_bytes / 2**20, mjdata_nbuffer_mb=int(model.nbuffer) / 2**20,
        peak_rss_mb=_peak_rss_mb(), peak_rss_after_compile_mb=rss_compile, rss_after_import_mb=rss_import)
    write_json(args.out, dict(header("model_stats", args.scene,
                                     threads=dict(OPENBLAS_NUM_THREADS=os.environ["OPENBLAS_NUM_THREADS"],
                                                  OMP_NUM_THREADS=os.environ["OMP_NUM_THREADS"])), **stats))
    print(json.dumps(stats, indent=1))


if __name__ == "__main__":
    main()
