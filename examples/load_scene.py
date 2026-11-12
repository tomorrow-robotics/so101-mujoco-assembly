#!/usr/bin/env python3
"""Load the SO-101 assembly scene, step it passively and inspect it.

    python examples/load_scene.py data/so101_mujoco_assembly_v1.1/scene.xml --cache scene.mjb

Compiling the XML is slow because of the many collision meshes, so --cache saves
the compiled model as a binary .mjb and reuses it on later runs. An .mjb is only
valid for the MuJoCo version that wrote it, so the version is added to the file
name (scene.mjb -> scene.mujoco-3.9.0.mjb). Delete the cache if you edit the scene.
"""
import argparse
import time
from pathlib import Path

import mujoco


def load(scene, cache=None):
    """Return the compiled model, using the version-tagged .mjb cache when given."""
    if cache is not None:
        cache = cache.with_name(f"{cache.stem}.mujoco-{mujoco.__version__}.mjb")
        if cache.is_file():
            t0 = time.perf_counter()
            model = mujoco.MjModel.from_binary_path(str(cache))
            print(f"loaded cache {cache} in {time.perf_counter() - t0:.2f} s")
            return model
    t0 = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(scene))
    print(f"compiled {scene} in {time.perf_counter() - t0:.2f} s")
    if cache is not None:
        mujoco.mj_saveModel(model, str(cache))
        print(f"saved cache {cache}")
    return model


def free_bodies(model):
    """Ids of the bodies attached to the world by a free joint (the loose parts)."""
    return [int(model.jnt_bodyid[j]) for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]


def collision_geoms(model, body_id):
    """Ids of a body's geoms that can collide (contype | conaffinity != 0)."""
    return [g for g in range(model.ngeom)
            if model.geom_bodyid[g] == body_id
            and (model.geom_contype[g] | model.geom_conaffinity[g]) != 0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scene", type=Path, help="path to scene.xml")
    ap.add_argument("--cache", type=Path, help="compiled-model cache, e.g. scene.mjb")
    ap.add_argument("--steps", type=int, default=500, help="passive steps to run")
    args = ap.parse_args()

    model = load(args.scene, args.cache)
    data = mujoco.MjData(model)
    print(f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh} nq={model.nq} nu={model.nu}")

    # Passive stepping: controls stay at their initial values.
    t0 = time.perf_counter()
    for _ in range(args.steps):
        mujoco.mj_step(model, data)
    ms = 1e3 * (time.perf_counter() - t0) / max(args.steps, 1)
    print(f"{args.steps} steps: {ms:.3f} ms/step, sim time {data.time:.3f} s, ncon={data.ncon}")

    bodies = free_bodies(model)
    print(f"free bodies ({len(bodies)}):")
    for b in bodies:
        x, y, z = data.xpos[b]
        print(f"  {model.body(b).name:32s} pos=({x:+.4f}, {y:+.4f}, {z:+.4f})"
              f"  collision geoms={len(collision_geoms(model, b))}")

    if bodies:  # list the collision geoms of one body
        b = bodies[0]
        geoms = collision_geoms(model, b)
        print(f"collision geoms of '{model.body(b).name}' (first 5 of {len(geoms)}):")
        for g in geoms[:5]:
            kind = mujoco.mjtGeom(model.geom_type[g]).name
            print(f"  id={g} name='{model.geom(g).name}' type={kind}")


if __name__ == "__main__":
    main()
