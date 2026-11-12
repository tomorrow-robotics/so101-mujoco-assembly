#!/usr/bin/env python
"""Passive stepping speed and stability of a candidate scene from its loose-parts reset.

    python benchmark/step_speed_stability.py --scene scene.xml --out step_speed_stability.json

Protocol (per repeat): ``mj_resetData`` (the model's default state), 100 warm-up steps, then 2000 timed ``mj_step``
calls on one pinned CPU core.  Controls stay at their reset values (nothing is commanded).  Reported per repeat and as the
median over repeats: ms/step, mean contact count, MuJoCo warning counts, non-finite state, the displacement of every
free body from its reset pose and the final kinetic energy.  Only ``mj_step`` is inside the timed region.
"""
from __future__ import annotations

import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse
import json
import time

import mujoco
import numpy as np

from _common import header, load_model, write_json


def run_once(model, free_bodies, warmup, steps):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    pos0 = data.xpos[free_bodies].copy()
    quat0 = data.xquat[free_bodies].copy()
    max_disp = np.zeros(len(free_bodies))
    step_s, ncon, nefc = np.zeros(steps), np.zeros(steps), np.zeros(steps)
    finite = True
    for i in range(warmup + steps):
        t = time.perf_counter()
        mujoco.mj_step(model, data)
        dt = time.perf_counter() - t
        if i >= warmup:
            step_s[i - warmup], ncon[i - warmup], nefc[i - warmup] = dt, data.ncon, data.nefc
        max_disp = np.maximum(max_disp, np.linalg.norm(data.xpos[free_bodies] - pos0, axis=1))
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            finite = False
            step_s, ncon, nefc = (a[:max(i - warmup, 1)] for a in (step_s, ncon, nefc))
            break
    rot = np.zeros(len(free_bodies))
    for k in range(len(free_bodies)):
        rot[k] = np.degrees(2 * np.arccos(min(1.0, abs(float(quat0[k] @ data.xquat[free_bodies[k]])))))
    mujoco.mj_energyVel(model, data)
    names = [model.body(b).name for b in free_bodies]
    final_disp = np.linalg.norm(data.xpos[free_bodies] - pos0, axis=1)
    return dict(
        ms_per_step=float(step_s.mean() * 1e3), ms_per_step_p50=float(np.median(step_s) * 1e3),
        ms_per_step_p99=float(np.percentile(step_s, 99) * 1e3), steps_timed=int(len(step_s)),
        realtime_factor=float(model.opt.timestep / step_s.mean()),
        mean_ncon=float(ncon.mean()), max_ncon=int(ncon.max()), mean_nefc=float(nefc.mean()),
        warnings={mujoco.mjtWarning(w).name: int(data.warning[w].number) for w in range(mujoco.mjtWarning.mjNWARNING)
                  if data.warning[w].number},
        n_warnings=int(sum(data.warning[w].number for w in range(mujoco.mjtWarning.mjNWARNING))),
        finite=finite, kinetic_energy_final_J=float(data.energy[1]),
        max_free_body_displacement_mm=float(max_disp.max() * 1e3),
        free_body_max_displacement_mm={n: float(d * 1e3) for n, d in zip(names, max_disp)},
        free_body_final_displacement_mm={n: float(d * 1e3) for n, d in zip(names, final_disp)},
        free_body_final_rotation_deg={n: float(r) for n, r in zip(names, rot)})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="candidate scene (.xml or .mjb)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup", type=int, default=100)
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--core", type=int, default=None, help="CPU core to pin to (default: lowest allowed core)")
    args = ap.parse_args()

    core = args.core if args.core is not None else min(os.sched_getaffinity(0))
    os.sched_setaffinity(0, {core})
    model = load_model(args.scene)
    free_bodies = np.array([model.jnt_bodyid[j] for j in range(model.njnt)
                            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE])
    start = time.perf_counter()
    repeats = [run_once(model, free_bodies, args.warmup, args.steps) for _ in range(args.repeats)]
    scalar = ("ms_per_step", "ms_per_step_p50", "ms_per_step_p99", "realtime_factor", "mean_ncon", "mean_nefc",
              "n_warnings", "kinetic_energy_final_J", "max_free_body_displacement_mm")
    median = {k: float(np.median([r[k] for r in repeats])) for k in scalar}
    median["finite"] = all(r["finite"] for r in repeats)
    median["free_body_max_displacement_mm"] = {
        n: float(np.median([r["free_body_max_displacement_mm"][n] for r in repeats]))
        for n in repeats[0]["free_body_max_displacement_mm"]}
    write_json(args.out, dict(header("step_speed_stability", args.scene, warmup=args.warmup, steps=args.steps,
                                     pinned_core=core, timestep_s=float(model.opt.timestep),
                                     threads=dict(OPENBLAS_NUM_THREADS=os.environ["OPENBLAS_NUM_THREADS"],
                                                  OMP_NUM_THREADS=os.environ["OMP_NUM_THREADS"])),
                              median=median, repeats=repeats, wall_s=time.perf_counter() - start))
    print(json.dumps({k: v for k, v in median.items() if not isinstance(v, dict)}, indent=1))
    print("warnings per repeat:", [r["warnings"] for r in repeats])


if __name__ == "__main__":
    main()
