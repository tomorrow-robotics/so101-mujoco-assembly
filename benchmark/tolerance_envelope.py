#!/usr/bin/env python3
"""Insertion tolerance envelope of the SO-101 assembly interfaces for any candidate MuJoCo scene.

How much pose error of the moving part does each straight-line insertion tolerate? The moving part's target pose is
perturbed, then swept along the unchanged withdrawal direction with the same convex sweep and the same pass rule
(clearance > 0.005 mm) as ``interface_sweep.py``.

    python benchmark/tolerance_envelope.py --scene scene.xml --interfaces interfaces.json --out out.json [--workers N]

Perturbations (rigid transforms of the moving part's world vertices; qpos is not touched)
-----------------------------------------------------------------------------------------
* Lateral: offset δ in the plane perpendicular to the withdrawal direction, 8 directions 45° apart.
  ``delta_max_mm`` = largest δ for which all 8 directions pass (bisection on [0, 1] mm, resolution 0.002 mm).
* Tilt: rotation θ about 2 orthogonal axes perpendicular to the withdrawal direction, both signs (4 cases), pivoting
  at the AABB centre of the moving part's collision pieces at the target.
  ``theta_max_deg`` = largest θ for which all 4 cases pass (bisection on [0, 3]°, resolution 0.01°).

The in-plane basis is deterministic: ``u = normalize(e × d)`` with ``e`` the world axis least aligned with the
withdrawal direction ``d``, and ``v = d × u``. Lateral direction k is ``cos(45°k) u + sin(45°k) v``; the tilt axes are
``u`` and ``v``.

Each case is bisected independently, assuming pass/fail is monotone in the perturbation magnitude; the envelope is
the minimum over the cases, and the limiting case is the one attaining it. A value equal to the search range means
"at least that much" (``saturated``). If the unperturbed interface fails, both limits are 0 and
``nominal_failed`` is set.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

import interface_sweep as core

LATERAL_RANGE_MM, LATERAL_RESOLUTION_MM, LATERAL_DIRECTIONS = 1.0, 0.002, 8
TILT_RANGE_DEG, TILT_RESOLUTION_DEG = 3.0, 0.01
SEARCH_CAP_MM = 0.1      # distance cap during the search: only pass/fail matters, so far pairs are pruned early


def in_plane_basis(direction) -> tuple[np.ndarray, np.ndarray]:
    """Deterministic orthonormal basis (u, v) of the plane perpendicular to ``direction``."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    u = np.cross(np.eye(3)[np.argmin(np.abs(d))], d)
    u /= np.linalg.norm(u)
    return u, np.cross(d, u)


def cases(direction) -> list[dict]:
    """The 8 lateral and 4 tilt perturbation cases of an interface."""
    u, v = in_plane_basis(direction)
    out = []
    for k in range(LATERAL_DIRECTIONS):
        angle = 2 * np.pi * k / LATERAL_DIRECTIONS
        out.append(dict(kind="lateral", label=f"{360 * k // LATERAL_DIRECTIONS}deg", angle_deg=360.0 * k / LATERAL_DIRECTIONS,
                        vector_world=(np.cos(angle) * u + np.sin(angle) * v).tolist()))
    for axis_name, axis in (("u", u), ("v", v)):
        for sign in (1, -1):
            out.append(dict(kind="tilt", label=f"{'+' if sign > 0 else '-'}{axis_name}",
                            vector_world=(sign * axis).tolist()))
    return out


def perturbation(case: dict, magnitude: float, pivot_mm: np.ndarray) -> np.ndarray:
    """4x4 world transform (mm) of ``case`` at ``magnitude`` (mm for lateral, degrees for tilt)."""
    vector = np.asarray(case["vector_world"])
    if case["kind"] == "lateral":
        return core.rigid_transform(translation_mm=magnitude * vector)
    quat, rotation = np.zeros(4), np.zeros(9)
    mujoco.mju_axisAngle2Quat(quat, vector, np.deg2rad(magnitude))
    mujoco.mju_quat2Mat(rotation, quat)
    return core.rigid_transform(rotation.reshape(3, 3), pivot_mm=pivot_mm)


def bisect(evaluate, upper: float, resolution: float) -> dict:
    """Largest passing magnitude in [0, upper], given that 0 passes and pass/fail is monotone."""
    result = evaluate(upper)
    evaluations = 1
    if result["passed"]:
        return dict(max_pass=upper, saturated=True, first_fail=None, fail=None, evaluations=evaluations)
    low, high, fail = 0.0, upper, result
    while high - low > resolution * (1 + 1e-9):
        middle = (low + high) / 2
        result = evaluate(middle)
        evaluations += 1
        if result["passed"]:
            low = middle
        else:
            high, fail = middle, result
    return dict(max_pass=low, saturated=False, first_fail=high, fail=fail, evaluations=evaluations)


def _nominal(name: str) -> tuple[str, dict]:
    return name, core.context(name).sweep()


def _run_case(task: tuple[str, int]) -> tuple[str, int, dict]:
    name, index = task
    start = time.time()
    sweep = core.context(name)
    case = cases(sweep.spec["withdrawal_world"])[index]
    pivot = sweep.pivot_mm()
    lateral = case["kind"] == "lateral"
    found = bisect(lambda magnitude: sweep.sweep(perturbation(case, magnitude, pivot), cap_mm=SEARCH_CAP_MM),
                   LATERAL_RANGE_MM if lateral else TILT_RANGE_DEG,
                   LATERAL_RESOLUTION_MM if lateral else TILT_RESOLUTION_DEG)
    fail = found.pop("fail")
    unit = "mm" if lateral else "deg"
    row = dict(case=case["label"], vector_world=case["vector_world"], saturated=found["saturated"],
               evaluations=found["evaluations"], seconds=round(time.time() - start, 2))
    row[f"max_pass_{unit}"] = found["max_pass"]
    row[f"first_fail_{unit}"] = found["first_fail"]
    if fail is not None:      # geometry state at the smallest failing magnitude that was evaluated
        row.update(pair_at_limit=fail["closest_pair"], clearance_at_fail_mm=fail["clearance_mm"],
                   colliding_pair_count_at_fail=fail["colliding_pair_count"],
                   invalid_near_hulls_at_fail=fail["invalid_near_hulls"])
    return name, index, row


def _envelope(rows: list[dict], unit: str) -> dict:
    limit = min(rows, key=lambda row: row[f"max_pass_{unit}"])
    return dict(value=limit[f"max_pass_{unit}"], saturated=limit["saturated"], limiting_case=limit["case"],
                limiting_vector_world=limit["vector_world"], pair_at_limit=limit.get("pair_at_limit"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scene", required=True, help="candidate scene (.xml or .mjb)")
    parser.add_argument("--interfaces", required=True, help="interface data file (json)")
    parser.add_argument("--out", required=True, help="output json")
    parser.add_argument("--workers", type=int, default=None, help="processes (default: available CPUs)")
    args = parser.parse_args()

    start = time.time()
    core.init_globals(args.scene, args.interfaces)
    names = core.interface_names()
    specs = core.load_interfaces(args.interfaces)
    with core.make_pool(args.workers, len(names)) as pool:
        nominal = dict(pool.map(_nominal, names))
    case_count = LATERAL_DIRECTIONS + 4
    tasks = [(name, index) for name in names if nominal[name]["passed"] for index in range(case_count)]
    per_case: dict[str, dict[int, dict]] = {name: {} for name in names}
    if tasks:
        with core.make_pool(args.workers, len(tasks)) as pool:
            for name, index, row in pool.map(_run_case, tasks):
                per_case[name][index] = row

    results = {}
    for name in names:
        spec, base = specs[name], nominal[name]
        entry = dict(moving=spec["moving"], required=bool(spec.get("required", False)),
                     nominal_clearance_mm=base["clearance_mm"], nominal_passed=base["passed"],
                     nominal_failed=not base["passed"])
        if base["passed"]:
            rows = [per_case[name][index] for index in range(case_count)]
            lateral = _envelope(rows[:LATERAL_DIRECTIONS], "mm")
            tilt = _envelope(rows[LATERAL_DIRECTIONS:], "deg")
            entry.update(delta_max_mm=lateral["value"], delta_saturated=lateral["saturated"],
                         delta_limiting_direction=lateral["limiting_case"],
                         delta_limiting_vector_world=lateral["limiting_vector_world"],
                         delta_pair_at_limit=lateral["pair_at_limit"],
                         theta_max_deg=tilt["value"], theta_saturated=tilt["saturated"],
                         theta_limiting_direction=tilt["limiting_case"],
                         theta_limiting_axis_world=tilt["limiting_vector_world"],
                         theta_pair_at_limit=tilt["pair_at_limit"],
                         tilt_pivot_world_mm=None, lateral_cases=rows[:LATERAL_DIRECTIONS],
                         tilt_cases=rows[LATERAL_DIRECTIONS:])
        else:
            entry.update(delta_max_mm=0.0, theta_max_deg=0.0, delta_limiting_direction=None,
                         theta_limiting_direction=None, delta_pair_at_limit=base["closest_pair"],
                         theta_pair_at_limit=base["closest_pair"])
        results[name] = entry
        print(f"{name:18s} nominal {'PASS' if base['passed'] else 'FAIL'}  "
              f"delta_max {entry['delta_max_mm']:.3f} mm [{entry['delta_limiting_direction']}]  "
              f"theta_max {entry['theta_max_deg']:.3f} deg [{entry['theta_limiting_direction']}]", flush=True)

    # Tilt pivots are cheap to recompute in the parent (kinematics only).
    for name in names:
        if results[name]["nominal_passed"]:
            results[name]["tilt_pivot_world_mm"] = core.context(name).pivot_mm().tolist()

    output = dict(tool="tolerance_envelope", scene=str(args.scene), scene_sha256=core.sha256(args.scene),
                  interfaces_file=str(args.interfaces), interfaces_sha256=core.sha256(args.interfaces),
                  mujoco=mujoco.__version__, pass_threshold_mm=core.PASS_MM,
                  lateral=dict(range_mm=LATERAL_RANGE_MM, resolution_mm=LATERAL_RESOLUTION_MM,
                               directions=LATERAL_DIRECTIONS),
                  tilt=dict(range_deg=TILT_RANGE_DEG, resolution_deg=TILT_RESOLUTION_DEG, cases=4,
                            pivot="AABB centre of the moving part's collision pieces at the target"),
                  search_distance_cap_mm=SEARCH_CAP_MM, wall_seconds=round(time.time() - start, 1),
                  interfaces=results)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(output, indent=1) + "\n")
    print(f"wall={output['wall_seconds']} s -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
