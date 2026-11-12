#!/usr/bin/env python3
"""Parity check: compare interface_sweep.py output with an acceptance JSON of the original checker.

    python benchmark/compare_reference.py --sweep interface_sweep.json --reference acceptance.json

The reference stores each interface either as ``{"continuous": {"clearance_mm": ...}}`` or with a top-level
``clearance_mm``. Interfaces without a reference clearance (e.g. ``base_holder``) are listed but not compared.
"""
import argparse
import json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--sweep", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--tolerance-mm", type=float, default=1e-6)
    args = parser.parse_args()
    sweep = json.load(open(args.sweep))["interfaces"]
    reference = json.load(open(args.reference))["interfaces"]
    worst, compared = 0.0, 0
    for name, row in sweep.items():
        entry = reference.get(name, {})
        entry = entry.get("continuous", entry)
        if "clearance_mm" not in entry:
            print(f"{name:18s} ours {row['clearance_mm']}  reference n/a (not compared)")
            continue
        ours, theirs = row["clearance_mm"], entry["clearance_mm"]
        both_colliding = ours is None and theirs is None
        difference = 0.0 if both_colliding else (float("inf") if None in (ours, theirs) else abs(ours - theirs))
        same_pass = row["passed"] == entry.get("continuous_passed", row["passed"])
        same_pair = row["closest_pair"] == entry.get("closest_pair", row["closest_pair"])
        worst, compared = max(worst, difference), compared + 1
        print(f"{name:18s} ours {ours}  reference {theirs}  |diff| {difference:.3e}  "
              f"passed_match={same_pass} pair_match={same_pair}")
    print(f"compared {compared} interfaces, max |clearance diff| = {worst:.3e} mm")
    return 0 if worst < args.tolerance_mm else 1


if __name__ == "__main__":
    raise SystemExit(main())
