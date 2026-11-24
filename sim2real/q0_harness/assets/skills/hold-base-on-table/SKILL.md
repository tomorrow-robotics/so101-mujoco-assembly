---
name: hold-base-on-table
description: Keeping the Base (or the Base with parts already on it) from sliding, tipping or lifting while the other hand pushes a part into it.
applies_to: all
status: verified
evidence: operator note 2026-11-13; real_cad_ctx_20plans_1113_0104 (grasp on the rear housing); C2_step2_cap_resume_1113_1526 (two fingers on two feet); tipping with one-sided holds in D_ and C2_
---
## When to use
Before any push into the Base, and until `station.fastened` is true for the part being installed. The Base may slide on the table but must not rise more than 1 mm.

## Two holds that worked
- **Grasp the Base itself** (used when the push is horizontal, step 1): grip the rear housing side walls across Base X, pad centre at Base body (0, +0.039, +0.010) m, wall width 34.7 mm, tool 45 degrees down from behind the Base, jaw along Base X. Keep it closed and still for the whole insertion. Once the retaining cap is on, the cap wraps the housing at this height and the width to grip is 44.7 mm, not 34.7 mm.
- **One finger on each foot** (used when the push comes from above, step 2): gripper opened to 70 mm, tool 45 degrees down, jaw across the two feet, fingertips on the foot tops at Base body x = +/-0.037, y = -0.023 to -0.029, z = -0.0185 m.

## Checks
- Place the fingertips with `fingertip_center` from the observation, not a bounding-box corner of the finger: a hold computed from the corner stopped 5 mm above the feet and held nothing.
- The hold is loaded when the arm stops a few millimetres above its commanded point and stays there. If it tracks its command within a tenth of a millimetre, it is not touching.
- A hold that presses only one foot or one wing lets the Base tip when the part is pushed or pulled.
- Keep the holding wrist out of the route of the part being brought in (`carry-clear-of-other-arm`).
