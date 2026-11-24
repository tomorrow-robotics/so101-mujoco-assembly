---
name: push-to-seat
description: The final push that seats a part in its mating part: how well to align first, how far beyond the seat to aim, how hard to push, and what a seated part looks like.
applies_to: all
status: verified
evidence: S4B_step4_defaultgate_1119_1331 (loosened grip seated the part); station press measurements (cap: 258 trials; M1 grids); D2_step2_cap_signal_1113_1616, E1_step1_guide_1113_1727, E2_step2_guide_1113_1742; F1_step3_fork_skills_1114_0124 (failed long compliant slide)
---
## When to use
For the last few centimetres of an insertion, after the part has been aligned in free space in front of (or above) its seat and the Base is held (`hold-base-on-table`).

## Procedure
1. Align in free space, not in contact: within about 1 mm sideways and well under 1 degree of tilt is enough. Sideways error is corrected by the mating geometry on the way in; tilt is not.
2. Push straight along the insertion axis in one group of 2 to 3 seconds. Aim the target 2 mm beyond the CAD seat. The arm gives way under the insertion load, so a target exactly at the seat leaves the part about 1 mm short.
3. Travel to within about 10 mm of the seat under plain position control. Use `compliance` (with `press_direction` along the insertion axis and `max_force_n` 10) only for that last short push, or not at all. Under compliance the arm yields sideways: over a long slide a first contact pushed the hand 17.5 mm off its line and turned the part in the fingers (step 3, a 27 mm compliant slide), and M1 sagged and twisted when the compliant move started 25 mm out. About 9 N is what seating the cap took; more force did not seat more often and moved the Base.
4. Read `station.fastened` in the next observation (`station-fastened-then-release`).
5. If it is false and the part is at its depth but sits a little off (a few tenths of a millimetre or about a degree), the closed gripper may be holding it cocked in a snug fit: open the gripper in place to about 1 mm more than the part's width, without moving the wrist, and let the fit centre the part (step 4: this seated the shoulder seat at 0.09 mm and 0.6 degrees after several pushes had failed). Keep it loosely held until `station.fastened` is reported.
6. If it is false: do not lift the part out. Check tilt from a fresh fit, correct it by a small rotation about the part, and push again 2 mm beyond the seat.

## What a seated part looks like
- The push group ends "complete" with a remaining distance of 1 to 2 mm to its target, or with an arrival mismatch after a few millimetres of travel: both mean it rests on its stop.
- A fastened cap rests 0.3 to 0.5 mm above its CAD position. Do not try to remove that remainder.

{{include: so101_m1_cap_install_measured.md}}

{{include: so101_base_m1_insert_measured.md}}
