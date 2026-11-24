---
name: grasp-then-lift-test
description: After closing the gripper on a part: confirm it is really held and measure how it sits in the hand before carrying it.
applies_to: all
status: verified
evidence: used in every episode that carried a part (real_cad_ctx_20plans_1113_0104, C_, D_, C2_ step-2 episodes)
---
## When to use
Right after closing on a part and before any carry. The gripper state in the observation is a command echo: it does not tell you whether the part is held.

## Procedure
1. Open the gripper wider than the part before approaching (10 to 15 mm more than its width) and approach along the tool axis.
2. Close with `held_width_mm` set to the part's width at the pads.
3. Lift 25 mm straight up without rotating (about 0.8 s).
4. In the wrist image the part must stay fixed relative to the fingers while table and shadow move; in the head image it must have risen.
5. Fit the part's CAD silhouette in the wrist view now and save the measured gripper-to-part transform to a file in /work (not only a Python variable). Parts sat up to 2 degrees off in the hand; use the measured transform, not the commanded one, for every later target.

## Signs of failure
- The part stays on the table in the head view: reopen and regrasp, do not carry.
- The fitted pose in the hand changes between two views by more than about 0.3 mm or 0.5 degrees: re-measure before inserting.
