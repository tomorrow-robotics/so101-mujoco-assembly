---
name: reach-with-oblique-tool
description: Before choosing any tool direction or carry route: which tool tilts each arm can reach where, and how to route a carry so no leg is rejected as unreachable.
applies_to: all
status: verified
evidence: station IK measurement (robot_reach); rejections recorded in episodes F1_step3_fork_skills_1114_0124, real_cad_ctx_20plans_1113_0104, C_step2_cap_normalfix_30plans_1113_1426, D_step2_cap_experience_30plans_1113_1502
---
## When to use
Every time you pick a tool direction for a grasp, a hold or a carry target. A target the arm cannot reach is rejected before any motion ("Cartesian segment is not reachable") and costs a planning round.

## What holds on this robot
- A straight-down tool is only reachable close to the arm and low: not at all above 0.14 m, and not near the Base for either arm.
- Tools tilted 30 to 45 degrees from straight down reach the whole work area when the tool leans away from the arm's own base.
- Leaning sideways (along world x) reaches much less: at 45 degrees only 0.14 to 0.36-0.44 m from the arm base axis, and nothing above 0.14 m. In step 3 a right-arm tool leaning 45 degrees toward world -x at 0.455 m was rejected twice. To approach a part from the side, lean the tool away from the arm base and turn the jaw instead.
- Leaning back toward the arm's own base was not reachable with the probed jaw direction (see the note above the tables).
- A horizontal tool is not reachable below 0.10 m.

{{include: robot_reach.md}}

## Procedure
1. Compute the horizontal distance from the tool point to the arm's base axis and look it up in the table for the tilt you want.
2. If it is outside the range, change the tilt first (30 to 45 degrees), not the grasp point.
3. Rotating a held part by a large yaw at a far pickup spot is often unreachable. Carry in two legs: translate without rotating to the middle of the table, then rotate and go on.
4. Moves that sweep more than about 12 cm need an intermediate target.
