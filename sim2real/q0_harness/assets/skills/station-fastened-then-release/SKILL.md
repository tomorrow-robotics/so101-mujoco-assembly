---
name: station-fastened-then-release
description: Deciding that a part is installed and letting go of it: read station.fastened after every action, then release in place, withdraw, go home.
applies_to: all
status: verified
evidence: D2_step2_cap_signal_1113_1616 (success); F1_step3_fork_skills_1114_0124, F2b_step3_skill_1118_2040, G2_step2_chain_1118_2114 (zero-duration release rejected); failures without it in D_step2_cap_experience_30plans_1113_1502 and C2_step2_cap_resume_1113_1526
---
## When to use
After every push that could have seated the part, and before any motion of a hand that still holds the part.

## Procedure
1. Read `station.fastened` in the observation that follows the push. The station checks after every finished action and fastens the part when it is seated within tolerance.
2. If it is true: do not push, pull or re-measure. Open the gripper that holds the part in place (to at least 10 mm more than the part's width) in a group with no arm motion. Give that group `duration_s` of at least 0.2: a group with no arm motion and duration 0 is rejected before anything moves and costs a planning round (this happened in F1, F2b and G2).
3. Withdraw that hand along its own tool axis, away from the part; then withdraw the hand that holds the Base; then return both arms to their home poses and open them.
4. If it is false after a push that stopped: the part is not within tolerance. Do not lift it out to re-measure; push again as in `push-to-seat`.

## Why
A fastened part is rigidly joined to the Base. In two episodes the cap was already fastened, the policy read "still about 1 mm high" from images and lifted it: the Base came up with it (21 mm and 1.3 mm) and the episode failed on the no-lift rule.
