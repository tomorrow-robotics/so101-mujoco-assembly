---
name: carry-clear-of-other-arm
description: Routing a carried part past the other hand: the planner does not check the carried part against the other arm, so keep the other wrist out of the route.
applies_to: all
status: verified
evidence: D_step2_cap_experience_30plans_1113_1502 (cap hit the right wrist during the move above the Base; fixed by moving the holding hand)
---
## When to use
Whenever one hand carries a part toward a place the other hand is working at.

## What happened
The planner checks arm against arm and arm against table, with the gripper opening, but not the part in the gripper. The far end of a carried cap hit the wrist of the hand that held the Base: the carrying arm stopped 6.5 mm short ("arrival mismatch") and the holding hand was pushed 9 mm.

## Procedure
1. Before the carry, work out where the carried part's far end will be along the route, from the measured gripper-to-part transform and the part's CAD bounds.
2. Put the holding hand on the side of the fixed part that faces away from the approach, with its wrist outside that route.
3. Carry at clearance height (part bottom at least 10 mm above everything below it) and come down only over the target.
4. If an arrival mismatch of several millimetres appears during a carry, suspect contact with the other arm: back off the way you came, then move the other hand.
