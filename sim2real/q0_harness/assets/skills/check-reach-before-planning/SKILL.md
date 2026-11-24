---
name: check-reach-before-planning
description: Before sending any motion, ask the station whether the arm can reach the targets (and a whole route) with /work/station_tools.py; a rejected plan costs a planning round, a query costs seconds.
applies_to: all
status: verified
evidence: station query channel (q0_harness/station_queries.py) answers with the same planner, joint limits and path search that accept or reject plans; earlier episodes lost most of their rounds to "Cartesian segment is not reachable" rejections (S6 21 of 30, S6b 12 of 30)
---
## When to use
Every time you are about to send a motion group whose target you have not reached before in this episode, and always
before long carries, large wrist turns, and the approach to an insertion.

## How
```python
from station_tools import check_pose, check_route
t = {'target_points': [p1, p2], 'orientation_point': o}      # exactly what you would put in the plan
check_pose('right', t)          # {'reachable': True/False, 'reason': ..., 'goal_joint_limit_margin_deg': ...}
check_route('right', [t1, t2, t3])   # legs chained from the arm's current joints; first_failed_leg tells you where it breaks
```
- Search in Python, not with plans: when a target is unreachable, try nearby variants in a loop (a different height, a
  point closer to or farther from the arm's base, the turn split into two halves, the other jaw direction) and send only
  a route whose legs all pass.
- Prefer routes whose goal joint-limit margin is at least a few degrees: a pose with less than about 2 degrees of margin
  tends to stop short ("arrival mismatch" / "IK pose mismatch") when executed.
- The answer is about the robot only. It does not check the carried part or the other arm against the assembly; keep
  those clear yourself, as in a plan.
- Ask while planning (between rounds); the station does not answer while the robot is moving.
