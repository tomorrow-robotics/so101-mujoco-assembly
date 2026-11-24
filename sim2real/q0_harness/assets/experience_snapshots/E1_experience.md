# Experience from earlier episodes

Task: step 1 (M1 into Base). This file is the station's record of earlier episodes of this task on this robot. Each episode starts with an empty memory; this file is the only thing carried over. It contains measurements and what earlier attempts did, not instructions: the current images and the CAD decide what to do now. Object poses differ between episodes, so earlier coordinates are examples, not current positions.

## Guide for this step: what worked and what did not

Written by the station from the earlier episodes of this step (their records are further down). It describes the one
procedure that ended in success and the mistakes that cost rounds. Poses are from those episodes: re-measure them from
your own images, but the part layout, the reachable tool directions and the order of actions carry over.

**Geometry (from the CAD, confirmed in the episodes)**
- The Base rests with body +Z up; in the earlier episodes its origin was near (-0.094, 0.345, 0.036) m, yaw about 90 deg, so Base +Y points along world -X.
- M1 lay on the table near (0.150, 0.245, 0.019) m, yaw about 135 deg, +Z up. Its round horn is at body -Y; check this against a CAD rendering, the 180-degree alternative looks similar.
- M1 goes into the Base through the open front (the Base -Y side), moving along Base +Y at its assembled height. It does not go in from above through the roof. Assembled pose: same rotation as the Base, offset (-0.0001, -0.0053, +0.0105) m.

**Procedure that succeeded (16 planning rounds; with the notes below it should need fewer)**
1. Left hand takes the Base itself and never lets go until M1 is fastened: grip the rear housing side walls across Base X, pad centre at Base body (0, +0.039, +0.010) m, wall width 34.7 mm, tool pointing 45 degrees down from behind (world (0.71, 0, -0.71)), jaw along Base X. A horizontal tool there was unreachable.
2. Right hand takes M1 across its X side faces at motor body (0, -0.010, +0.005) m, width 24.6 mm, tool along motor +Y and 45 degrees down. A straight-down tool at the pickup spot was unreachable.
3. Lift M1 25 mm and confirm in the wrist image that it came along.
4. Carry in two legs. Rotating M1 to the Base yaw at the far pickup spot was unreachable; first translate without rotating to the middle of the table (about 100 mm toward -X, 70 mm toward +Y, 60 mm up), then rotate to the Base yaw and go to a staging point 60 mm in front of the seat (Base -Y side) at clearance height, then lower to the assembled height. The tool direction that reached the seat was world (-0.73, 0, -0.68).
5. Re-fit both parts from the two wrist views before any contact. In the successful episode M1 turned out to be rolled 1.9 degrees in the grasp; that was corrected in free space, then M1 was moved to 10 mm in front of the seat.
6. Push in along Base +Y, starting about 10 mm in front of the seat. The measurements below ("Measured: pushing M1 into the Base") say how far to aim: 2 to 3 mm beyond the CAD depth, not at it. In the successful episode the first push aimed at the CAD depth and stopped about 0.45 mm short; a second command 0.44 mm further along the same line brought M1 to 0.04 mm short.
7. After each push look at `station.fastened`. While it is false keep hold of M1 (released unfastened, it tips over in the Base) and push again further along Base +Y. When it is true: open the right gripper to about 40 mm in place, retreat the right hand about 75 mm toward world +X and up, open the left gripper and retreat about 60 mm toward world -X and up, then return both arms home.

**Mistakes that cost rounds**
- Asking for unreachable tool directions (left horizontal at the Base rear; right straight down at the M1 pickup; right rotation at the far pickup spot): three planning rounds were lost to "Cartesian segment is not reachable". Use the 45-degree tools above and the reach table.
- Stopping at the commanded CAD depth. The first push aimed exactly at the assembled pose and M1 stopped about 0.45 mm short because the arm gives way under the insertion load. That depth was accepted at the time and made step 2 impossible: the retaining cap jams when M1 is more than 0.05 mm short. The depth tolerance is now -0.05 to +0.6 mm.
- Trusting an image fit for the last half millimetre. After the policy had corrected the grasp from its image fits and moved M1 to 10 mm in front of the seat, the true misalignment was still 0.5 mm sideways and 1.1 degrees; its depth fits were off by several tenths of a millimetre. `station.fastened` is the reliable signal.

## Measured arm reach (this robot, this table)

Kinematics only (joint limits, no collision check), measured with the IK the planner uses for single poses. "Tilt" is the angle of the tool direction from straight down: 0 deg = tool pointing straight down, 90 deg = horizontal, pointing away from the arm base. Each cell is the range of horizontal distance (m) between the tool point and the arm's base axis in which that pose is reachable; `none` = not reachable at any distance. Probed every 0.02 m from 0.10 to 0.70 m, toward world -y (the work area in front of the arms). A pose outside these ranges is rejected by the planner with "Cartesian segment is not reachable".

**left arm**, base axis at world (x, y) = (-0.332, 0.706) m

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.10-0.48 | 0.14-0.62 | 0.20-0.68 | 0.30-0.70 | none |
| 0.06 | 0.10-0.46 | 0.12-0.60 | 0.16-0.68 | 0.22-0.70 | none |
| 0.10 | 0.10-0.42 | 0.10-0.58 | 0.14-0.66 | 0.20-0.70 | 0.48-0.70 |
| 0.14 | 0.14-0.36 | 0.10-0.56 | 0.10-0.64 | 0.16-0.70 | 0.34-0.70 |
| 0.18 | none | 0.10-0.54 | 0.10-0.62 | 0.12-0.68 | 0.26-0.70 |
| 0.25 | none | 0.16-0.48 | 0.10-0.56 | 0.10-0.66 | 0.22-0.70 |

**right arm**, base axis at world (x, y) = (0.218, 0.724) m

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.10-0.48 | 0.14-0.62 | 0.20-0.68 | 0.30-0.70 | none |
| 0.06 | 0.10-0.46 | 0.12-0.60 | 0.16-0.68 | 0.22-0.70 | none |
| 0.10 | 0.10-0.42 | 0.10-0.58 | 0.14-0.66 | 0.20-0.70 | 0.48-0.70 |
| 0.14 | 0.14-0.36 | 0.10-0.56 | 0.10-0.64 | 0.16-0.70 | 0.34-0.70 |
| 0.18 | none | 0.10-0.54 | 0.10-0.62 | 0.12-0.68 | 0.26-0.70 |
| 0.25 | none | 0.16-0.48 | 0.10-0.56 | 0.10-0.66 | 0.22-0.70 |

## Measured: pushing M1 into the Base

Measured by the station in this simulator, starting from a real episode state (left gripper holding the Base by its rear housing, right gripper holding M1 by its side faces with the 45-degree tool). M1 was put 10 mm in front of its seat with a known misalignment and then pushed along Base +Y in 2.5 s. Columns give the true misalignment of M1 relative to its assembled pose at that 10 mm point, before any contact: sideways offset (with tilt under 1.6 degrees), or tilt. Numbers are "fastened by the station / trials".

| how it was pushed | sideways 0-0.5 mm | sideways 0.5-1 mm | sideways 1-1.5 mm | sideways over 1.5 mm | tilt 1.6-2.5 deg | tilt over 2.5 deg |
|---|---|---|---|---|---|---|
| position control, target at the CAD depth | 1 of 4 | 0 of 6 | 0 of 2 | 0 of 5 | 0 of 9 | 0 of 7 |
| position control, target 1 mm beyond | 3 of 4 | 1 of 6 | 0 of 2 | 0 of 5 | 0 of 9 | 0 of 7 |
| position control, target 2 mm beyond | 4 of 4 | 2 of 6 | 0 of 2 | 0 of 5 | 1 of 9 | 0 of 7 |
| position control, target 3 mm beyond | 4 of 4 | 4 of 6 | 0 of 2 | 0 of 5 | 1 of 9 | 0 of 7 |
| compliance, max_force_n 10, target 2 mm beyond | 4 of 4 | 5 of 6 | 1 of 2 | 0 of 5 | 1 of 9 | 0 of 7 |

What these measurements say:
- Aim well beyond the CAD depth. With M1 aligned to 0.5 mm and 1.3 degrees, a push aimed exactly at the CAD depth stopped 0.55 mm short (the arm gives way under the load); aimed 1 mm beyond it reached depth but stayed tilted 0.94 degrees (tolerance 0.75) and was not fastened; aimed 2 or 3 mm beyond it was fastened. M1 runs against a stop in the Base, so aiming further does not push it deeper.
- A fastened M1 rests -0.03 to 0.53 mm beyond its CAD depth (tolerance -0.05 to +0.6), 0.13 to 0.23 mm sideways (tolerance 0.25) and 0.10 to 0.75 degrees off (tolerance 0.75) in the 32 fastened trials. An image fit that shows M1 about half a millimetre too deep is the seated position.
- Alignment before entering decides the outcome. With under 0.5 mm sideways offset and under 1.6 degrees tilt, 12 of 12 pushes aimed 2 mm or more beyond were fastened; above 1 mm sideways or 2.5 degrees tilt almost none were, whatever the push.
- Do not open the gripper while `station.fastened` is false. In 105 of 105 trials where M1 had gone in but was not fastened, opening the right gripper let M1 tip over in the Base (about 7 degrees, 2 mm sideways). Push again first.
- The largest contact force along the axis was about 5 N in the fastened trials. The Base did not lift (at most 0.07 mm); held by the left gripper it slid up to 1.6 mm on the table in the fastened trials, which is allowed.
- `compliance` (see the tool schema) with max_force_n 10 over these last 10 mm fastened 11 of 33 trials, against 7 of 33 for position control with the same target. Started 25 mm in front of the seat instead, the compliant move let M1 sag and twist on the way (2 of 66 fastened): use it only for the last short push, not for carrying.

## Earlier episodes of this task (newest 2 of 2, newest first)

### step1_tight_curobo_30plans_1113_1111 (2026-11-13): INVALID (simulator defect, since fixed)

The simulator misbehaved at control step 626; the result says nothing about the plan. Only what happened before that step is kept below.
Used 16 planning rounds, 15 motion groups completed, 677 control steps; ended by: external_stop.

Targets the planner rejected before any motion:
- 1x "arm path sweeps over 12 cm; add an explicit intermediate pose"
- 1x "right Cartesian segment is not reachable; add/change explicit keypoints"
  - right tool point [0.2, 0.36, 0.16] m, tool pointing [-0.14, -0.7, -0.7], jaw axis [0.92, -0.35, 0.17]
- 1x "arrival mismatch; after-arrival gripper commands cancelled"

The policy's own summary just before the simulator defect:

> Final seating refinement based on fresh evidence, not force-search. After1.2sec hold, Base stayed table-supported and nearly stationary. Current Base arc fit obs00018 origin[-.08043690,.36652623,.036],yaw70.661929. Rigid-grip orientation relative Base~[-.005,-.031,.024]deg. Current visible7-marker translation/yaw fit with this orientation gives motor error relative CAD[+.1614,-.2449,+.3994]mm; projected CAD seat landmarks are consistently~.8pixel upper-left of observed, so still a measurable seating discrepancy. Full6D7-feature estimate is poorly conditioned; preserve established nine-marker tilt. Command this image-derived correction and compensate steady measured right tracking bias world[+.1587,+.0882,+.0173]mm from prior held target. Net right movement[-.4478,-.1742,-.4178]mm. Base remains clamped35mm by left, motor24.7mm by right. Then stationary1sec to settle/fasten; checkpoint only verifies gross stability, not micrometre accuracy. No claim of fastening yet. CAD goal R=BaseR,t=tb+Rb[-.00009943,-.00528959,.01051683]. /work/held_poses.npz updated. Next inspect against CAD, release if seated, then withdraw right safely while left holds; release/withdraw left and both home. Avoid endless noisy orientation fits: right9 established tilt, current7 only position/yaw. Never lift Base.

### real_cad_ctx_20plans_1113_0104 (2026-11-13): SUCCESS

Evaluator (true simulation state against the CAD target): final error lateral 0.181 mm, along the insertion direction -0.445 mm (negative = short of the designed depth, positive = past it), angle 0.64 deg; fastened after seating: True.
Station event: the part was seated within tolerance and fastened at control step 293 (lateral 0.180 mm, along the insertion direction -0.445 mm (negative = short of the designed depth, positive = past it), angle 0.64 deg).
Used 16 planning rounds, 15 motion groups completed, 518 control steps; ended by: external_stop.
Station note: This episode was judged with an earlier, looser depth tolerance (0.5 mm) and counted as success then. M1 was in fact 0.445 mm short of its designed depth. In the following step-2 episode the retaining cap could not seat over it: measured afterwards, the cap jams when M1 is more than 0.05 mm short. The step-1 tolerance was tightened because of this.

Targets the planner rejected before any motion:
- 2x "right Cartesian segment is not reachable; add/change explicit keypoints"
  - right tool point [0.157, 0.252, 0.095] m, tool pointing [-0.0, 0.0, -1.0], jaw axis [0.71, -0.7, -0.0]
  - right tool point [0.162, 0.245, 0.112] m, tool pointing [-0.71, -0.01, -0.71], jaw axis [0.01, -1.0, -0.0]
- 1x "left Cartesian segment is not reachable; add/change explicit keypoints"
  - left tool point [-0.179, 0.344, 0.046] m, tool pointing [1.0, 0.0, 0.0], jaw axis [-0.0, 1.0, 0.0]

The policy's own last summary in that episode (its claims were not all verified):

> Task complete based on current visual evidence: M1 remains seated in Base after release, full withdrawal, home return, and final one-second stationary hold. Both arms are empty/open and near their supplied initial positions/orientations. Base stayed on table and was braced throughout CAD-directed insertion. No further manipulation is needed; preser [...]

## Results of step 2 (retaining cap) episodes (other step of the same assembly)

- D2_step2_cap_signal_1113_1616: success; final error lateral 0.121 mm, along the insertion direction +0.468 mm (positive = above its CAD seat position), angle 0.60 deg. Station note: Continuation of D_step2_cap_experience_30plans_1113_1502: its first 16 planning rounds were re-executed exactly, then the policy went on from there, now with the station fastening signal (the earlier episode had none).
- C2_step2_cap_resume_1113_1526: failed; final error lateral 0.081 mm, along the insertion direction +0.459 mm (positive = above its CAD seat position), angle 0.32 deg. Station note: Continuation of C_step2_cap_normalfix_30plans_1113_1426 after an infrastructure cut-off (its first 15 planning rounds were re-executed exactly). The cap was seated and fastened at control step 801; the policy estimated from images that it was still about 1.1 mm high and made a small recovery lift, which tipped the Base (1.29 mm at its origin, feet still on the table). In that episode the station gave no fastening signal. Since then the station reports station.fastened in every observation.
- D_step2_cap_experience_30plans_1113_1502: failed; final error lateral 0.121 mm, along the insertion direction +0.468 mm (positive = above its CAD seat position), angle 0.60 deg. Station note: The cap was already seated and fastened (control step 689) when the policy, judging from images that it was still about 0.9 mm high, pushed again and then lifted the cap; the fastened cap took the Base up with it. In that episode the station gave no fastening signal. Since then the station reports station.fastened in every observation.
- C_step2_cap_normalfix_30plans_1113_1426: aborted; final error lateral 0.141 mm, along the insertion direction +6.201 mm (positive = above its CAD seat position), angle 0.51 deg.
