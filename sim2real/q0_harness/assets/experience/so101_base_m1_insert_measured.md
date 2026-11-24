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
