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
6. Push in along Base +Y, starting about 10 mm in front of the seat. The measurements below ("Measured: pushing M1 into the Base") say how far to aim: 2 to 3 mm beyond the CAD depth, not at it. In the successful episode the first push aimed at the CAD depth and stopped about 0.45 mm short; a second command 0.44 mm further along the same line brought M1 to 0.04 mm short. In two later episodes (10 N `compliance`, aimed 2 to 2.5 mm beyond the CAD depth) the Base slid about 1.7 mm across the table under the first push and took up most of that overshoot, so `station.fastened` stayed false; a second push 2.5 mm further along the same line, same hold, seated and fastened M1 both times. Expect that second push rather than re-measuring in between.
7. After each push look at `station.fastened`. While it is false keep hold of M1 (released unfastened, it tips over in the Base) and push again further along Base +Y. When it is true: open the right gripper to about 40 mm in place, retreat the right hand about 75 mm toward world +X and up, open the left gripper and retreat about 60 mm toward world -X and up, then return both arms home.

**Mistakes that cost rounds**
- Asking for unreachable tool directions (left horizontal at the Base rear; right straight down at the M1 pickup; right rotation at the far pickup spot): three planning rounds were lost to "Cartesian segment is not reachable". Use the 45-degree tools above and the reach table.
- Stopping at the commanded CAD depth. The first push aimed exactly at the assembled pose and M1 stopped about 0.45 mm short because the arm gives way under the insertion load. That depth was accepted at the time and made step 2 impossible: the retaining cap jams when M1 is more than 0.05 mm short. The depth tolerance is now -0.05 to +0.6 mm.
- Trusting an image fit for the last half millimetre. After the policy had corrected the grasp from its image fits and moved M1 to 10 mm in front of the seat, the true misalignment was still 0.5 mm sideways and 1.1 degrees; its depth fits were off by several tenths of a millimetre. `station.fastened` is the reliable signal.
