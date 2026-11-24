---
name: measure-poses-with-depth
description: Measure a part's pose to a fraction of a millimetre from the cameras' depth images and the CAD with /work/tools/pose_tools.py (and check how a held part sits in the fingers), instead of hand-written silhouette fits.
applies_to: all
status: verified
evidence: station accuracy test against simulator truth (step-6 assembly): from a starting guess 5 mm / 5 deg off, the upper arm seen by the head and right-wrist cameras came out 0.18 mm / 0.09 deg from its true pose; started at the true pose the fit stays within 0.18 mm / 0.09 deg
---
## What you get
Every observation now has `depth` (one float32 image per camera, metres along the camera axis, files under /work/depth,
listed in observation['depth']['files']). /work/tools/pose_tools.py turns colour + depth + the CAD into a 6D pose:

```python
import sys; sys.path.insert(0, '/work/tools'); import pose_tools as pt
obs = pt.load('/work/observations/obs_00012')
T, info = pt.estimate_pose(obs, 'upper_print', color=(26, 191, 204), init=T_guess)   # 4x4 world pose of the part body frame
rel, twist = pt.in_hand(obs, 'right', T, reference=rel_after_lift)                   # pose in the gripper; turn about the jaw since then
```

## How to use it well
- Always give `init`: your previous estimate, the pose from the CAD relative to a part you measured, or a coarse
  silhouette estimate. It needs to be within about 5 mm and 6 degrees; without it a flat part can come out flipped.
- Give the part's colour as seen in the images (RGB); other objects of a similar colour farther than 8 cm from the
  guess are ignored automatically.
- Trust a fit with `rms_mm` around 1 mm or less and `inlier_fraction` above about 0.8, from at least a few hundred
  points (`points_used`). Fewer points (a part seen only from far away or mostly hidden): look with a wrist camera first.
- After grasping and after every large wrist turn, measure the part in the hand with `in_hand(...)` against the pose
  right after the lift: a long part can turn in the fingers (step 6: 12 to 35 degrees) and the hand pose that seats it
  then changes, or becomes unreachable.
- It measures; it does not decide seating. `station.fastened` still decides whether a part is installed.

## Check which way round
Box-like parts (servos) look almost the same turned 180 degrees about a body axis. After a fit, run `pt.flip_check(obs, part, color, T)`: it refits from the three turned poses and says whether the depth can tell them apart. If it cannot, check a distinguishing feature (the step guide names it) before acting.
