---
name: fit-pose-from-cad-silhouette
description: Estimating part poses from images with the CAD meshes, and how far those estimates can be trusted (they cannot decide the last half millimetre).
applies_to: all
status: verified
evidence: pose estimates versus evaluator results in D_step2_cap_experience_30plans_1113_1502 and C2_step2_cap_resume_1113_1526; per-round fit disagreements recorded in those episodes; hole-landmark stereo fits in F1_step3_fork_skills_1114_0124; hole-landmark stereo fits in F1_step3_fork_skills_1114_0124
---
## When to use
Whenever a target is computed from where a part is: before grasping, before alignment, before pushing.

## Procedure
1. First estimate from the head view, then refine from a wrist view that sees the part without the fingers in front of it.
2. For a relative pose between two parts, fit both from the same camera in the same image; estimates of the same part from different cameras differed by 0.3 to 0.6 mm.
3. Mask out robot fingers and the other part's colour before fitting; fits that included occluded regions were off by more.
4. When a part has holes or other sharp features in a face both wrist cameras see, locate those features from the CAD in both images and fit the pose to all of them at once (one stereo fit). In step 3 the five top-face holes of M1 and of the fork, fitted this way, gave poses to a few tenths of a millimetre and degree (reprojection error 0.1 to 0.2 px), where colour silhouettes had reported a 3 to 5 degree tilt that was not there.
5. Keep colour masks strict: an orange M1 horn leaking into the yellow fork mask shifted the fork fit.
6. A part resting on the table can be fitted with its height fixed to the CAD resting height; once a part has been pushed or lifted, fit all six degrees of freedom (a Base assumed flat was in fact tilted 1.4 degrees).

## Limits
- Depth along the insertion direction read from images was wrong by about 0.5 mm near the seat (a seated, fastened cap was read as 0.8 to 1.1 mm high in two episodes).
- Use image fits to get within about 1 mm and 1 degree before entering. Use `station.fastened` to decide whether a part is seated.
