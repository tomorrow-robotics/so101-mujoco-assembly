# SO-101 MuJoCo Assembly Scene — v1.1 Acceptance Report

**Release:** v1.1, frozen 2026-11-12. **MuJoCo:** 3.9.0.

**Scene:** two stock Panthera-HT 6-DoF arms and the loose SO-101 parts on a plain table. Every SO-101 printed part, and the M1/M2 servos,
has CAD-derived convex collision geometry.

This report is the **baseline** for future collider methods. Every number below is produced by the scripts in
`benchmark/`, which need only the released scene, `interfaces.json` and the CAD reference meshes. To compare a new
collider set, run `benchmark/run_all.sh <scene.xml> <out_dir>` and compare against the tables below.

Hardware for the timings: Intel Xeon Platinum 8481C, one pinned core, `OPENBLAS_NUM_THREADS=1`.

## 0. Scorecard

| # | Metric | Tool | v1.1 |
|---|---|---|---|
| 1 | Interfaces passing the swept-solid check (clearance > 0.005 mm) | `interface_sweep.py` | **12 / 13** (required: 8 / 8) |
| 2 | Min interface clearance (excluding base_holder) | `interface_sweep.py` | 0.0128 mm |
| 3 | Lateral pose error tolerated by insertion (min over interfaces) | `tolerance_envelope.py` | 0.008 mm |
| 4 | Tilt tolerated by insertion (min over interfaces) | `tolerance_envelope.py` | 0.018° |
| 5 | Max penetration at an interface's assembled pose | `static_interference.py` | 0.0097 mm (base_holder) |
| 7 | Exposed CAD surface within 0.05 mm (per part, min / median) | `exposed_gate.py` | 67.2 % / 91.1 % |
| 8 | Max depth of missing CAD material | `exposed_gate.py` | 2.97 mm |
| 9 | Exposed area where the collider is fatter than the CAD by > 0.05 mm (max over parts) | `exposed_gate.py` | 0.8 % |
| 10 | Jaw-width probe, \|width error\| ≤ 0.1 mm (11 parts, 1099 probes) | `jaw_probe.py` | 95.2 %; worst part forearm_holder 66.7 %, max 3.40 mm |
| 11 | Compile time from XML | `model_stats.py` | 46.9 s |
| 12 | Step time from the loose-parts reset (median / p99) | `step_speed_stability.py` | 4.93 / 21.4 ms (p50 2.45 ms) |
| 13 | Passive stability, 2000 steps from the reset | `step_speed_stability.py` | 0 warnings, max drift 1.31 mm |
| 14 | Physical holder insertion: quasi-static, 2 N limit (one-off probe, not in `benchmark/`) | — | seated; error 0.020 mm lateral, 0.009° tilt |

## 1. Interface sweep (geometric insertion)

For each of the 13 interfaces, the moving part starts at its CAD target, with its predecessors assembled. It is
withdrawn along a straight line. Every convex piece K sweeps the exact solid conv(K ∪ K + d), and its fcl distance to
every predecessor piece must be **> 0.005 mm**.

The full model state at each check is in `interfaces.json`. That state was exported from the mainline checkpoints;
the predecessors sit where the mainline physically assembled them. This tool reproduces the original internal checker
**bit-for-bit**: the clearance difference is 0.0 mm on all 12 swept interfaces.

The other three columns:
- **δ_max:** the largest lateral offset of the start pose (8 directions ⟂ withdrawal) for which the insertion still passes.
- **θ_max:** the largest tilt (4 directions) about the part's collider centre for which it still passes.
- **Static:** the min signed distance at the assembled pose.

| Interface | Moving part | Withdrawal (world) | Travel mm | Clearance mm | δ_max mm | θ_max ° | Static mm | Required |
|---|---|---|---|---|---|---|---|---|
| base_m1 | motor (M1) | [0, −1, 0] | 80 | 0.2836 | 0.281 | 0.797 | 0.2849 | ✔ |
| base_holder | holder | [0, 0, 1] | 65 | **−0.0097 (fail)** | 0 | 0 | −0.0097 | |
| shoulder_m2 | motor_2 | [0.002, 0, 1.0] | 80 | 0.0128 | 0.008 | 0.018 | 0.0256 | ✔ |
| shoulder_upper | upper_print | [0.995, 0.102, −0.002] | 100 | 0.0190 | 0.018 | 0.018 | 0.0190 | ✔ |
| upper_m3 | motor3 | [0.995, 0.102, −0.003] | 45 | 0.0210 | 0.016 | 0.041 | 0.0239 | ✔ |
| m3_forearm | forearm | [0.002, −0.001, 1.0] | 65 | 0.0144 | 0.008 | 0.023 | 0.0194 | |
| forearm_holder | forearm_holder | [0.995, 0.102, −0.004] | 100 | 0.0154 | 0.010 | 0.053 | 0.0156 | |
| forearm_m4 | motor4 | [0.995, 0.102, −0.004] | 100 | 0.0194 | 0.014 | 0.041 | 0.0194 | ✔ |
| wrist_mount | wrist_shell | [0.004, −0.002, 1.0] | 40 | 0.1353 | 0.168 | 0.445 | 0.1353 | |
| wrist_motor5 | motor5 | [−0.102, 0.995, 0.003] | 65 | 0.0185 | 0.012 | 0.170 | 0.0575 | ✔ |
| wrist_end_frame | frame | [0.995, 0.102, −0.004] | 100 | 0.0245 | 0.020 | 0.018 | 0.0245 | |
| end_frame_motor6 | housing (M6) | [0.009, −0.051, 0.999] | 55 | 0.0342 | 0.027 | 0.100 | 0.0999 | ✔ |
| end_frame_jaw | jaw | [0.009, −0.051, 0.999] | 35 | 0.0927 | 0.088 | 0.352 | 0.1082 | ✔ |

**Known problem 1: base_holder.** The seated holder overlaps the Base by 0.0097 mm (22 geom pairs at the seat; 216 colliding pairs along the 65 mm drop).
- **Cause:** the CAD gives these side walls zero clearance.
- **Why it was accepted internally:** the internal holder check allows 0.01 mm of penetration.
- **Physical effect (measured, `holder_physics_probe.py`):** quasi-static insertion at 5 mm/s with a 2 N force limit seats the holder. Final error is 0.003 mm in height, 0.020 mm lateral and 0.009° tilt. During the descent the holder rubs the Base walls: up to 272 contacts, ≤ 0.18 N total normal force, ≤ 0.0097 mm penetration. There is no contact once seated.

**Known problem 2: tight interfaces.** Ten interfaces have only 0.013–0.035 mm of clearance. A rigid straight-line
insertion tolerates only **0.008–0.027 mm lateral error and 0.018–0.17° tilt**, about 10× less than the physical seating
gates (0.25 mm, 1°). Expect insertions to rely on contact compliance to self-centre. Only base_m1, wrist_mount and
end_frame_jaw keep δ_max ≥ 0.08 mm.

**Not covered:**
- curved, rotated or arm-driven insertion paths;
- grasp-induced pose error beyond the envelope;
- dynamic seating at interfaces other than base_holder.

## 2. Assembled-state interference

`static_interference.py` also reports the state of the last interface, with the whole arm assembled. The poses come
from the mainline checkpoint, so they include joint rotations and settling that were simulated with the older colliders.
Six part pairs penetrate there:

| Part pair | Max penetration mm | Penetrating geom pairs |
|---|---|---|
| motor (M1 output) × shoulder_fork | 0.908 | 355 |
| shoulder_fork × shoulder_seat | 0.135 | 241 |
| upper_print × motor3 | 0.126 | 8 |
| motor_2 × upper_print | 0.114 | 8 |
| shoulder_fork × motor_2 | 0.014 | 3 |
| base × holder | 0.011 | 76 |

The fork is not one of the 13 checked interfaces; its fit onto the M1 output and the seat is never swept. The
fork × M1-output overlap (0.91 mm) agrees with an interference of 0.85 mm (136 mm³) already present between the official
CAD models at the CAD assembly pose (`scores/v1.1/cad_reference_overlaps.txt`). The upper_print × motor3 and motor_2 × upper_print overlaps do not appear at their own
interfaces (+0.021 / +0.019 mm), so they come from the checkpoint pose. A checkpoint-free, CAD-pose version of this
check is not part of this release.

## 3. Collider fidelity vs CAD

The fidelity gate uses a 0.5 mm probe, 0.05 mm tolerance and the official STEP as reference (Base: a cleaned STEP). "Missing" is CAD material that the
collider does not cover; its depth is measured inside the solid.

| Part | Pieces | Within tol | Thinner > tol | Fatter > tol | Missing mm³ | Max depth mm |
|---|---|---|---|---|---|---|
| base | 948 | 100 % | 0 % | 0 % | 0 | 0 (max surface deviation 0.066) |
| motor (M1) | 698 | 100 % | 0 % | 0 % | 0 | 0 (max surface deviation 0.20) |
| motor_2 (M2) | 752 | 100 % | 0 % | 0 % | 0 | 0 |
| shoulder_seat | 585 | 96.3 % | 3.7 % | 0 % | 274 | 2.31 |
| shoulder_fork | 1226 | 94.8 % | 5.2 % | 0 % | 888 | 2.43 |
| upper_print | 1790 | 91.1 % | 8.9 % | 0 % | 1069 | 2.97 |
| forearm | 1403 | 91.1 % | 8.8 % | 0 % | 698 | 1.73 |
| jaw | 1056 | 89.7 % | 10.3 % | 0 % | 559 | 2.27 |
| frame | 1952 | 86.1 % | 13.4 % | 0.5 % | 801 | 2.45 |
| holder | 519 | 84.9 % | 15.1 % | 0 % | 1522 | 2.39 |
| wrist_shell | 1593 | 70.8 % | 28.4 % | 0.8 % | 3505 | 2.11 |
| forearm_holder | 1377 | 67.2 % | 32.4 % | 0.3 % | 2798 | 1.97 |

Servos M3–M6 (`motor3`–`motor5`, `housing`) still use the previous release's colliders and are not scored in this table.

**Known problem 3: thin colliders at mating zones.** At each interface, the colliding pieces were replaced by their
intersection with a validated older collider that has interface relief. That collider is thinner than the CAD there.
Every missing region larger than 1 mm³ lies within 0.15 mm of an insertion path.

### Largest missing regions (part body frame, mm; full list in `fidelity_locations.json`)

| Part | Volume mm³ | Depth mm | Bounding box min → max (x, y, z) |
|---|---|---|---|
| wrist_shell | 2863 | 2.11 | (−12.5, −62.3, 10.6) → (12.9, −22.5, 53.4) |
| wrist_shell | 288 | 2.03 | (−10.5, −10.1, 36.4) → (12.1, 12.1, 38.6) |
| wrist_shell | 269 | 1.67 | (−9.9, −10.5, −1.6) → (12.1, 12.1, 1.6) |
| forearm_holder | 2179 | 1.97 | (−53.7, −44.3, −26.0) → (−25.7, −16.9, 19.8) |
| forearm_holder | 295 | 1.45 | (−53.7, −52.5, −22.8) → (−33.7, −43.9, 15.6) |
| forearm_holder | 253 | 1.50 | (−53.3, −43.9, 20.8) → (−25.7, −19.1, 26.2) |
| holder | 824 | 0.95 | (−15.1, 43.7, 2.2) → (15.1, 44.7, 33.6) |
| holder | 329 | 2.39 | (17.5, 34.7, 2.2) → (22.3, 41.9, 33.6) |
| shoulder_fork | 888 | 2.43 | (−12.7, −25.7, 21.8) → (20.7, −5.9, 42.0) |
| upper_print | 615 | 1.97 | (−60.1, 4.5, −18.4) → (−34.5, 23.7, 17.0) |
| upper_print | 313 | 2.97 | (55.1, 0.7, −31.6) → (77.1, 23.3, −16.6) |
| forearm | 494 | 1.14 | (−53.7, −43.9, −20.2) → (−33.5, −24.3, 17.0) |
| frame | 337 | 2.45 | (−14.7, −15.9, −16.4) → (28.1, 18.7, −10.6) |
| jaw | 313 | 1.99 | (−9.9, −9.9, −4.8) → (9.7, 10.1, 1.6) |
| jaw | 242 | 2.27 | (−9.9, −12.1, 35.0) → (8.9, 10.1, 38.2) |
| shoulder_seat | 274 | 2.31 | (0.3, 8.1, −8.4) → (19.7, 14.5, 13.8) |

**For policies:**
- Don't put grasp contacts inside these boxes. A finger there closes up to about 2–3 mm further than on the real part (see §4).
- The configured grasp points (`base_grasp_in_base`, `motor_grasp_in_motor`) lie on Base and M1, which are 100 % within tolerance.

## 4. Jaw-width probe

Two virtual 4 mm × 4 mm pads close on the part along each of its 3 principal axes, at grid positions where the CAD
has opposite, locally flat faces under both pads: ≥ 60 % of rays hit, and the first hits spread ≤ 1 mm. Each pad casts
5×5 rays and keeps its first hit. width error = collider width − CAD
width; negative means the collider is narrower, so the jaw closes further. A probe whose collider hit is more than 10 mm from
the CAD hit counts as "no local contact" (a hole in the collider).

| Part | Probes scored | \|err\| ≤ 0.1 mm | ≤ 0.5 mm | p95 \|err\| mm | Signed min / max mm |
|---|---|---|---|---|---|
| base | 59 | 100 % | 100 % | 0.000 | −0.000 / +0.014 |
| motor (M1) | 197 | 100 % | 100 % | 0.009 | −0.000 / +0.040 |
| motor_2 (M2) | 60 | 100 % | 100 % | 0.001 | −0.001 / +0.003 |
| shoulder_fork | 59 | 100 % | 100 % | 0.001 | −0.002 / +0.000 |
| frame | 60 | 100 % | 100 % | 0.060 | −0.000 / +0.072 |
| jaw | 110 | 100 % | 100 % | 0.043 | −0.007 / +0.085 |
| holder | 125 | 95.2 % | 95.2 % | 0.011 | −0.949 / +0.014 |
| upper_print | 164 | 95.1 % | 100 % | 0.057 | −0.368 / +0.047 |
| forearm | 126 | 93.7 % | 100 % | 0.340 | −0.340 / +0.029 |
| wrist_shell | 67 | 89.6 % | 100 % | 0.300 | −0.303 / +0.024 |
| forearm_holder | 72 (+4 no local contact) | 66.7 % | 79.2 % | 1.639 | −3.397 / +0.274 |
| **all 11** | **1099** | **95.2 %** | **98.1 %** | **0.082** | **−3.397 / +0.274** |

Reading:
- Grasp widths across the outer faces are exact almost everywhere: the median |error| is 0.0001 mm. The probe mostly samples outer faces, while the thin regions in §3 lie mostly inside mating zones.
- The exceptions all read narrow, so the jaw closes further than on the real part:
  - holder: 5 % of probes, by up to 0.95 mm;
  - forearm, wrist_shell, upper_print: 5–10 % of probes, by up to 0.37 mm;
  - forearm_holder: a third of probes, by up to 3.4 mm.
- This is consistent with §3.

## 5. Model size, speed, stability

| Metric | v1.1 |
|---|---|
| Geoms (collision) | 16 009 (15 943) |
| Mesh vertices (in collision meshes) | 2 949 447 (1 902 332) |
| Compile from XML | 46.9 s wall, 71.4 s CPU; peak RSS 7.4 GB |
| Compiled model (`mj_saveModel`) | 1.10 GB, load 1.5 s (not shipped) |
| Step time from the loose-parts reset, 2000 steps, median of 3 runs | mean 4.93 ms, p50 2.45 ms, p99 21.4 ms (0.20× real time) |
| Contacts / constraint rows | mean 477 / 1439 |
| Warnings, non-finite state | 0, none |
| Max drift of any free body over 2000 steps | 1.31 mm (holder); final kinetic energy 3.4e-7 J |

## 6. Reproducibility

- `MANIFEST.json` lists the sha256 of every released file.
- The packaged scene (relative paths) was compiled from a clean directory. It matches the frozen build exactly: 16 009 geoms, and bit-identical mesh vertices, geom poses and body masses. All 15 696 files match their sha256.
- The original 13-interface acceptance was re-run on the packaged scene: 13 / 13 pass. That checker allows the holder 0.01 mm of penetration; `benchmark/` does not.
- `benchmark/` regenerates every number in §0–§5, except the one-off holder physics probe (§1). The raw output behind every number is in `scores/v1.1/`.

## 7. Known limitations (summary)

1. Holder × Base zero-clearance overlap of 0.0097 mm (§1). Measured effect on insertion: small.
2. Tight interfaces: insertion tolerates 0.008–0.027 mm lateral error and 0.02–0.17° tilt (§1).
3. Design interference at the shoulder fork, inherited from the CAD (§2).
4. Thin colliders, up to 2.97 mm, at mating zones; worst on wrist_shell and forearm_holder (§3, §4).
5. Servos M3–M6 use older colliders (§3).
