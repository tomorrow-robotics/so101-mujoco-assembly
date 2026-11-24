# Sim2Real gap report: Step 1 (M1 into the Base)

English | [中文](REPORT_zh.md)

How to run is in [README.md](README.md). This page lists what is **still missing or unverified** on the real cell, most important first.

## 0. Provided and verified offline

Provided:
- the harness control interface `real/interface.py`, and `real/backend.py`, which connects it to the harness;
- the launcher, the pre-run check, the operator station signal and the RealSense camera class;
- the one-command pipeline `pipeline/step1_real.sh`.

**What the verification covers.** Everything has been run only against `real/mock.py`, the twin acting as hardware:
- A full smoke-agent episode: 3 action groups and 75 control steps. Observations, depth files and the station signal have the same format as in simulation.
- The pre-run check: overlay projection correct, test-motion tracking error about 0.
- The compliant press: 3 cm along −z in free space, final error < 0.1 mm, rotation 0.004°.
- The fallback: an adapter without the compliant method falls back to position control with a warning.

**Not verified:**
- real hardware;
- a full episode with the model (Astra) on `RealBackend`;
- `real/realsense.py`;
- local cuRobo mode.

## 1. Adapter (yours to write)

Implement `RobotIO` with your modified SDK; the template is `real/adapter_template.py`.

- **Joint convention.** Zero and sign must match the twin's URDF. The twin's start pose is all zeros, and the lower limits of joints 2 and 3 are 0. Verify with the overlays from `pipeline/step1_real.sh check`.
- **Holding.** Keep holding between calls (gravity compensation + PD). One model round can take minutes.
- **Speed.** The executor caps joint speed at 2 rad/s. With `SLOWDOWN=2` the real arm moves at most 1 rad/s.

## 2. Compliant press (required before the insertion)

The last part of the insertion uses the plan's `compliance` field:
- push along `press_direction`, never harder than `max_force_n` (10 N in step 1);
- yield sideways so the part centres itself.

In simulation this is a Cartesian impedance on top of joint-torque control. The law and gains are in `real/compliance.py`:

```
F_a = clip(K_a·e_a − D_a·v_a, −F_max, F_max)        # along the press axis, K_a = 20000 N/m, D_a = 200
F   = F_a·axis + K_l·e_lat − D_l·v_lat              # sideways, K_l = 250 N/m, D_l = 25
M   = K_r·rotvec(R_d Rᵀ) − D_r·ω                    # rotation, K_r = 150, D_r = 1.1
τ   = Jᵀ[F; M] + g(q) − 1.0·q̇ + null-space posture term
```

No force sensor is used, only commanded torque and joint state. At every control step the backend passes `command_compliant` the TCP reference pose, the press axis, the force limit, the current Jacobian and the TCP pose.

On hardware:
- Joint friction absorbs part of the push. Measure the real maximum force, for example by pushing on a scale.
- Measure how much sideways compliance you actually get.
- The gains may need retuning.

## 3. "Fastened" signal (replaced by operator confirmation)

In simulation, a station stands in for the screws. After every action it checks whether M1 is seated within the gates; if so, it fixes M1 and reports `station.fastened = true` in the observation. The agent uses this signal to decide when to let go.

On the real cell, `real/station.py` asks the operator in the terminal after every action that used the compliant press. `touch <run>/FASTENED` also sets it.

Agree on one criterion, for example: M1's top face flush with the Base's top, not tilted. To automate it later (vision or a caliper), replace `OperatorStation.after_action`.

## 4. Vision accuracy (will be clearly worse on hardware)

**Simulation.** Camera calibration is exact and depth has no noise. Even so, the best-case ICP error is about 0.18 mm / 0.09°: a step-6 test with a large, unoccluded part. The lateral gate is only 0.25 mm.

**Real cell.** Depth is noisy and the calibration has errors. The lab calibration record shows plane-distance errors of up to 6.3 mm and a median residual of 1.4 mm, an order of magnitude above the gate.

**What this means:**
- On hardware, alignment cannot rely on vision alone. It has to rely on the compliant press (section 2) plus the guidance of the Base's pocket, which has about 0.95 mm clearance.
- Box-shaped servos look symmetric, and ICP can return a pose turned by 180°. Check with `pose_tools.flip_check`.
- The D405s give good depth at close range. The overhead camera is about 1 m away, so its depth error is larger.

## 5. Part appearance and task text

The task card, the skills and the agent's colour segmentation assume a blue Base and an orange M1. A real STS3215 is black, which is harder to segment in both colour and depth.

Either:
- wrap M1 in orange tape or stickers (recommended; no text changes), or
- change the colour wording in `q0_harness/assets/tasks/task_cards.json` and the step-1 skills.

The words "In the simulator ..." in the CAD package README do no harm.

## 6. Gripper (needs measuring)

- **Commands.** In simulation the gripper takes only open, close and keep; width commands are rejected. In the adapter, map 1/0 to your gripper's open and closed positions and set the grip force.
- **Finger geometry.** The printed fingers differ from the URDF. Check the grasp-point constants (`GRASP_POINT_OFFSET_M`, `FINGERTIP_REACH_M` in `controller/robot_geometry.py`) against the real fingers.
- **In-hand motion.** In simulation, M1 turns or slips in the hand after the pick, so the agent refits the pose after lifting. Real friction will behave differently.

## 7. Other notes

- **Numbers from simulation.** The grasp points, the 2–3 mm push overshoot, the Base sliding about 1.7 mm and the need for a second push in the experience and skills are all simulation numbers. The method carries over; update the numbers after the first real episodes. Real episodes are not yet written to the experience library automatically.
- **Table friction.** The Base slides during the push and the left hand holds it down. A different table surface will slide differently.
- **"Home".** Home is wherever the arms are when the episode starts; the agent returns there at the end.
- **Simulation-only modules.** In `q0_harness/adapters/`: `mujoco_backend.py` (whose interface logic `real/backend.py` reuses), `so101_station.py`, `model_loading.py`, `replay.py`, `pod_render.py`, `pod_curobo.py` and `codex_subscription.py`. Also `scripts/run_q0_episode.py`.
