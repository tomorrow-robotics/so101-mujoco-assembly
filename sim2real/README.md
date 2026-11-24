# SO-101 Sim2Real: Step 1 (M1 into the Base)

English | [中文](README_zh.md)

Two Panthera-HT 6-DoF arms with their stock parallel grippers, on a plain table, put the first SO-101 servo, M1 (an STS3215), into the Base. The arms are driven by the LLM agent Astra (`gpt-6-astra`), using a harness adapted from XPolicyLab Q0-Preview. In MuJoCo this step has succeeded three times in a row from loose parts (see the end of this page). This directory moves it to the real cell.

**The only thing you need to write is an adapter.** Implement `RobotIO` from `real/interface.py` on top of your own SDK. It reads joints, tracks joint targets, drives the grippers and does the compliant press used for the insertion.

Everything else is wired up and tested against mocked hardware:
- the agent, the plan protocol and the executor;
- IK and path planning;
- the 2 mm inter-arm clearance check;
- camera observations and depth files;
- the "fastened" signal;
- logging.

What is still unverified on hardware is listed in [REPORT.md](REPORT.md).

## 1. Prepare

### Hardware and layout
- **Two Panthera-HT arms.**
  - Their base poses must match the twin model `twin/twin.xml`. Left arm = arm "a" in simulation, with wrist camera 1868. Right arm = arm "b", with wrist camera 1416.
  - The twin was built from your lab's calibration. If you recalibrate or move an arm, update it as described in "Updating the twin" below.
- **Three RealSense cameras.**
  - One overhead (261822072968).
  - One D405 on each wrist (262422271868, 262422271416).
  - 640×480. Other serial numbers can be passed with `CAMERA_ARGS`.
- **Parts.**
  - Print the Base from Base_SO101 v12 in `parts/source/` (`Base_SO101.step` or `print-base.stl`); simulation used v12. M1 is an STS3215.
  - The task text, the skills and the agent's colour segmentation all assume a **blue Base and an orange M1**. A real STS3215 is black, so wrap M1 in orange tape or stickers. Otherwise, change the colour wording in the task card and the skills.
- **Placement.**
  - Roughly as in `layout/step1_sim_initial_layout.json`, in table coordinates: Base at (−0.095, 0.345) m with yaw 90°, M1 at (0.15, 0.245) m with yaw 135°.
  - The agent finds the parts in the images, so the placement does not have to be exact.
- **Start pose.** All arm joints at 0, grippers open. Every simulated episode started here, and the reach figures in the experience files were measured from it.
- **Emergency stop** within reach. In step 1 the Base is not fixed to the table; the left hand holds it down.

### Software (control PC, Linux x86_64)
```bash
pip install mujoco==3.9.0 numpy scipy pillow pyyaml pyrealsense2
# Python tool sandbox of the agent: a separate prefix, mounted read-only into bubblewrap, no network
conda create -y -p /opt/q0_tools python=3.11 && /opt/q0_tools/bin/pip install -r q0_harness/requirements.txt
sudo apt install bubblewrap
```
- **Model.**
  - In `real/agent.real.example.json`, set `base_url` to your OpenAI-compatible endpoint and set `python_prefix`.
  - Put the API key in the environment variable `Q0_PREVIEW_API_KEY`.
- **cuRobo** is optional: `PLANNER=curobo`, which needs a GPU and cuRobo commit 895c651. The default `mujoco` planner runs on the CPU.
- **Rendering** is needed only for the mock stage: `MUJOCO_GL=egl` with a GPU, otherwise `MUJOCO_GL=osmesa`.

### Write the adapter (required)
Start from `real/adapter_template.py` and subclass `RobotIO` for your SDK:

| Method | What it must do |
|---|---|
| `read_joints()` | Measured angles of the 6 joints of each arm in rad. Zero and sign must follow the twin's URDF. |
| `command(q, gripper, duration_s)` | Move smoothly from the previous target to `q` within `duration_s`, then keep holding it. Gripper: 1 = open, 0 = closed. The model can think for minutes between calls, so do the holding in your own high-rate loop (gravity compensation + joint PD). |
| `command_compliant(...)` | The force-limited press at the end of the insertion. Along the press direction, never push harder than `max_force_n` (10 N in step 1). Sideways, stay soft so the part centres itself. Keep the orientation firm. Reference law: `real/compliance.py` (the one used in simulation); working example: `real/mock.py`. If this is not implemented, the backend falls back to stiff position control and warns. **Do not do the insertion on hardware with that fallback.** |
| `read_gripper()`, `stop()`, `close()` | Optional: measured opening, safe stop, disconnect. |

On any fault, raise an exception: large tracking error, over-current, joint limit, lost communication. The episode then ends at once and the fault is recorded.

## 2. Run

```bash
cd sim2real
export ROBOT=your_package.your_module:YourRobot                      # your RobotIO adapter
export ROBOT_ARGS='{"left_config": "...", "right_config": "..."}'   # optional constructor kwargs
pipeline/step1_real.sh mock     # 1. no robot: mocked twin + API-free smoke agent, checks the environment
pipeline/step1_real.sh check    # 2. read-only: joints, three images with the twin's points drawn on them
pipeline/step1_real.sh move     # 3. small test motions: joint 1 of each arm +-0.05 rad, grippers; tracking error
pipeline/step1_real.sh smoke    # 4. the whole loop on the robot with the smoke agent (no-op actions), no model
Q0_PREVIEW_API_KEY=... pipeline/step1_real.sh run    # 5. Astra does step 1
```

Each stage writes to `sim2real/runs/<stage>_<time>/`. Before moving on:

| Stage | Pass criterion |
|---|---|
| mock | Both commands finish. In `mock_check_*/overlay_*.png` the dots sit on the rendered arms. |
| check | In `overlay_*.png` the red (TCP), yellow (finger pads) and cyan (link origins) dots sit on the **real** grippers and links. A uniform offset means a calibration error. Dots on the wrong part of an arm usually mean a joint zero or direction that differs from the twin. |
| move | Motions are smooth, and the settled tracking error in `check.json` is small (we suggest < 0.01 rad). |
| smoke | The episode ends normally (`end.reason = integration_smoke_complete` in `RESULT.json`), and the arms never move unexpectedly. |
| run | See below. |

**What the operator does during a run:**
- **Station prompt.** After every action that used the compliant press, the terminal asks "Is M1 fully seated in the Base?". Answer `y` when it is. Once the agent sees `station.fastened = true`, it lets go and withdraws. You can also `touch runs/<run>/FASTENED` from another terminal.
- **Stopping.** `touch runs/<run>/session/STOP` lets the agent wrap up. Ctrl-C holds the arms immediately.
- **Result.** At the end the terminal asks for the outcome: seated, hands released, optional height measurement. The answers go into `RESULT.json`.

**Settings** (environment variables):
- `SLOWDOWN=2`: the real motion runs at half the simulator's speed; the timing the agent sees is unchanged.
- `PLANS=30`: planning rounds.
- `PLANNER=mujoco|curobo`.
- `CAMERAS` / `CAMERA_ARGS`: another camera class or other serial numbers.
- `AGENT_CONFIG`.

**Outputs:**
- `RESULT.json`: operator verdict and faults.
- `trajectory.npz`: target and measured joints, gripper commands and whether the press was on, per control step.
- `session/`: every observation, reply, tool output and the conversation of the agent.
- `final_*.png`.

## 3. Updating the twin (after recalibration)
`twin/twin.xml` is a MuJoCo model holding only the two arms, the table and the cameras. It is the scene of the successful simulated episodes with the parts removed; see `twin/provenance.json`. It provides forward kinematics, IK, collision checks and the wrist-camera extrinsics.

When the calibration changes, edit:
- `a_base_link` / `b_base_link` `pos` and `quat`: the arm base poses in the table frame (left = a, right = b);
- `camera name="agent"`: pose and intrinsics of the overhead camera;
- `wrist_1868` / `wrist_1416`: wrist-camera mounts relative to link6;
- `lab_table`: the table; its top should be at z = 0.

If you calibrate the cameras yourself, you can leave the twin alone instead. Return `camera_to_world` per camera from `CameraIO.capture()` (OpenGL axes: +X right, +Y up, −Z forward), and the backend uses it directly.

## 4. Contents

| Path | Contents |
|---|---|
| `pipeline/step1_real.sh` | one-command, stage-by-stage runner |
| `real/interface.py` | **the harness control interface**: `RobotIO`, `CameraIO` |
| `real/adapter_template.py` | adapter template |
| `real/compliance.py` | reference law and simulator gains for the compliant press |
| `real/backend.py` | connects the interface to the harness: hardware executes, the twin does kinematics and collision checks |
| `real/run_real_episode.py`, `real/check_setup.py` | episode launcher; pre-run check (overlays, test motions) |
| `real/station.py` | operator-confirmed "fastened" signal and result record |
| `real/realsense.py` | `CameraIO` for the three RealSense cameras |
| `real/mock.py` | the twin as mocked hardware, for offline tests |
| `real/agent.real.example.json` | agent config template |
| `twin/` | twin model (arm collision meshes, calibrated cameras) |
| `parts/` | CAD of Base v12 and STS3215, assembled pose |
| `layout/` | initial placement used in simulation |
| `q0_harness/` | the adapted Q0 harness, plus the step-1 CAD package, task card, prompts, experience, skills and pose tools |
| `third_party/` | upstream Q0-Preview snapshot (commit `6baccd5`, Apache-2.0) and shared utilities |
| `scripts/` | simulator launcher (reference) and the cuRobo service |
| `docs/q0_preview/` | harness migration report and runbook |

## 5. Results in simulation (reference)

| Episode | Lateral | Axial | Angle | Planning rounds used |
|---|---|---|---|---|
| E1 | 0.120 mm | 0.526 mm | 0.727° | 11 |
| G1 | 0.156 mm | 0.528 mm | 0.716° | 12 |
| H1 | 0.186 mm | 0.521 mm | 0.732° | 14 |

Gates: lateral 0.25 mm; axial between −0.05 and 0.6 mm; angle 0.75°.

The procedure:
1. The left hand holds the Base down.
2. The right hand picks up M1.
3. The right hand pushes M1 along the Base's +Y with a 10 N force limit, aiming 2–3 mm past the CAD depth.
4. The Base slides about 1.7 mm on the table, so a second push is usually needed.

The agent gets no ground-truth object poses; it estimates them from three RGB-D cameras and the CAD.

## 6. Not included, and licences
- Not included:
  - simulated episode recordings;
  - the local model gateway (use your own endpoint);
  - cuRobo sources (only the commit, `895c651`, is recorded).
- `third_party/q0_preview` and the `q0_harness` adapted from it are Apache-2.0.
- The SO-101 CAD and the STS3215 model belong to their authors (TheRobotStudio / Feetech).
- The arm meshes in the twin come from the Panthera-HT SDK.
