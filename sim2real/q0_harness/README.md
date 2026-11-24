# Q0 harness — Panthera / MuJoCo subscription adaptation

This is a working adaptation of XPolicyLab/Q0_Preview, not a replacement harness written from scratch. The exact upstream snapshot and Apache-2.0 license are retained at `../third_party/q0_preview/`. See `../docs/q0_preview/FULL_HARNESS_MIGRATION_REPORT.md` for scope, evidence and limitations.

Active entry: `../scripts/run_q0_harness.py`. It reuses `harness/runner.Agent`, the persistent Python sandbox, observation attribution, history/archive, action protocol, and `controller/executor.Executor` / `motion` logic. The model transport uses authenticated Codex CLI with `gpt-6-astra`; API-key fallback and native Codex shell/apps/browser tools are disabled. Structured output carries Q0 function calls back to the existing Q0 tool dispatcher.

The current validated mode is text-only MuJoCo ground truth and **dry-run**. It performs two planning rounds, requiring Python namespace persistence across calls/rounds, current-pose no-op groups, and native feedback. It asserts no change to qpos/qvel/ctrl/equality or simulated time. It does not perform an assembly trajectory.

```bash
cd <repo>/mujoco_so101
export PYTHONPATH=/tmp/q0-mujoco-runtime:$PWD
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
codex login status  # must say ChatGPT; model inference access also required
python scripts/run_q0_harness.py \
  --scene /tmp/q0-step1-scene/scene.xml \
  --out results/q0_harness_migration/new_unique_run \
  --python-prefix /tmp/q0-preview-python \
  --bwrap ~/.codex/packages/standalone/releases/0.167.2-x86_64-unknown-linux-musl/codex-resources/bwrap
```

The parent must be allowed to start bubblewrap. Model-generated Python always runs inside bubblewrap, not on the host. This container needs `sandbox_mount_proc=false`; `/proc` is empty, optional proc-based memory statistics are unavailable, but network/process/user/mount isolation and resource limits remain enabled. Failure to create isolation stops the harness.

Temporary environments used here: a dedicated Python 3.11 venv with original `requirements.txt` for model tools, and separate MuJoCo 3.9.0 bindings for the native backend. Do not install MuJoCo or mount simulation assets/credentials in the Python calculation sandbox. The original installation and RoboDojo deployment scripts are retained as upstream references; they are not the active entry points for this adaptation.

Physical motion code in `adapters/mujoco_backend.py` uses existing native IK and torque PD, with original model limits. Its `allow_motion` defaults to false, and the validated launcher fixes it to false. Real grasps, load/support checks, carried-object collision prediction, contact tuning, straight insertion and fastening must be validated in the subsequent trajectory phase. Do not interpret the smoke's `COMPLETE` as assembly success. Width commands are explicitly rejected until the actual Panthera pad-gap mapping is validated. Keep/open/close retain the Q0 command semantics and map to native actuator closure.
