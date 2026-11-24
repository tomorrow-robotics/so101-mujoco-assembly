#!/usr/bin/env bash
# Step 1 (M1 into Base) on the real cell, one stage at a time. Run from anywhere; each stage writes runs/<stage>_<time>.
#
#   pipeline/step1_real.sh mock     # offline: twin + smoke agent, no robot, no model API (run this first)
#   pipeline/step1_real.sh check    # robot + cameras, read-only: joints, overlays of the twin on the images
#   pipeline/step1_real.sh move     # + small test motions (joint 1 +-0.05 rad, grippers)
#   pipeline/step1_real.sh smoke    # whole loop on the robot with the API-free smoke agent (no-op groups)
#   pipeline/step1_real.sh run      # Astra does step 1
#
# Settings (environment variables):
#   ROBOT         module:Class of your RobotIO adapter, e.g. my_cell.panthera_io:CellRobot   (required on hardware)
#   ROBOT_ARGS    JSON keyword arguments for it                         (default {})
#   CAMERAS       module:Class of the CameraIO                          (default real.realsense:RealSenseCameras)
#   CAMERA_ARGS   JSON keyword arguments                                (default {})
#   AGENT_CONFIG  agent config JSON                                     (default real/agent.real.example.json)
#   Q0_PREVIEW_API_KEY  model API key                                   (stage run)
#   SLOWDOWN      real seconds per control step x 25                    (default 2 = half the simulator speed)
#   PLANS         planning rounds                                       (default 30)
#   PLANNER       mujoco (CPU, default) | curobo (local GPU, cuRobo 895c651)
#   RUNS          output root                                           (default <repo>/sim2real/runs)
#   MUJOCO_GL     egl (default, GPU) | osmesa (CPU) -- only the mock renders
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
STAGE="${1:-}"
RUNS="${RUNS:-$HERE/runs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROBOT_ARGS="${ROBOT_ARGS:-{\}}"
CAMERAS="${CAMERAS:-real.realsense:RealSenseCameras}"
CAMERA_ARGS="${CAMERA_ARGS:-{\}}"
AGENT_CONFIG="${AGENT_CONFIG:-$HERE/real/agent.real.example.json}"
SLOWDOWN="${SLOWDOWN:-2}"
PLANS="${PLANS:-30}"
PLANNER="${PLANNER:-mujoco}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONPATH="$HERE:$HERE/third_party/xpolicylab_shared${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$RUNS"

need_robot() { [ -n "${ROBOT:-}" ] || { echo "set ROBOT=module:Class (your RobotIO adapter, see real/adapter_template.py)"; exit 2; }; }
hw=(--robot "${ROBOT:-}" --robot-args "$ROBOT_ARGS" --cameras "$CAMERAS" --camera-args "$CAMERA_ARGS")

case "$STAGE" in
  mock)
    python real/check_setup.py --mock --move --out "$RUNS/mock_check_$STAMP"
    python real/run_real_episode.py --mock --smoke-agent --yes --out "$RUNS/mock_smoke_$STAMP"
    echo "mock OK: $RUNS/mock_check_$STAMP (overlay_*.png), $RUNS/mock_smoke_$STAMP (session/observations)" ;;
  check)
    need_robot
    python real/check_setup.py "${hw[@]}" --out "$RUNS/check_$STAMP"
    echo "look at $RUNS/check_$STAMP/overlay_*.png: the dots must sit on the real grippers and links" ;;
  move)
    need_robot
    python real/check_setup.py "${hw[@]}" --move --slowdown "$SLOWDOWN" --out "$RUNS/move_$STAMP" ;;
  smoke)
    need_robot
    python real/run_real_episode.py "${hw[@]}" --smoke-agent --slowdown "$SLOWDOWN" --planner "$PLANNER" \
        --out "$RUNS/smoke_$STAMP" ;;
  run)
    need_robot
    [ -n "${Q0_PREVIEW_API_KEY:-}" ] || { echo "set Q0_PREVIEW_API_KEY"; exit 2; }
    python real/run_real_episode.py "${hw[@]}" --agent-config "$AGENT_CONFIG" --plans "$PLANS" \
        --slowdown "$SLOWDOWN" --planner "$PLANNER" --out "$RUNS/step1_$STAMP" ;;
  *)
    sed -n '2,23p' "$0"; exit 2 ;;
esac
