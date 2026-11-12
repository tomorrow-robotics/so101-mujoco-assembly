#!/usr/bin/env bash
# Score one candidate scene with every benchmark tool (writes <out_dir>/<tool>.json).
#   bash benchmark/run_all.sh <scene.xml|scene.mjb> <out_dir> <interfaces.json> <cad_references_dir>
# <cad_references_dir> = extracted cad_references_v1.1.tar.gz (one <body>_reference_mesh.ply per part, body frame, mm).
set -euo pipefail
scene=$1; out=$2; interfaces=$3; refs=$4
here=$(cd "$(dirname "$0")" && pwd)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$out"
t() { local s=$SECONDS; "$@"; echo "[wall] $(basename "$2"): $((SECONDS - s)) s"; }
mjb=$scene
if [[ $scene != *.mjb ]]; then
  mjb=$out/_scene.mjb
  python -c "import mujoco,sys; mujoco.mj_saveModel(mujoco.MjModel.from_xml_path(sys.argv[1]), sys.argv[2], None)" "$scene" "$mjb"
fi
t python "$here/interface_sweep.py"      --scene "$mjb" --interfaces "$interfaces" --out "$out/interface_sweep.json"
t python "$here/tolerance_envelope.py"   --scene "$mjb" --interfaces "$interfaces" --out "$out/tolerance_envelope.json"
t python "$here/static_interference.py"  --scene "$mjb" --interfaces "$interfaces" --out "$out/static_interference.json"
t python "$here/model_stats.py"          --scene "$scene" --out "$out/model_stats.json"
t python "$here/step_speed_stability.py" --scene "$mjb" --out "$out/step_speed_stability.json"
t python "$here/jaw_probe.py"            --scene "$mjb" --references "$refs" --out "$out/jaw_probe.json"
mkdir -p "$out/exposed_gate"
for ref in "$refs"/*_reference_mesh.ply; do
  b=$(basename "$ref" _reference_mesh.ply)
  python "$here/exposed_gate.py" --reference "$ref" --mjb "$mjb" --body "$b" --r 0.5 --h 0.2 \
      --out "$out/exposed_gate/$b" > "$out/exposed_gate/$b.log" 2>&1 &
done
wait; echo "[done] $out"
