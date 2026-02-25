#!/usr/bin/env bash
# run_benchmark.sh — Run all benchmark stages in sequence.
#
# Usage:
#   bash run_benchmark.sh [--skip-llm] [--skip-vlm] [--skip-mission] [--trials N]
#
# Prerequisites:
#   - source ~/ardu_ws/install/setup.bash
#   - ollama serve running with all four LLM/VLM models pulled
#   - Simulation running (required for bench_vlm capture and bench_mission)

set -euo pipefail

WS="${HOME}/ardu_ws"
SKIP_LLM=0
SKIP_VLM=0
SKIP_MISSION=0
TRIALS=""

for arg in "$@"; do
  case $arg in
    --skip-llm)     SKIP_LLM=1 ;;
    --skip-vlm)     SKIP_VLM=1 ;;
    --skip-mission) SKIP_MISSION=1 ;;
    --trials=*)     TRIALS="${arg#*=}" ;;
  esac
done

source "${WS}/install/setup.bash"
export PATH="${HOME}/.local/bin:${PATH}"
export LD_LIBRARY_PATH="${HOME}/.local/lib/ollama:${LD_LIBRARY_PATH:-}"

echo "============================================================"
echo " Drone NL Control — Full Benchmark Suite"
echo " $(date)"
echo "============================================================"

# Check Ollama
if ! ollama list &>/dev/null; then
  echo "[ERROR] Cannot reach Ollama. Run: ollama serve"
  exit 1
fi

echo ""
echo "Installed models:"
ollama list
echo ""

# ── Stage 1: LLM validity ──────────────────────────────────────────────────
if [[ $SKIP_LLM -eq 0 ]]; then
  echo "------------------------------------------------------------"
  echo " Stage 1/3 — LLM command validity"
  echo "------------------------------------------------------------"
  ros2 run drone_nl_control bench_llm
else
  echo "[SKIP] Stage 1 — LLM benchmark"
fi

# ── Stage 2: VLM validity ──────────────────────────────────────────────────
if [[ $SKIP_VLM -eq 0 ]]; then
  echo ""
  echo "------------------------------------------------------------"
  echo " Stage 2/3 — VLM binary-response validity"
  echo " (Needs simulation running for first-time image capture)"
  echo "------------------------------------------------------------"
  ros2 run drone_nl_control bench_vlm
else
  echo "[SKIP] Stage 2 — VLM benchmark"
fi

# ── Stage 3: Mission success ───────────────────────────────────────────────
if [[ $SKIP_MISSION -eq 0 ]]; then
  echo ""
  echo "------------------------------------------------------------"
  echo " Stage 3/3 — Mission success rate (all LLM × VLM combos)"
  echo " Simulation must be running and unpaused."
  echo "------------------------------------------------------------"
  if [[ -n "$TRIALS" ]]; then
    ros2 run drone_nl_control bench_mission \
      --ros-args -p trials:="${TRIALS}"
  else
    ros2 run drone_nl_control bench_mission
  fi
else
  echo "[SKIP] Stage 3 — Mission benchmark"
fi

# ── Report ─────────────────────────────────────────────────────────────────
echo ""
echo "------------------------------------------------------------"
echo " Generating report"
echo "------------------------------------------------------------"
ros2 run drone_nl_control bench_report

echo ""
echo "============================================================"
echo " All done.  Results in: ~/.drone_benchmark/"
echo "============================================================"
