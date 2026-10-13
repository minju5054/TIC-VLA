REPO="$(pwd)"
RUN_ID="api-e16-$(date -u +%Y%m%dT%H%M%S)-$$"
RUN_DIR="$REPO/outputs/$RUN_ID"

(
  set -euo pipefail
  cd "$REPO"

  source /home/gpuadmin/Workspace/TIC-VLA/.env.testing.local
  export TICVLA_DYNANAV_ROOT="$REPO/DynaNav"
  export ISAAC_SIM_ROOT="/home/gpuadmin/isaacsim"
  export ISAAC_SIM_PYTHON="$ISAAC_SIM_ROOT/python.sh"
  export CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
  unset LD_LIBRARY_PATH PYTHONPATH PYTHONEXE

  test ! -e "$RUN_DIR"
  test ! -e "$REPO/DynaNav/logs/$RUN_ID"
  mkdir -p "$REPO/outputs"
  mkdir "$RUN_DIR"

  cat "$ISAAC_SIM_ROOT/VERSION" > "$RUN_DIR/isaac-version.txt"

  bash DynaNav/run_benchmark.sh DynaNav/configs/benchmark_full.yaml \
    --child --episode_name episode_16 --run_id "$RUN_ID" \
    --result_json "$RUN_DIR/result.json" \
    2>&1 | tee "$RUN_DIR/console.log"
)
