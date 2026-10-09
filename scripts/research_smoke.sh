#!/usr/bin/env bash
# One official example episode, no model/controller instrumentation.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
if [[ -f .env.testing.local ]]; then
  source .env.testing.local
else
  echo 'BLOCKED: create .env.testing.local using docs/SETUP.md' >&2
  exit 2
fi
: "${ISAAC_SIM_ROOT:?}"
: "${ISAAC_SIM_PYTHON:?}"
: "${TICVLA_BASE_MODEL_PATH:?}"
: "${TICVLA_CHECKPOINT_PATH:?}"
export TICVLA_DYNANAV_ROOT="${ROOT}/DynaNav"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
[[ -x "${ISAAC_SIM_PYTHON}" ]] || { echo 'BLOCKED: missing Isaac Python' >&2; exit 2; }
[[ -f "${ISAAC_SIM_ROOT}/VERSION" ]] || { echo 'BLOCKED: missing Isaac VERSION' >&2; exit 2; }
case "$(cat "${ISAAC_SIM_ROOT}/VERSION")" in
  5.0.0|5.0.0[-+]*) ;;
  *) echo 'BLOCKED: this reproduction requires Isaac Sim 5.0.0' >&2; exit 2 ;;
esac
[[ -f "${TICVLA_BASE_MODEL_PATH}/model.safetensors" ]] || { echo 'BLOCKED: missing base weights' >&2; exit 2; }
[[ -f "${TICVLA_CHECKPOINT_PATH}" ]] || { echo 'BLOCKED: missing checkpoint' >&2; exit 2; }

EPISODE="${1:-episode_hospital_smoke}"
case "${EPISODE}" in
  episode_hospital_smoke|episode_office_smoke|episode_outdoor_smoke|episode_warehouse_smoke) ;;
  *) echo 'Choose one episode from DynaNav/configs/benchmark_example.yaml' >&2; exit 2 ;;
esac
RUN_ID="${TICVLA_RUN_ID:-smoke-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
[[ "${RUN_ID}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Invalid run ID' >&2; exit 2; }
OUT_ROOT="${TICVLA_OUTPUT_DIR:-${ROOT}/outputs}"
[[ "${OUT_ROOT}" = /* ]] || OUT_ROOT="${ROOT}/${OUT_ROOT}"
RUN_DIR="${OUT_ROOT}/${RUN_ID}"
for existing in "${RUN_DIR}" "${ROOT}/DynaNav/logs/${RUN_ID}" "${ROOT}/DynaNav/benchmark_results/${RUN_ID}" "${ROOT}/DynaNav/configs/tmp/${RUN_ID}"; do
  [[ ! -e "${existing}" ]] || { echo "Refusing existing run path: ${existing}" >&2; exit 2; }
done
mkdir -p "${OUT_ROOT}"
mkdir "${RUN_DIR}"
{
  echo "upstream_sha=$(git rev-parse upstream/main)"
  echo "local_sha=$(git rev-parse HEAD)"
  echo "branch=$(git branch --show-current)"
  echo "episode=${EPISODE}"
  echo "cuda_visible_devices=${CUDA_VISIBLE_DEVICES}"
  echo "isaac_root=${ISAAC_SIM_ROOT}"
  echo "base_model=${TICVLA_BASE_MODEL_PATH}"
  echo "checkpoint=${TICVLA_CHECKPOINT_PATH}"
  git status --short
} > "${RUN_DIR}/provenance.txt"
git diff HEAD --binary > "${RUN_DIR}/working-tree.patch"
git ls-files --others --exclude-standard -z | xargs -0 -r sha256sum > "${RUN_DIR}/untracked-files.sha256"
sha256sum DynaNav/configs/benchmark_example.yaml "${TICVLA_BASE_MODEL_PATH}/config.json" "${TICVLA_CHECKPOINT_PATH}" > "${RUN_DIR}/inputs.sha256"

# Upstream ignores TICVLA_OUTPUT_DIR for its logs. A fresh per-episode run ID
# isolates those DynaNav/logs and configs/tmp paths without changing source.
LIMIT="${TICVLA_SMOKE_WALL_SECONDS:-900}"
[[ "${LIMIT}" =~ ^[1-9][0-9]*$ ]] || { echo 'Invalid wall timeout' >&2; exit 2; }
COMMAND=(bash "${ROOT}/DynaNav/run_benchmark.sh" "${ROOT}/DynaNav/configs/benchmark_example.yaml"
  --child --episode_name "${EPISODE}" --run_id "${RUN_ID}" --result_json "${RUN_DIR}/result.json")
printf '%q ' "${COMMAND[@]}" > "${RUN_DIR}/command.txt"
printf '\n' >> "${RUN_DIR}/command.txt"
echo "Run directory: ${RUN_DIR}"
set +e
timeout --signal=INT --kill-after=30 "${LIMIT}s" "${COMMAND[@]}" 2>&1 | tee "${RUN_DIR}/console.log"
RC=${PIPESTATUS[0]}
set -e
echo "${RC}" > "${RUN_DIR}/exit-code.txt"
echo "Exit ${RC}; inspect console/result for model, RGB, prediction, and motion evidence."
exit "${RC}"
