#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${HEI_REBOT_LEROBOT_CONDA_ENV:-lerobot5}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [[ -z "${CONDA_PREFIX:-}" || "$(basename "$CONDA_PREFIX")" != "$ENV_NAME" ]]; then
  if [[ -n "${CONDA_EXE:-}" ]]; then
    CONDA_BASE="$("$CONDA_EXE" info --base)"
  else
    CONDA_BASE="$(conda info --base)"
  fi
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate "$ENV_NAME"
fi

cd "$REPO_ROOT"
PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}" python -u \
  examples/hei_rebot_lift/VR_mujoco_ik/mujoco_ik/rollout_mujoco.py "$@"
