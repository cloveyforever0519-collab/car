#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/config.env"

LOG_DIR="${PROJECT_DIR}/logs/delivery"
mkdir -p "${LOG_DIR}" "${WATCHDOG_STATE_DIR}"

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

log() {
  echo "[$(timestamp)] $*"
}

load_conda() {
  local conda_base
  if command -v conda >/dev/null 2>&1; then
    conda_base="$(conda info --base)"
  elif [ -d "${HOME}/miniconda3" ]; then
    conda_base="${HOME}/miniconda3"
  elif [ -d "${HOME}/anaconda3" ]; then
    conda_base="${HOME}/anaconda3"
  else
    log "ERROR: conda not found"
    return 1
  fi
  # shellcheck source=/dev/null
  source "${conda_base}/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}"
}

display_socket_ready() {
  local display="$1"
  local display_num="${display#:}"
  display_num="${display_num%%.*}"
  [ -n "${display_num}" ] && [ -S "/tmp/.X11-unix/X${display_num}" ]
}

resolve_xauthority() {
  local runtime_xauth="/run/user/$(id -u)/gdm/Xauthority"

  if [ -n "${CARLA_XAUTHORITY:-}" ]; then
    XAUTHORITY="${CARLA_XAUTHORITY}"
  elif [ -n "${XAUTHORITY:-}" ] && [ -r "${XAUTHORITY}" ]; then
    :
  elif [ -r "${runtime_xauth}" ]; then
    XAUTHORITY="${runtime_xauth}"
  elif [ -r "${HOME}/.Xauthority" ]; then
    XAUTHORITY="${HOME}/.Xauthority"
  fi

  if [ -n "${XAUTHORITY:-}" ]; then
    export XAUTHORITY
  fi
}

xauthority_ready() {
  [ -n "${XAUTHORITY:-}" ] && [ -r "${XAUTHORITY}" ]
}

wait_for_display() {
  local timeout_sec="${1:-120}"
  local elapsed=0
  local socket=""

  if [ -n "${CARLA_DISPLAY:-}" ]; then
    DISPLAY="${CARLA_DISPLAY}"
  fi

  while [ "${elapsed}" -lt "${timeout_sec}" ]; do
    resolve_xauthority

    if [ -n "${DISPLAY:-}" ] && display_socket_ready "${DISPLAY}" && xauthority_ready; then
      export DISPLAY
      return 0
    fi

    socket="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' 2>/dev/null | sort -V | tail -n 1 || true)"
    if [ -n "${socket}" ] && xauthority_ready; then
      DISPLAY=":${socket##*/X}"
      export DISPLAY
      return 0
    fi

    sleep 1
    elapsed="$((elapsed + 1))"
  done

  return 1
}

detect_display_resolution() {
  local fallback="${1:-1920x1080}"
  local line=""
  local resolution=""

  if command -v xrandr >/dev/null 2>&1; then
    line="$(xrandr --query 2>/dev/null | awk '/ connected primary / {print; exit}')"
    if [ -z "${line}" ]; then
      line="$(xrandr --query 2>/dev/null | awk '/ connected / {print; exit}')"
    fi
    resolution="$(printf '%s\n' "${line}" | grep -oE '[0-9]+x[0-9]+\\+[0-9]+\\+[0-9]+' | head -n 1 | cut -d+ -f1)"
  fi

  if [ -n "${resolution}" ]; then
    printf '%s\n' "${resolution}"
  else
    printf '%s\n' "${fallback}"
  fi
}

build_carla_display_args() {
  local resolution="${CARLA_RESOLUTION:-auto}"
  local width=""
  local height=""
  local args=()

  if [ "${resolution}" = "auto" ]; then
    resolution="$(detect_display_resolution "1920x1080")"
  fi

  width="${resolution%x*}"
  height="${resolution#*x}"

  if [ "${CARLA_FULLSCREEN:-1}" = "1" ]; then
    args+=("-fullscreen")
  fi

  if [ -n "${width}" ] && [ -n "${height}" ] && [ "${width}" != "${height}" ]; then
    args+=("-ResX=${width}" "-ResY=${height}")
  fi

  printf '%s\n' "${args[@]}"
}
