#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

if [ -f "${SCRIPT_DIR}/mirror_stream.env" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/mirror_stream.env"
fi

load_conda

export CARLA_HOST
export CARLA_PORT
export CARLA_CONNECT_TIMEOUT
export CARLA_DEPLOY_TIMEOUT
export CARLA_RUNTIME_TIMEOUT
export CARLA_API_PORT="${API_PORT}"
export CARLA_DELIVERY_BACKEND_ONLY="1"
export MANUAL_BRIDGE_SCRIPT
export MANUAL_BRIDGE_STATUS_FILE
export CARLA_CAN_DBC
export CARLA_TCP_CAN_HOST
export CARLA_TCP_CAN_PORT
export CARLA_TCP_CAN_RECONNECT_SEC
export CARLA_CAN_REVERSE_GEARS
export CARLA_CAN_DRIVE_GEARS
export CENTER_DISPLAY_AUTO_START
export CAMERA_SINGLE_ACTIVE_STREAM="${CAMERA_SINGLE_ACTIVE_STREAM:-${MIRROR_SINGLE_ACTIVE_VIEW:-1}}"
export CAMERA_ACTIVE_STREAM_PATH="${CAMERA_ACTIVE_STREAM_PATH:-${MIRROR_ACTIVE_VIEW_PATH:-carla_view}}"
export CAMERA_STREAM_RTSP_HOST="${CAMERA_STREAM_RTSP_HOST:-${MIRROR_RTSP_PUBLIC_HOST:-${DELIVERY_IP}}}"
export CAMERA_STREAM_RTSP_PORT="${CAMERA_STREAM_RTSP_PORT:-${MIRROR_RTSP_PORT:-8554}}"

cd "${PROJECT_DIR}"

BACKEND_SCRIPT="${CARLA_BACKEND_SCRIPT:-main_gui.py}"
if [ ! -f "${PROJECT_DIR}/${BACKEND_SCRIPT}" ]; then
  log "ERROR: backend script not found: ${PROJECT_DIR}/${BACKEND_SCRIPT}"
  exit 2
fi

log "Waiting for CARLA before backend startup"
python "${SCRIPT_DIR}/wait_for_carla.py" \
  --host "${CARLA_HOST}" \
  --port "${CARLA_PORT}" \
  --timeout 240 \
  --client-timeout 5

log "Starting backend-only ${BACKEND_SCRIPT} API ${API_PORT}"
exec python "${PROJECT_DIR}/${BACKEND_SCRIPT}"
