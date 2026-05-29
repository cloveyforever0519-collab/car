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
export MIRROR_WIDTH
export MIRROR_HEIGHT
export MIRROR_FPS
export MIRROR_BITRATE
export MIRROR_BUFSIZE
export MIRROR_QUEUE_SIZE
export MIRROR_STALE_FRAME_SEC
export MIRROR_RTSP_PUBLIC_HOST
export MIRROR_RTSP_HOST
export MIRROR_RTSP_PORT
export MIRROR_ENCODER
export MIRROR_HFLIP
export MIRROR_SINGLE_ACTIVE_VIEW
export MIRROR_ACTIVE_VIEW_PATH
export MIRROR_ENABLE_VIEW_STREAMS
export MIRROR_ENABLE_REAR_STREAMS
export MIRROR_ENABLE_BIRDVIEW_STREAM
export MIRROR_BIRDVIEW_PATH
export MIRROR_BIRDVIEW_FOV
export MIRROR_HEALTH_URL
export MIRROR_STATUS_FILE
export MIRROR_FFMPEG_BIN

cd "${PROJECT_DIR}"

log "Waiting for CARLA before mirror stream startup"
python "${SCRIPT_DIR}/wait_for_carla.py" \
  --host "${CARLA_HOST}" \
  --port "${CARLA_PORT}" \
  --timeout 240 \
  --client-timeout 5

if ! command -v "${MIRROR_FFMPEG_BIN}" >/dev/null 2>&1; then
  log "ERROR: ffmpeg not found. Install ffmpeg before enabling mirror stream."
  exit 2
fi

MEDIAMTX_PID=""
if command -v mediamtx >/dev/null 2>&1; then
  MIRROR_MEDIAMTX_BIN="$(command -v mediamtx)"
fi

if [ -x "${MIRROR_MEDIAMTX_BIN}" ]; then
  log "Starting local RTSP server ${MIRROR_MEDIAMTX_BIN} on port ${MIRROR_RTSP_PORT}"
  "${MIRROR_MEDIAMTX_BIN}" "${MIRROR_MEDIAMTX_CONFIG}" &
  MEDIAMTX_PID="$!"
  sleep 1
  if ! kill -0 "${MEDIAMTX_PID}" 2>/dev/null; then
    log "ERROR: mediamtx exited during startup. Check ${MIRROR_MEDIAMTX_CONFIG} or port ${MIRROR_RTSP_PORT}."
    exit 3
  fi
else
  log "ERROR: mediamtx not found at ${MIRROR_MEDIAMTX_BIN} and not in PATH."
  log "Put the mediamtx binary at ${PROJECT_DIR}/bin/mediamtx or install it into PATH."
  exit 3
fi

cleanup() {
  if [ -n "${MEDIAMTX_PID}" ]; then
    kill "${MEDIAMTX_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

log "Starting CARLA mirror stream service"
python "${PROJECT_DIR}/carla_mirror_stream.py"
