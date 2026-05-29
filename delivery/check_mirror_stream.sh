#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

if [ -f "${SCRIPT_DIR}/mirror_stream.env" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/mirror_stream.env"
fi

echo "== carla-video.service =="
systemctl --user status carla-video.service --no-pager || true

echo
echo "== RTSP listen port =="
ss -lntp | grep ":${MIRROR_RTSP_PORT:-8554}" || true

echo
echo "== ffmpeg encoders =="
if command -v "${MIRROR_FFMPEG_BIN:-ffmpeg}" >/dev/null 2>&1; then
  "${MIRROR_FFMPEG_BIN:-ffmpeg}" -hide_banner -encoders 2>/dev/null | grep -E "h264_nvenc|libx264| h264 " || true
else
  echo "ffmpeg not found"
fi

echo
echo "== mirror status =="
if [ -f "${MIRROR_STATUS_FILE:-${PROJECT_DIR}/logs/mirror_stream_status.json}" ]; then
  cat "${MIRROR_STATUS_FILE:-${PROJECT_DIR}/logs/mirror_stream_status.json}"
else
  echo "status file not found: ${MIRROR_STATUS_FILE:-${PROJECT_DIR}/logs/mirror_stream_status.json}"
fi

echo
echo "== URLs =="
echo "left : rtsp://${MIRROR_RTSP_PUBLIC_HOST:-${DELIVERY_IP}}:${MIRROR_RTSP_PORT:-8554}/carla_rear_left"
echo "right: rtsp://${MIRROR_RTSP_PUBLIC_HOST:-${DELIVERY_IP}}:${MIRROR_RTSP_PORT:-8554}/carla_rear_right"
echo "bird : rtsp://${MIRROR_RTSP_PUBLIC_HOST:-${DELIVERY_IP}}:${MIRROR_RTSP_PORT:-8554}/${MIRROR_BIRDVIEW_PATH:-carla_birdview}"
