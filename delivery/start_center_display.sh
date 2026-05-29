#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

if [ -f "${SCRIPT_DIR}/mirror_stream.env" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/mirror_stream.env"
fi

CENTER_DISPLAY_URL="${CENTER_DISPLAY_URL:-rtsp://${MIRROR_RTSP_PUBLIC_HOST:-${DELIVERY_IP}}:${MIRROR_RTSP_PORT:-8554}/${MIRROR_ACTIVE_VIEW_PATH:-carla_view}}"
CENTER_DISPLAY_HEALTH_URL="${CENTER_DISPLAY_HEALTH_URL:-http://127.0.0.1:${API_PORT:-8765}/health}"
CENTER_DISPLAY_STATUS_FILE="${CENTER_DISPLAY_STATUS_FILE:-${MIRROR_STATUS_FILE:-${PROJECT_DIR}/logs/mirror_stream_status.json}}"
CENTER_DISPLAY_PLAYER="${CENTER_DISPLAY_PLAYER:-auto}"
CENTER_DISPLAY_POLL_SEC="${CENTER_DISPLAY_POLL_SEC:-1}"
CENTER_DISPLAY_WAIT_FOR_VEHICLE="${CENTER_DISPLAY_WAIT_FOR_VEHICLE:-1}"
CENTER_DISPLAY_STOP_WHEN_STREAM_NOT_READY="${CENTER_DISPLAY_STOP_WHEN_STREAM_NOT_READY:-0}"
CENTER_DISPLAY_ALWAYS_ON_TOP="${CENTER_DISPLAY_ALWAYS_ON_TOP:-0}"
CENTER_DISPLAY_FULLSCREEN="${CENTER_DISPLAY_FULLSCREEN:-0}"
CENTER_DISPLAY_WINDOW_SIZE="${CENTER_DISPLAY_WINDOW_SIZE:-1920x1080}"

log "Waiting for graphical display for center-screen kiosk"
if ! wait_for_display 120; then
  log "ERROR: no graphical X11 display is ready"
  exit 1
fi
resolve_xauthority

choose_player() {
  if [ "${CENTER_DISPLAY_PLAYER}" != "auto" ]; then
    printf '%s\n' "${CENTER_DISPLAY_PLAYER}"
    return
  fi
  if command -v ffplay >/dev/null 2>&1; then
    printf '%s\n' "ffplay"
  elif command -v mpv >/dev/null 2>&1; then
    printf '%s\n' "mpv"
  elif command -v vlc >/dev/null 2>&1; then
    printf '%s\n' "vlc"
  else
    printf '%s\n' ""
  fi
}

PLAYER="$(choose_player)"
if [ -z "${PLAYER}" ]; then
  log "ERROR: no RTSP player found. Install ffmpeg/ffplay, mpv, or vlc."
  exit 2
fi

get_reload_key() {
  python3 - "$CENTER_DISPLAY_HEALTH_URL" <<'PY'
import json
import sys
import urllib.request

url = sys.argv[1]
try:
    with urllib.request.urlopen(url, timeout=1.0) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    print(data.get("camera_stream_reload_key") or data.get("camera_view") or "unknown")
except Exception:
    print("unreachable")
PY
}

vehicle_alive() {
  python3 - "$CENTER_DISPLAY_HEALTH_URL" <<'PY'
import json
import sys
import urllib.request

url = sys.argv[1]
try:
    with urllib.request.urlopen(url, timeout=1.0) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    print("1" if data.get("vehicle_alive") else "0")
except Exception:
    print("0")
PY
}

stream_ready() {
  python3 - "$CENTER_DISPLAY_STATUS_FILE" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
try:
    data = json.loads(path.read_text(encoding="utf-8"))
    stream = (data.get("streams") or {}).get("carla_view") or {}
    age = stream.get("last_frame_age_sec")
    fresh = isinstance(age, (int, float)) and age <= 3.0
    ok = (
        data.get("running")
        and data.get("carla_connected")
        and data.get("ego_actor_id") is not None
        and stream.get("publishing")
        and stream.get("camera_attached")
        and fresh
        and not (data.get("last_error") or stream.get("last_error"))
    )
    print("1" if ok else "0")
except Exception:
    print("0")
PY
}

get_stream_key() {
  python3 - "$CENTER_DISPLAY_HEALTH_URL" "$CENTER_DISPLAY_STATUS_FILE" <<'PY'
import json
import sys
import urllib.request
from pathlib import Path

health_url, status_path = sys.argv[1], Path(sys.argv[2])
health_key = "unreachable"
try:
    with urllib.request.urlopen(health_url, timeout=1.0) as resp:
        health = json.loads(resp.read().decode("utf-8"))
    health_key = health.get("camera_stream_reload_key") or health.get("camera_view") or "unknown"
except Exception:
    pass

try:
    data = json.loads(status_path.read_text(encoding="utf-8"))
    stream = (data.get("streams") or {}).get("carla_view") or {}
    parts = [
        str(health_key),
        str(data.get("ego_actor_id")),
        str(data.get("active_view")),
        str(stream.get("sensor_id")),
        str(stream.get("ffmpeg_pid")),
    ]
    print(":".join(parts))
except Exception:
    print(health_key)
PY
}

get_stream_url() {
  python3 - "$CENTER_DISPLAY_STATUS_FILE" "$CENTER_DISPLAY_URL" <<'PY'
import json
import sys
from pathlib import Path

status_path = Path(sys.argv[1])
fallback = sys.argv[2]
try:
    data = json.loads(status_path.read_text(encoding="utf-8"))
except Exception:
    print(fallback)
    raise SystemExit

stream = (data.get("streams") or {}).get("carla_view") or {}
url = (
    data.get("active_view_url")
    or data.get("center_view_url")
    or stream.get("rtsp_url")
    or fallback
)
print(url)
PY
}

start_player() {
  local stream_url
  local window_width="${CENTER_DISPLAY_WINDOW_SIZE%x*}"
  local window_height="${CENTER_DISPLAY_WINDOW_SIZE#*x}"
  local fullscreen_args=()
  stream_url="$(get_stream_url)"
  log "Starting center-screen player ${PLAYER}: ${stream_url}"
  case "${PLAYER}" in
    ffplay)
      ffplay_extra_args=()
      if [ "${CENTER_DISPLAY_ALWAYS_ON_TOP}" = "1" ]; then
        ffplay_extra_args+=("-alwaysontop")
      fi
      if [ "${CENTER_DISPLAY_FULLSCREEN}" = "1" ]; then
        fullscreen_args+=("-fs" "-noborder")
        window_width="1920"
        window_height="1080"
      fi
      ffplay \
        -hide_banner \
        -loglevel warning \
        -nostats \
        -rtsp_transport tcp \
        -fflags nobuffer \
        -flags low_delay \
        -framedrop \
        -an \
        -left 0 \
        -top 0 \
        -x "${window_width}" \
        -y "${window_height}" \
        "${fullscreen_args[@]}" \
        "${ffplay_extra_args[@]}" \
        -window_title "CARLA Center View" \
        "${stream_url}" &
      ;;
    mpv)
      mpv_extra_args=()
      if [ "${CENTER_DISPLAY_FULLSCREEN}" = "1" ]; then
        mpv_extra_args+=("--fullscreen")
      else
        mpv_extra_args+=("--geometry=${CENTER_DISPLAY_WINDOW_SIZE}+0+0")
      fi
      mpv \
        --really-quiet \
        --force-window=immediate \
        --no-osc \
        --no-input-default-bindings \
        --profile=low-latency \
        "${mpv_extra_args[@]}" \
        "${stream_url}" &
      ;;
    vlc|cvlc)
      vlc_extra_args=()
      if [ "${CENTER_DISPLAY_FULLSCREEN}" = "1" ]; then
        vlc_extra_args+=("--fullscreen")
      fi
      cvlc \
        --no-video-title-show \
        --network-caching=100 \
        "${vlc_extra_args[@]}" \
        "${stream_url}" &
      ;;
    *)
      log "ERROR: unsupported CENTER_DISPLAY_PLAYER=${PLAYER}"
      exit 2
      ;;
  esac
  PLAYER_PID="$!"
}

stop_player() {
  if [ -n "${PLAYER_PID:-}" ] && kill -0 "${PLAYER_PID}" 2>/dev/null; then
    kill "${PLAYER_PID}" 2>/dev/null || true
    wait "${PLAYER_PID}" 2>/dev/null || true
  fi
  PLAYER_PID=""
}

cleanup() {
  stop_player
}
trap cleanup EXIT

log "Center-screen kiosk ready on DISPLAY=${DISPLAY}, XAUTHORITY=${XAUTHORITY:-unset}"
LAST_KEY=""
PLAYER_PID=""

while true; do
  if [ "${CENTER_DISPLAY_WAIT_FOR_VEHICLE}" = "1" ] && [ "$(vehicle_alive)" != "1" ]; then
    stop_player
    LAST_KEY=""
    sleep "${CENTER_DISPLAY_POLL_SEC}"
    continue
  fi
  if [ "${CENTER_DISPLAY_STOP_WHEN_STREAM_NOT_READY}" = "1" ] && [ "$(stream_ready)" != "1" ]; then
    stop_player
    LAST_KEY=""
    sleep "${CENTER_DISPLAY_POLL_SEC}"
    continue
  fi
  CURRENT_KEY="$(get_stream_key)"
  if [ -z "${PLAYER_PID}" ] || ! kill -0 "${PLAYER_PID}" 2>/dev/null; then
    start_player
    LAST_KEY="${CURRENT_KEY}"
  elif [ "${CURRENT_KEY}" != "unreachable" ] && [ "${CURRENT_KEY}" != "${LAST_KEY}" ]; then
    log "Camera view changed: ${LAST_KEY} -> ${CURRENT_KEY}; restarting center-screen player"
    stop_player
    start_player
    LAST_KEY="${CURRENT_KEY}"
  fi
  sleep "${CENTER_DISPLAY_POLL_SEC}"
done
