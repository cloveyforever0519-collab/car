#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${CARLA_PROJECT_ROOT:-${HOME}/Carla_Project}"
RUNTIME_DIR="${PROJECT_DIR}/official_carla_demo/runtime"
cd "${PROJECT_DIR}"

if [ ! -f "${RUNTIME_DIR}/lib.sh" ]; then
  echo "ERROR: missing ${RUNTIME_DIR}/lib.sh"
  exit 2
fi

# shellcheck source=/dev/null
source "${RUNTIME_DIR}/lib.sh"

log "Preparing official CARLA demo backend"
chmod +x "${PROJECT_DIR}/official_carla_demo/"*.sh 2>/dev/null || true

load_conda

if ! wait_for_display "${DISPLAY_WAIT_SEC:-120}"; then
  log "ERROR: no graphical X11 display is ready"
  exit 3
fi
resolve_xauthority

export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-x11}"
export SDL_VIDEO_WINDOW_POS="${SDL_VIDEO_WINDOW_POS:-0,0}"

CARLA_ENGINE_RESOLUTION="${CARLA_ENGINE_RESOLUTION:-1920x1080}"
CARLA_FORCE_RESTART="${CARLA_FORCE_RESTART:-0}"
CARLA_ENGINE_WAIT_SEC="${CARLA_ENGINE_WAIT_SEC:-120}"
OFFICIAL_DEMO_RES="${OFFICIAL_DEMO_RES:-1920x1080}"
OFFICIAL_DEMO_FULLSCREEN="${OFFICIAL_DEMO_FULLSCREEN:-1}"

systemctl --user set-environment \
  CARLA_PROJECT_ROOT="${PROJECT_DIR}" \
  CARLA_BACKEND_SCRIPT="official_carla_demo/official_demo_gateway.py" \
  CARLA_RENDER_OFFSCREEN=0 \
  CARLA_FULLSCREEN=0 \
  CARLA_RESOLUTION="${CARLA_ENGINE_RESOLUTION}" \
  CARLA_QUALITY_LEVEL="${CARLA_QUALITY_LEVEL:-High}" \
  CENTER_DISPLAY_AUTO_START=0 \
  OFFICIAL_DEMO_RES="${OFFICIAL_DEMO_RES}" \
  OFFICIAL_DEMO_FULLSCREEN="${OFFICIAL_DEMO_FULLSCREEN}" \
  OFFICIAL_AIGO_LIGHT_MODE="${OFFICIAL_AIGO_LIGHT_MODE:-1}" \
  OFFICIAL_AIGO_DISABLE_SENSORS="${OFFICIAL_AIGO_DISABLE_SENSORS:-1}" \
  OFFICIAL_AIGO_DISABLE_SIDE_MIRRORS="${OFFICIAL_AIGO_DISABLE_SIDE_MIRRORS:-0}" \
  OFFICIAL_AI_BASELINE_DISABLE_SENSORS="${OFFICIAL_AI_BASELINE_DISABLE_SENSORS:-1}" \
  OFFICIAL_AI_BASELINE_DISABLE_SIDE_MIRRORS="${OFFICIAL_AI_BASELINE_DISABLE_SIDE_MIRRORS:-0}" \
  OFFICIAL_SIDE_MIRRORS_ALWAYS_ON="${OFFICIAL_SIDE_MIRRORS_ALWAYS_ON:-1}" \
  OFFICIAL_AI_USE_MAP_SPAWN="${OFFICIAL_AI_USE_MAP_SPAWN:-0}" \
  OFFICIAL_AI_IGNORE_LIGHTS_PERCENT="${OFFICIAL_AI_IGNORE_LIGHTS_PERCENT:-0}" \
  OFFICIAL_AI_IGNORE_SIGNS_PERCENT="${OFFICIAL_AI_IGNORE_SIGNS_PERCENT:-0}" \
  OFFICIAL_TRAFFIC_MANAGER_PORT="${OFFICIAL_TRAFFIC_MANAGER_PORT:-8000}" \
  OFFICIAL_FIXED_DELTA_SECONDS="${OFFICIAL_FIXED_DELTA_SECONDS:-0.02}" \
  OFFICIAL_MAX_SUBSTEP_DELTA_TIME="${OFFICIAL_MAX_SUBSTEP_DELTA_TIME:-0.005}" \
  OFFICIAL_MAX_SUBSTEPS="${OFFICIAL_MAX_SUBSTEPS:-16}" \
  OFFICIAL_CAMERA_SENSOR_TICK="${OFFICIAL_CAMERA_SENSOR_TICK:-0.033333333}" \
  OFFICIAL_DISPLAY_LOOP_HZ="${OFFICIAL_DISPLAY_LOOP_HZ:-60}" \
  OFFICIAL_ENABLE_SENSORS="${OFFICIAL_ENABLE_SENSORS:-1}" \
  OFFICIAL_SENSOR_UDP_PORTS="${OFFICIAL_SENSOR_UDP_PORTS:-5010}" \
  OFFICIAL_SENSOR_SUMMARY_HZ="${OFFICIAL_SENSOR_SUMMARY_HZ:-5}" \
  OFFICIAL_SENSOR_UDP_HZ="${OFFICIAL_SENSOR_UDP_HZ:-5}" \
  OFFICIAL_SENSOR_LIDAR_MAX_PPS="${OFFICIAL_SENSOR_LIDAR_MAX_PPS:-300000}" \
  OFFICIAL_SENSOR_LIDAR_MAX_HZ="${OFFICIAL_SENSOR_LIDAR_MAX_HZ:-5}" \
  CARLA_EXAMPLES_DIR="${CARLA_EXAMPLES_DIR:-${HOME}/Workspace/carla_hil_project/PythonAPI/examples}" \
  CARLA_PYTHONAPI="${CARLA_PYTHONAPI:-${HOME}/Workspace/carla_hil_project/PythonAPI/carla}" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
  SDL_VIDEODRIVER="${SDL_VIDEODRIVER}" \
  SDL_VIDEO_WINDOW_POS="${SDL_VIDEO_WINDOW_POS}"

carla_ready() {
  python "${RUNTIME_DIR}/wait_for_carla.py" \
    --host "${CARLA_HOST:-127.0.0.1}" \
    --port "${CARLA_PORT:-2000}" \
    --timeout "${1:-3}" \
    --client-timeout "${CARLA_READY_CLIENT_TIMEOUT:-1}" >/tmp/official_carla_ready.log 2>&1
}

wait_for_carla_ready() {
  local timeout="${1:-${CARLA_ENGINE_WAIT_SEC}}"
  python "${RUNTIME_DIR}/wait_for_carla.py" \
    --host "${CARLA_HOST:-127.0.0.1}" \
    --port "${CARLA_PORT:-2000}" \
    --timeout "${timeout}" \
    --client-timeout "${CARLA_READY_CLIENT_TIMEOUT:-3}"
}

systemctl --user stop carla-center-display.service carla-video.service 2>/dev/null || true
pkill -f 'ffplay.*carla_view' 2>/dev/null || true
pkill -f 'mpv.*carla_view' 2>/dev/null || true
pkill -f 'vlc.*carla_view' 2>/dev/null || true
pkill -f 'automatic_control.py|official_udp_vehicle_client.py|official_demo_gateway.py' 2>/dev/null || true

if [ "${CARLA_FORCE_RESTART}" = "1" ]; then
  log "Force restarting CARLA engine"
  systemctl --user restart carla-engine.service
  wait_for_carla_ready "${CARLA_ENGINE_WAIT_SEC}"
elif carla_ready 3; then
  log "CARLA is already ready; skipping engine restart"
elif systemctl --user is-active --quiet carla-engine.service; then
  log "CARLA engine service is active; waiting for API instead of restarting"
  if ! wait_for_carla_ready "${CARLA_ENGINE_WAIT_SEC}"; then
    log "CARLA did not become ready; restarting engine once"
    systemctl --user restart carla-engine.service
    wait_for_carla_ready "${CARLA_ENGINE_WAIT_SEC}"
  fi
else
  log "Starting CARLA engine service"
  systemctl --user start carla-engine.service
  wait_for_carla_ready "${CARLA_ENGINE_WAIT_SEC}"
fi

systemctl --user restart carla-backend.service

for _ in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:8765/health >/tmp/official_gateway_health.json 2>/dev/null; then
    python3 -m json.tool /tmp/official_gateway_health.json
    exit 0
  fi
  sleep 1
done

echo "ERROR: official gateway did not answer on http://127.0.0.1:8765/health"
systemctl --user status carla-backend.service --no-pager -l || true
exit 4
