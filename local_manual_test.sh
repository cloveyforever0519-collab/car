#!/usr/bin/env bash
set -euo pipefail

# Local manual CARLA test runner.
# Goal: use the same delivery services/scripts as the demo host, but never
# enable boot autostart. This file is intentionally standalone so the existing
# delivery files are not modified for a local comparison test.

ACTION="${1:-start}"
PROJECT_DIR="${PROJECT_DIR:-/home/z/Carla_Project}"
CARLA_DIR="${CARLA_DIR:-/home/z/Workspace/carla_hil_project}"
CONDA_ENV="${CONDA_ENV:-carla_vcu}"
BACKEND_SCRIPT="${BACKEND_SCRIPT:-main_gui.py}"
API_PORT="${API_PORT:-8765}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_WAIT_SEC="${CARLA_WAIT_SEC:-180}"
BACKEND_WAIT_SEC="${BACKEND_WAIT_SEC:-60}"
START_VIDEO="${START_VIDEO:-1}"
START_CENTER_DISPLAY="${START_CENTER_DISPLAY:-1}"
START_WATCHDOG="${START_WATCHDOG:-1}"

USER_UNIT_DIR="${HOME}/.config/systemd/user"

UNITS=(
  carla-engine.service
  carla-backend.service
  carla-video.service
  carla-center-display.service
  delivery-watchdog.service
  delivery-watchdog.timer
)

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

need_project() {
  if [ ! -d "${PROJECT_DIR}" ]; then
    log "ERROR: PROJECT_DIR not found: ${PROJECT_DIR}"
    exit 2
  fi
  if [ ! -d "${PROJECT_DIR}/delivery/systemd" ]; then
    log "ERROR: delivery/systemd not found under ${PROJECT_DIR}"
    exit 2
  fi
  if [ ! -f "${PROJECT_DIR}/${BACKEND_SCRIPT}" ]; then
    log "ERROR: backend script not found: ${PROJECT_DIR}/${BACKEND_SCRIPT}"
    exit 2
  fi
}

load_conda() {
  if ! command -v conda >/dev/null 2>&1; then
    log "ERROR: conda not found"
    return 1
  fi
  # shellcheck source=/dev/null
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}"
}

install_units_manual_only() {
  need_project
  mkdir -p "${USER_UNIT_DIR}"
  cp "${PROJECT_DIR}/delivery/systemd/"*.service "${USER_UNIT_DIR}/"
  cp "${PROJECT_DIR}/delivery/systemd/"*.timer "${USER_UNIT_DIR}/"
  chmod +x "${PROJECT_DIR}/delivery/"*.sh "${PROJECT_DIR}/delivery/wait_for_carla.py" 2>/dev/null || true
  systemctl --user daemon-reload

  # Critical: leave everything disabled so there is no boot autostart.
  for unit in "${UNITS[@]}"; do
    systemctl --user disable "${unit}" >/dev/null 2>&1 || true
  done
  systemctl --user daemon-reload
}

set_manual_environment() {
  systemctl --user set-environment \
    CARLA_BACKEND_SCRIPT="${BACKEND_SCRIPT}" \
    CARLA_HOST="${CARLA_HOST}" \
    CARLA_PORT="${CARLA_PORT}" \
    CARLA_API_PORT="${API_PORT}"
}

clear_manual_environment() {
  systemctl --user unset-environment \
    CARLA_BACKEND_SCRIPT \
    CARLA_HOST \
    CARLA_PORT \
    CARLA_API_PORT >/dev/null 2>&1 || true
}

wait_for_carla_ready() {
  load_conda
  python "${PROJECT_DIR}/delivery/wait_for_carla.py" \
    --host "${CARLA_HOST}" \
    --port "${CARLA_PORT}" \
    --timeout "${CARLA_WAIT_SEC}" \
    --client-timeout 5
}

wait_for_backend_ready() {
  local elapsed=0
  while [ "${elapsed}" -lt "${BACKEND_WAIT_SEC}" ]; do
    if curl -fsS "http://127.0.0.1:${API_PORT}/health" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    elapsed="$((elapsed + 1))"
  done
  return 1
}

disable_autostart() {
  for unit in "${UNITS[@]}"; do
    systemctl --user disable "${unit}" >/dev/null 2>&1 || true
  done
  systemctl --user daemon-reload
}

stop_stack() {
  log "Stopping local manual stack."
  systemctl --user stop \
    delivery-watchdog.timer \
    carla-center-display.service \
    carla-video.service \
    carla-backend.service \
    carla-engine.service >/dev/null 2>&1 || true
  disable_autostart
  clear_manual_environment
  log "Stopped. Boot autostart remains disabled."
}

start_stack() {
  need_project
  if [ ! -d "${CARLA_DIR}" ]; then
    log "ERROR: CARLA_DIR not found: ${CARLA_DIR}"
    exit 2
  fi

  install_units_manual_only
  set_manual_environment

  log "Stopping old services first."
  systemctl --user stop \
    delivery-watchdog.timer \
    carla-center-display.service \
    carla-video.service \
    carla-backend.service \
    carla-engine.service >/dev/null 2>&1 || true
  sleep 2

  log "Starting CARLA engine manually through carla-engine.service."
  systemctl --user start carla-engine.service

  log "Waiting for CARLA ${CARLA_HOST}:${CARLA_PORT}."
  wait_for_carla_ready

  log "Starting backend manually through carla-backend.service using ${BACKEND_SCRIPT}."
  systemctl --user start carla-backend.service

  log "Waiting for backend API."
  if ! wait_for_backend_ready; then
    log "WARNING: backend API did not become ready within ${BACKEND_WAIT_SEC}s."
  fi

  if [ "${START_VIDEO}" = "1" ]; then
    log "Starting RTSP video service."
    systemctl --user start carla-video.service
  else
    log "Skipping RTSP video service because START_VIDEO=${START_VIDEO}."
  fi

  if [ "${START_CENTER_DISPLAY}" = "1" ]; then
    log "Starting center display service."
    systemctl --user start carla-center-display.service || true
  else
    log "Skipping center display service because START_CENTER_DISPLAY=${START_CENTER_DISPLAY}."
  fi

  if [ "${START_WATCHDOG}" = "1" ]; then
    log "Starting watchdog timer for this manual session."
    systemctl --user start delivery-watchdog.timer || true
  else
    log "Skipping watchdog because START_WATCHDOG=${START_WATCHDOG}."
  fi

  disable_autostart
  status_stack
}

status_stack() {
  echo
  echo "== active states =="
  systemctl --user is-active \
    carla-engine.service \
    carla-backend.service \
    carla-video.service \
    carla-center-display.service \
    delivery-watchdog.timer || true

  echo
  echo "== enabled states, should all be disabled =="
  for unit in carla-engine.service carla-backend.service carla-video.service carla-center-display.service delivery-watchdog.timer; do
    printf '%-32s ' "${unit}"
    systemctl --user is-enabled "${unit}" 2>/dev/null || true
  done

  echo
  echo "== ports =="
  ss -lntp 2>/dev/null | egrep ":${CARLA_PORT}|:${API_PORT}|:8554" || true

  echo
  echo "== backend health =="
  curl -fsS "http://127.0.0.1:${API_PORT}/health" 2>/dev/null \
    | python3 -m json.tool 2>/dev/null \
    | egrep 'ok|carla_connected|deployment_active|vehicle_alive|drive_mode|target_ip|world|last_result' || true

  echo
  echo "== mirror status =="
  if [ -f "${PROJECT_DIR}/logs/mirror_stream_status.json" ]; then
    python3 -m json.tool < "${PROJECT_DIR}/logs/mirror_stream_status.json" 2>/dev/null \
      | egrep 'ego_actor_id|publishing|last_error|width|height|fps|frames_out|active_view_url' || true
  else
    echo "mirror_stream_status.json not found yet"
  fi
}

smoke_test() {
  need_project
  echo "== sending AIGO command =="
  curl -sS -X POST "http://127.0.0.1:${API_PORT}/command" \
    -H "Content-Type: application/json" \
    -d '{"sendstate":"START","scene":"Town02","sky":"Sunny","sunshinetime":"Noon","drive_mode":"AIGO","loadingtransportation":"1","vehiclemodel":"Lincoln MKZ","camera_view":"follow"}' \
    | python3 -m json.tool || true

  for i in $(seq 1 12); do
    echo
    echo "== poll ${i}/12 =="
    date
    curl -sS "http://127.0.0.1:${API_PORT}/health" 2>/dev/null \
      | python3 -m json.tool 2>/dev/null \
      | egrep 'carla_connected|deployment_active|vehicle_alive|drive_mode|last_result|actor_id|speed_kmh' || true
    if [ -f "${PROJECT_DIR}/logs/mirror_stream_status.json" ]; then
      python3 -m json.tool < "${PROJECT_DIR}/logs/mirror_stream_status.json" 2>/dev/null \
        | egrep 'ego_actor_id|publishing|last_error|width|height|fps|frames_out' || true
    fi
    systemctl --user is-active carla-engine.service carla-backend.service carla-video.service || true
    sleep 10
  done

  echo
  echo "== recent engine crash lines =="
  journalctl --user -u carla-engine.service --since "5 minutes ago" --no-pager -l \
    | egrep -i 'Signal|Segmentation|Fatal|RenderThread|GameThread|Failed|Started|Stopping|Stopped' || true
}

case "${ACTION}" in
  install) install_units_manual_only; disable_autostart; echo "Installed manual units; boot autostart disabled." ;;
  start) start_stack ;;
  stop) stop_stack ;;
  restart) stop_stack; start_stack ;;
  status) status_stack ;;
  smoke) smoke_test ;;
  disable-autostart) disable_autostart; echo "Boot autostart disabled." ;;
  *)
    echo "Usage: $0 {install|start|stop|restart|status|smoke|disable-autostart}"
    exit 2
    ;;
esac
