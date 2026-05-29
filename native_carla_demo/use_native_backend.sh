#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${CARLA_PROJECT_ROOT:-${HOME}/Carla_Project}"
cd "${PROJECT_DIR}"

systemctl --user set-environment \
  CARLA_PROJECT_ROOT="${PROJECT_DIR}" \
  CARLA_BACKEND_SCRIPT="native_carla_demo/native_demo_controller.py" \
  CARLA_RENDER_OFFSCREEN=0 \
  CARLA_FULLSCREEN=1 \
  CARLA_RESOLUTION=1920x1080 \
  CARLA_QUALITY_LEVEL="${CARLA_QUALITY_LEVEL:-Epic}" \
  CENTER_DISPLAY_AUTO_START=0

systemctl --user stop carla-center-display.service carla-video.service 2>/dev/null || true
pkill -f 'ffplay.*carla_view' 2>/dev/null || true
pkill -f 'mpv.*carla_view' 2>/dev/null || true
pkill -f 'vlc.*carla_view' 2>/dev/null || true

systemctl --user restart carla-engine.service
sleep "${CARLA_ENGINE_WAIT_SEC:-75}"
systemctl --user restart carla-backend.service
sleep 5

curl -sS http://127.0.0.1:8765/health | python3 -m json.tool
