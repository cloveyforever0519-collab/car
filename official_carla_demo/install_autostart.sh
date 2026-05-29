#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${CARLA_PROJECT_ROOT:-${HOME}/Carla_Project}"
RUNTIME_DIR="${PROJECT_DIR}/official_carla_demo/runtime"
DELIVERY_DIR="${PROJECT_DIR}/delivery"
USER_UNIT_DIR="${HOME}/.config/systemd/user"
ENGINE_DROPIN_DIR="${USER_UNIT_DIR}/carla-engine.service.d"
BACKEND_DROPIN_DIR="${USER_UNIT_DIR}/carla-backend.service.d"

cd "${PROJECT_DIR}"

if [ ! -f "${RUNTIME_DIR}/systemd/carla-engine.service" ] || [ ! -f "${RUNTIME_DIR}/systemd/carla-backend.service" ]; then
  echo "ERROR: runtime systemd units are missing under ${RUNTIME_DIR}/systemd"
  exit 2
fi

mkdir -p "${USER_UNIT_DIR}"

# Reset only this project's user units/drop-ins so an older V13/hotfix
# configuration cannot leak into a clean V12 restore.
systemctl --user stop carla-video.service carla-backend.service carla-engine.service 2>/dev/null || true
systemctl --user disable official-carla-demo.service carla-center-display.service delivery-watchdog.timer 2>/dev/null || true
rm -f "${USER_UNIT_DIR}/official-carla-demo.service"
rm -rf "${ENGINE_DROPIN_DIR}" "${BACKEND_DROPIN_DIR}" "${USER_UNIT_DIR}/carla-video.service.d"

cp "${RUNTIME_DIR}/systemd/carla-engine.service" "${USER_UNIT_DIR}/carla-engine.service"
cp "${RUNTIME_DIR}/systemd/carla-backend.service" "${USER_UNIT_DIR}/carla-backend.service"
if [ -f "${DELIVERY_DIR}/systemd/carla-video.service" ]; then
  cp "${DELIVERY_DIR}/systemd/carla-video.service" "${USER_UNIT_DIR}/carla-video.service"
fi
chmod +x "${PROJECT_DIR}/official_carla_demo/"*.sh
chmod +x "${RUNTIME_DIR}/"*.sh "${RUNTIME_DIR}/wait_for_carla.py"
chmod +x "${DELIVERY_DIR}/"*.sh "${DELIVERY_DIR}/wait_for_carla.py" 2>/dev/null || true
chmod +x "${PROJECT_DIR}/bin/mediamtx" 2>/dev/null || true
find "${PROJECT_DIR}" -type d -name '__pycache__' -prune -exec rm -rf '{}' + 2>/dev/null || true
find "${PROJECT_DIR}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete 2>/dev/null || true
rm -f "${PROJECT_DIR}/logs/official_udp_status.json" \
      "${PROJECT_DIR}/logs/official_gateway_status.json" \
      "${PROJECT_DIR}/logs/official_view_command.json" \
      "${PROJECT_DIR}/logs/mirror_stream_status.json" 2>/dev/null || true

# Keep ASCII aliases for files that may be mojibake after Windows/WeChat zip
# transfer and Linux unzip. The runtime uses these aliases, not Chinese names.
if [ -d "${PROJECT_DIR}/can" ] && [ ! -f "${PROJECT_DIR}/can/NJ0515.dbc" ]; then
  find "${PROJECT_DIR}/can" -maxdepth 1 -type f -name '*NJ0515.dbc' -exec cp -f '{}' "${PROJECT_DIR}/can/NJ0515.dbc" \; -quit
fi
if [ ! -f "${PROJECT_DIR}/NJ0423.dbc" ]; then
  find "${PROJECT_DIR}" -maxdepth 1 -type f -name '*NJ0423.dbc' -exec cp -f '{}' "${PROJECT_DIR}/NJ0423.dbc" \; -quit
fi

mkdir -p "${ENGINE_DROPIN_DIR}" "${BACKEND_DROPIN_DIR}"
cat > "${ENGINE_DROPIN_DIR}/official-demo.conf" <<EOF
[Unit]
StartLimitIntervalSec=300
StartLimitBurst=3

[Service]
Environment=CARLA_RENDER_OFFSCREEN=0
Environment=CARLA_FULLSCREEN=0
Environment=CARLA_RESOLUTION=1920x1080
Environment=CARLA_QUALITY_LEVEL=Low
Environment="CARLA_EXTRA_ARGS=-windowed -vulkanpresentmode=2"
Restart=on-failure
RestartSec=20
EOF

cat > "${BACKEND_DROPIN_DIR}/official-demo.conf" <<EOF
[Unit]
StartLimitIntervalSec=300
StartLimitBurst=3

[Service]
Environment=CARLA_BACKEND_SCRIPT=official_carla_demo/official_demo_gateway.py
Environment=CENTER_DISPLAY_AUTO_START=0
Environment=OFFICIAL_DEMO_RES=1920x1080
Environment=OFFICIAL_DEMO_FULLSCREEN=1
Environment=OFFICIAL_COMMAND_COOLDOWN_SEC=25
Environment=OFFICIAL_TERMINATE_TIMEOUT_SEC=20
Environment=OFFICIAL_RELAUNCH_GAP_SEC=4.0
Environment=OFFICIAL_CARLA_CLIENT_TIMEOUT=60
Environment=OFFICIAL_ENABLE_SENSORS=1
Environment=OFFICIAL_SENSOR_UDP_PORTS=5010
Environment=OFFICIAL_SENSOR_SUMMARY_HZ=5
Environment=OFFICIAL_SENSOR_UDP_HZ=5
Environment=OFFICIAL_SENSOR_LIDAR_MAX_PPS=300000
Environment=OFFICIAL_SENSOR_LIDAR_MAX_HZ=5
Environment=OFFICIAL_AIGO_DISABLE_SENSORS=1
Environment=OFFICIAL_AIGO_DISABLE_SIDE_MIRRORS=0
Environment=OFFICIAL_AI_BASELINE_DISABLE_SENSORS=1
Environment=OFFICIAL_AI_BASELINE_DISABLE_SIDE_MIRRORS=0
Environment=OFFICIAL_SIDE_MIRRORS_ALWAYS_ON=1
Environment=OFFICIAL_DISABLE_SIDE_MIRRORS=1
Environment=OFFICIAL_SIDE_RTSP_HOST=192.168.110.100
Environment=OFFICIAL_SIDE_RTSP_PORT=8554
Environment=OFFICIAL_REAR_LEFT_STREAM_URL=rtsp://192.168.110.100:8554/carla_rear_left
Environment=OFFICIAL_REAR_RIGHT_STREAM_URL=rtsp://192.168.110.100:8554/carla_rear_right
Environment=OFFICIAL_BIRDVIEW_STREAM_URL=rtsp://192.168.110.100:8554/carla_birdview
Environment=OFFICIAL_AI_USE_MAP_SPAWN=0
Environment=OFFICIAL_AI_IGNORE_LIGHTS_PERCENT=0
Environment=OFFICIAL_AI_IGNORE_SIGNS_PERCENT=0
Environment=OFFICIAL_AI_FALLBACK_ENABLED=0
Environment=OFFICIAL_TRAFFIC_MANAGER_PORT=8000
Environment=OFFICIAL_FIXED_DELTA_SECONDS=0.02
Environment=OFFICIAL_MAX_SUBSTEP_DELTA_TIME=0.005
Environment=OFFICIAL_MAX_SUBSTEPS=16
Environment=OFFICIAL_CAMERA_SENSOR_TICK=0.02
Environment=OFFICIAL_DISPLAY_LOOP_HZ=60
Environment=OFFICIAL_ROUTE_TM_SPEED_DIFFERENCE_PERCENT=0
Environment=OFFICIAL_ROUTE_ARRIVAL_DISTANCE_M=2.0
Environment=CARLA_EXAMPLES_DIR=%h/Workspace/carla_hil_project/PythonAPI/examples
Environment=CARLA_PYTHONAPI=%h/Workspace/carla_hil_project/PythonAPI/carla
Environment=SDL_VIDEODRIVER=x11
ExecStart=
ExecStart=/usr/bin/env bash %h/Carla_Project/official_carla_demo/start_backend_service.sh
Restart=on-failure
RestartSec=12
EOF

systemctl --user daemon-reload
systemctl --user disable --now official-carla-demo.service carla-center-display.service delivery-watchdog.timer 2>/dev/null || true
rm -f "${USER_UNIT_DIR}/official-carla-demo.service"
if [ -f "${USER_UNIT_DIR}/carla-video.service" ]; then
  systemctl --user enable carla-engine.service carla-backend.service carla-video.service
else
  systemctl --user enable carla-engine.service carla-backend.service
fi

echo "Installed stable official CARLA demo autostart."
echo
echo "Start now:"
echo "  systemctl --user restart carla-engine.service"
echo "  systemctl --user restart carla-backend.service"
echo "  systemctl --user restart carla-video.service"
echo
echo "Check status:"
echo "  systemctl --user status carla-engine.service --no-pager -l"
echo "  systemctl --user status carla-backend.service --no-pager -l"
echo "  systemctl --user status carla-video.service --no-pager -l"
echo "  curl -sS http://127.0.0.1:8765/health | python3 -m json.tool"
echo
echo "For boot demo, keep Ubuntu auto-login enabled for user: ${USER}"
