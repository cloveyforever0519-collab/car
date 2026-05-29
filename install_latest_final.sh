#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${CARLA_PROJECT_ROOT:-${HOME}/Carla_Project}"
USER_UNIT_DIR="${HOME}/.config/systemd/user"

echo "=== Final CARLA project install ==="
echo "PROJECT_DIR=${PROJECT_DIR}"

if [ ! -d "${PROJECT_DIR}" ]; then
  echo "ERROR: missing ${PROJECT_DIR}"
  exit 2
fi

cd "${PROJECT_DIR}"

echo "=== Clean transient files ==="
find "${PROJECT_DIR}" -type d -name '__pycache__' -prune -exec rm -rf '{}' + 2>/dev/null || true
find "${PROJECT_DIR}" -type f \( -name '*.pyc' -o -name '*.pyo' -o -name '.DS_Store' -o -name 'Thumbs.db' \) -delete 2>/dev/null || true

echo "=== Apply executable permissions ==="
chmod +x "${PROJECT_DIR}/install_latest_final.sh" || true
chmod +x "${PROJECT_DIR}/delivery/"*.sh "${PROJECT_DIR}/delivery/wait_for_carla.py" 2>/dev/null || true
chmod +x "${PROJECT_DIR}/official_carla_demo/"*.sh 2>/dev/null || true
chmod +x "${PROJECT_DIR}/official_carla_demo/runtime/"*.sh "${PROJECT_DIR}/official_carla_demo/runtime/wait_for_carla.py" 2>/dev/null || true
chmod +x "${PROJECT_DIR}/bin/mediamtx" 2>/dev/null || true

echo "=== Install official runtime services and drop-ins ==="
bash "${PROJECT_DIR}/official_carla_demo/install_autostart.sh"

echo "=== Ensure video service is installed ==="
mkdir -p "${USER_UNIT_DIR}"
if [ -f "${PROJECT_DIR}/delivery/systemd/carla-video.service" ]; then
  cp -f "${PROJECT_DIR}/delivery/systemd/carla-video.service" "${USER_UNIT_DIR}/carla-video.service"
fi

echo "=== Ensure video stream config ==="
rm -rf "${USER_UNIT_DIR}/carla-video.service.d"
mkdir -p "${USER_UNIT_DIR}/carla-video.service.d"
cat > "${USER_UNIT_DIR}/carla-video.service.d/official-video-final.conf" <<'EOF'
[Service]
Environment=MIRROR_WIDTH=1280
Environment=MIRROR_HEIGHT=720
Environment=MIRROR_FPS=20
Environment=MIRROR_BITRATE=5000k
Environment=MIRROR_BUFSIZE=5000k
Environment=MIRROR_QUEUE_SIZE=1
Environment=MIRROR_STALE_FRAME_SEC=0.35
Environment=MIRROR_ENABLE_REAR_STREAMS=1
Environment=MIRROR_ENABLE_BIRDVIEW_STREAM=1
Environment=MIRROR_BIRDVIEW_PATH=carla_birdview
Environment=MIRROR_BIRDVIEW_FOV=96.0
Environment=MIRROR_RTSP_PUBLIC_HOST=192.168.110.100
Environment=MIRROR_RTSP_HOST=127.0.0.1
Environment=MIRROR_RTSP_PORT=8554
Environment=MIRROR_MEDIAMTX_BIN=%h/Carla_Project/bin/mediamtx
Environment=MIRROR_MEDIAMTX_CONFIG=%h/Carla_Project/delivery/mediamtx.yml
Restart=on-failure
RestartSec=10
EOF

systemctl --user daemon-reload
systemctl --user reset-failed carla-engine.service carla-backend.service carla-video.service 2>/dev/null || true
systemctl --user enable carla-engine.service carla-backend.service carla-video.service

echo "=== Installed final project. Start with: ==="
echo "systemctl --user restart carla-engine.service"
echo "sleep 8"
echo "systemctl --user restart carla-backend.service"
echo "systemctl --user restart carla-video.service"
