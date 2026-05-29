#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_UNIT_DIR="${HOME}/.config/systemd/user"

mkdir -p "${USER_UNIT_DIR}"
cp "${SCRIPT_DIR}/systemd/"*.service "${USER_UNIT_DIR}/"
cp "${SCRIPT_DIR}/systemd/"*.timer "${USER_UNIT_DIR}/"

chmod +x "${SCRIPT_DIR}"/*.sh "${SCRIPT_DIR}/wait_for_carla.py"

systemctl --user daemon-reload
systemctl --user enable carla-engine.service
systemctl --user enable carla-backend.service
systemctl --user enable carla-video.service
systemctl --user enable carla-center-display.service
systemctl --user enable delivery-watchdog.timer

echo "Installed user services."
echo
echo "Recommended for delivery boot:"
echo "  1) Enable Ubuntu auto-login for user: ${USER}"
echo "  2) Start now with:"
echo "       systemctl --user start carla-engine.service"
echo "       systemctl --user start carla-backend.service"
echo "       systemctl --user start carla-video.service"
echo "       systemctl --user start carla-center-display.service"
echo "       systemctl --user start delivery-watchdog.timer"
echo
echo "  CARLA starts fullscreen first; the center RTSP player covers it after the camera stream is ready."
echo
echo "Optional if services must run before manual login:"
echo "       sudo loginctl enable-linger ${USER}"
echo "  Note: CARLA still needs a graphical DISPLAY, so auto-login is preferred."
