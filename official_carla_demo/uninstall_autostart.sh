#!/usr/bin/env bash
set -euo pipefail

USER_UNIT_DIR="${HOME}/.config/systemd/user"

systemctl --user disable --now official-carla-demo.service 2>/dev/null || true
rm -f "${USER_UNIT_DIR}/official-carla-demo.service"
rm -f "${USER_UNIT_DIR}/carla-engine.service.d/official-demo.conf"
rm -f "${USER_UNIT_DIR}/carla-backend.service.d/official-demo.conf"
rmdir "${USER_UNIT_DIR}/carla-engine.service.d" "${USER_UNIT_DIR}/carla-backend.service.d" 2>/dev/null || true
systemctl --user daemon-reload

echo "Removed official CARLA demo autostart."
