#!/usr/bin/env bash
set -euo pipefail

systemctl --user disable --now delivery-watchdog.timer || true
systemctl --user disable --now carla-video.service || true
systemctl --user disable --now carla-backend.service || true
systemctl --user disable --now carla-engine.service || true
systemctl --user daemon-reload

rm -f "${HOME}/.config/systemd/user/carla-engine.service"
rm -f "${HOME}/.config/systemd/user/carla-backend.service"
rm -f "${HOME}/.config/systemd/user/carla-video.service"
rm -f "${HOME}/.config/systemd/user/delivery-watchdog.service"
rm -f "${HOME}/.config/systemd/user/delivery-watchdog.timer"

systemctl --user daemon-reload
echo "Uninstalled delivery user services."
