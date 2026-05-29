#!/usr/bin/env bash
set -euo pipefail

echo "== Kernel =="
uname -r

echo
echo "== IP =="
ip -br addr

echo
echo "== User services =="
systemctl --user --no-pager --full status carla-engine.service carla-backend.service carla-video.service delivery-watchdog.timer || true

echo
echo "== Health =="
curl -fsS "http://192.168.110.100:8765/health" || true
echo

echo
echo "== Mirror stream =="
bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/check_mirror_stream.sh" || true
