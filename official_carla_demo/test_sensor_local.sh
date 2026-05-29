#!/usr/bin/env bash
set -euo pipefail

cd "${CARLA_PROJECT_ROOT:-${HOME}/Carla_Project}"

echo "===== launch AI sensor demo ====="
curl -sS -X POST http://127.0.0.1:8765/command \
  -H "Content-Type: application/json" \
  -d '{"sendstate":"START","scene":"Town04","sky":"Sunny","sunshinetime":"Noon","drive_mode":"AI","loadingtransportation":"0","loadingsensor":"0","vehiclemodel":"Tesla Model 3","camera_view":"follow"}' \
  | python3 -m json.tool

echo "===== wait for vehicle and sensors ====="
sleep 8

echo "===== health ====="
curl -sS http://127.0.0.1:8765/health | python3 -m json.tool

echo "===== sensors ====="
curl -sS http://127.0.0.1:8765/sensors | python3 -m json.tool

echo "===== side mirrors ====="
curl -sS http://127.0.0.1:8771/health | python3 -m json.tool || true

echo
echo "Sensor UDP listener:"
echo "  python3 official_carla_demo/listen_sensor_udp.py"
