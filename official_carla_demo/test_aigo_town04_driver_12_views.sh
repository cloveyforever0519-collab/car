#!/usr/bin/env bash
set -euo pipefail

IP="${1:-127.0.0.1}"
PORT="${2:-8765}"
RUN_SECONDS="${3:-45}"
BASE_URL="http://${IP}:${PORT}"

vehicles=(
  "Dodge Charger"
  "Lincoln MKZ"
  "Tesla Model 3"
  "Audi e-tron"
  "Jeep Wrangler"
  "Tesla Cybertruck"
  "Fuso Rosa"
  "Mercedes Sprinter"
  "Volkswagen T2"
  "Carlacola Truck"
  "European HGV"
  "Firetruck"
)

case_no=0
total="${#vehicles[@]}"

echo "===== AIGO Town04 12-car driver-view camera test ====="
echo "Target: ${BASE_URL}"
echo "Each car: ${RUN_SECONDS} sec"

for vehicle in "${vehicles[@]}"; do
  case_no=$((case_no + 1))
  echo
  echo "===== TEST ${case_no} / ${total} ====="
  echo "vehiclemodel=${vehicle}"
  echo "camera_view=driver"

  curl -sS -X POST "${BASE_URL}/command" \
    -H "Content-Type: application/json" \
    -d "{\"sendstate\":\"START\",\"scene\":\"Town04\",\"sky\":\"Sunny\",\"sunshinetime\":\"Noon\",\"drive_mode\":\"AIGO\",\"loadingtransportation\":\"1\",\"loadingsensor\":\"1\",\"vehiclemodel\":\"${vehicle}\",\"camera_view\":\"driver\"}" \
    | python3 -m json.tool

  elapsed=0
  while [ "${elapsed}" -lt "${RUN_SECONDS}" ]; do
    sleep 5
    elapsed=$((elapsed + 5))
    python3 - "${BASE_URL}" "${case_no}" "${total}" "${elapsed}" <<'PY'
import json
import sys
import urllib.request

base_url, case_no, total, elapsed = sys.argv[1:5]
try:
    with urllib.request.urlopen(base_url + "/health", timeout=5) as resp:
        h = json.load(resp)
    d = h.get("diagnostics") or {}
    print(f"[{case_no}/{total}][{elapsed} sec] mode={h.get('mode')} running={h.get('running')} vehicle_alive={h.get('vehicle_alive')} speed={d.get('speed_kmh')} view={h.get('camera_view')}")
except Exception as exc:
    print(f"health check failed: {exc}")
PY
  done
done

echo
echo "===== 12-car driver-view test done ====="
