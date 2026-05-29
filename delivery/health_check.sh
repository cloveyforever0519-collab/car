#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

API_URL="http://${DELIVERY_IP}:${API_PORT}/health"

if ! command -v curl >/dev/null 2>&1; then
  log "ERROR: curl not found"
  exit 2
fi

response="$(curl -fsS --max-time 5 "${API_URL}")"

python - "$response" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
api_ok = data.get("ok") is True and data.get("api") == "running"
carla_ok = data.get("carla_connected") is True
print(json.dumps({
    "api_ok": api_ok,
    "carla_ok": carla_ok,
    "world": data.get("world"),
    "vehicle_alive": data.get("vehicle_alive"),
    "carla_status": data.get("carla_status"),
}, ensure_ascii=False))
sys.exit(0 if api_ok and carla_ok else 1)
PY

