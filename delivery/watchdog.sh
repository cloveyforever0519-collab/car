#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

WATCHDOG_API_HOST="${WATCHDOG_API_HOST:-127.0.0.1}"
API_URL="http://${WATCHDOG_API_HOST}:${API_PORT}/health"
FAIL_FILE="${WATCHDOG_STATE_DIR}/health_fail_count"
CARLA_FAIL_FILE="${WATCHDOG_STATE_DIR}/carla_fail_count"
LAST_CARLA_RESTART_FILE="${WATCHDOG_STATE_DIR}/last_carla_restart"
DEPLOYMENT_STATE_FILE="${PROJECT_DIR}/logs/deployment_state.json"
DEPLOYMENT_GRACE_SEC="${WATCHDOG_DEPLOYMENT_GRACE_SEC:-420}"

read_num() {
  local file="$1"
  if [ -f "$file" ]; then
    cat "$file"
  else
    echo 0
  fi
}

write_num() {
  echo "$2" > "$1"
}

delivery_ip_present() {
  ip -4 addr show | grep -q "${DELIVERY_IP}/"
}

restart_backend() {
  log "Restarting carla-backend.service"
  systemctl --user restart carla-backend.service || true
}

restart_carla_stack() {
  local now last
  now="$(date +%s)"
  last="$(read_num "${LAST_CARLA_RESTART_FILE}")"
  if [ $((now - last)) -lt "${WATCHDOG_CARLA_RESTART_COOLDOWN_SEC}" ]; then
    log "CARLA stack restart suppressed by cooldown"
    return
  fi

  log "Restarting CARLA stack: backend then engine"
  systemctl --user stop carla-backend.service || true
  systemctl --user restart carla-engine.service || true
  sleep 10
  systemctl --user restart carla-backend.service || true
  write_num "${LAST_CARLA_RESTART_FILE}" "${now}"
}

deployment_active() {
  python3 - "${DEPLOYMENT_STATE_FILE}" "${DEPLOYMENT_GRACE_SEC}" <<'PY'
import json
import os
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
grace = float(sys.argv[2])
try:
    data = json.loads(path.read_text(encoding="utf-8"))
except Exception:
    print("0")
    raise SystemExit(0)

if not data.get("active"):
    print("0")
    raise SystemExit(0)

updated = float(data.get("updated_ts") or 0.0)
if updated <= 0.0 or time.time() - updated > grace:
    print("0")
    raise SystemExit(0)

pid = int(data.get("pid") or 0)
if pid > 0:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        print("0")
        raise SystemExit(0)
    except PermissionError:
        pass
    except Exception:
        pass

print("1")
PY
}

if ! delivery_ip_present; then
  log "WARNING: delivery IP ${DELIVERY_IP} not found on any interface"
fi

if ! command -v curl >/dev/null 2>&1; then
  log "ERROR: curl not found"
  exit 0
fi

if ! response="$(curl -fsS --max-time 5 "${API_URL}" 2>/dev/null)"; then
  if [ "$(deployment_active)" = "1" ]; then
    log "Backend health request failed during deployment; restart suppressed"
    exit 0
  fi
  fails="$(( $(read_num "${FAIL_FILE}") + 1 ))"
  write_num "${FAIL_FILE}" "${fails}"
  log "Backend health request failed (${fails}/${WATCHDOG_FAILURE_LIMIT}) at ${API_URL}"
  if [ "${fails}" -ge "${WATCHDOG_FAILURE_LIMIT}" ]; then
    write_num "${FAIL_FILE}" 0
    restart_backend
  fi
  exit 0
fi

write_num "${FAIL_FILE}" 0

status="$(python3 - "$response" <<'PY'
import json
import sys

try:
    data = json.loads(sys.argv[1])
except Exception:
    print("bad_json")
    raise SystemExit(0)

if data.get("deployment_active") is True:
    print("deploying")
    raise SystemExit(0)

api_ok = data.get("ok") is True and data.get("api") == "running"
carla_ok = data.get("carla_connected") is True
print("ok" if api_ok and carla_ok else ("api_bad" if not api_ok else "carla_bad"))
PY
)"

case "${status}" in
  ok)
    write_num "${CARLA_FAIL_FILE}" 0
    log "Health OK"
    ;;
  deploying)
    write_num "${FAIL_FILE}" 0
    write_num "${CARLA_FAIL_FILE}" 0
    log "Deployment active; watchdog restart suppressed"
    ;;
  api_bad|bad_json)
    if [ "$(deployment_active)" = "1" ]; then
      log "Backend unhealthy during deployment; restart suppressed"
      exit 0
    fi
    log "Backend returned unhealthy status=${status}; restarting backend"
    restart_backend
    ;;
  carla_bad)
    if [ "$(deployment_active)" = "1" ]; then
      log "CARLA disconnected during deployment; stack restart suppressed"
      exit 0
    fi
    fails="$(( $(read_num "${CARLA_FAIL_FILE}") + 1 ))"
    write_num "${CARLA_FAIL_FILE}" "${fails}"
    log "CARLA disconnected in backend health (${fails}/${WATCHDOG_FAILURE_LIMIT})"
    if [ "${fails}" -ge "${WATCHDOG_FAILURE_LIMIT}" ]; then
      write_num "${CARLA_FAIL_FILE}" 0
      restart_carla_stack
    fi
    ;;
esac
