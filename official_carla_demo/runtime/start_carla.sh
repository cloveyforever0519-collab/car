#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

if [ ! -d "${CARLA_DIR}" ]; then
  log "ERROR: CARLA_DIR does not exist: ${CARLA_DIR}"
  exit 1
fi

if [ "${OFFICIAL_KILL_STALE_CARLA:-1}" = "1" ] && command -v pgrep >/dev/null 2>&1; then
  while read -r pid cmdline; do
    [ -n "${pid:-}" ] || continue
    [ "${pid}" != "$$" ] || continue
    case "${cmdline}" in
      *"${CARLA_DIR}"*CarlaUE4*|*CarlaUE4-Linux-Shipping*)
        log "Stopping stale CARLA process before engine start: pid=${pid}"
        kill -TERM "${pid}" 2>/dev/null || true
        for _ in 1 2 3 4 5 6 7 8 9 10; do
          kill -0 "${pid}" 2>/dev/null || break
          sleep 1
        done
        kill -0 "${pid}" 2>/dev/null && kill -KILL "${pid}" 2>/dev/null || true
        ;;
    esac
  done < <(pgrep -af 'CarlaUE4|CarlaUE4-Linux-Shipping' || true)
fi

cd "${CARLA_DIR}"

log "Waiting for graphical display"
if ! wait_for_display 120; then
  log "ERROR: no graphical X11 display is ready"
  exit 1
fi
resolve_xauthority

render_args=()
display_args=()
extra_args=()
if [ "${CARLA_RENDER_OFFSCREEN:-0}" = "1" ]; then
  render_args+=("-RenderOffScreen")
else
  mapfile -t display_args < <(build_carla_display_args)
fi
if [ -n "${CARLA_QUALITY_LEVEL:-}" ]; then
  render_args+=("-quality-level=${CARLA_QUALITY_LEVEL}")
fi
if [ -n "${CARLA_EXTRA_ARGS:-}" ]; then
  # shellcheck disable=SC2206
  extra_args=(${CARLA_EXTRA_ARGS})
fi

log "Starting CARLA from ${CARLA_DIR} on DISPLAY=${DISPLAY}, XAUTHORITY=${XAUTHORITY:-unset}, args=${render_args[*]:-default} ${display_args[*]:-default} ${extra_args[*]:-}"

# Do not use exec here through a pipe; systemd should track the Unreal process
# directly. -nosound avoids audio device failures on unattended machines.
exec "${CARLA_DIR}/CarlaUE4.sh" -nosound "${render_args[@]}" "${display_args[@]}" "${extra_args[@]}"
