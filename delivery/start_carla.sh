#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

if [ ! -d "${CARLA_DIR}" ]; then
  log "ERROR: CARLA_DIR does not exist: ${CARLA_DIR}"
  exit 1
fi

cd "${CARLA_DIR}"

log "Waiting for graphical display"
if ! wait_for_display 120; then
  log "ERROR: no graphical X11 display is ready"
  exit 1
fi
resolve_xauthority

mapfile -t display_args < <(build_carla_display_args)
render_args=()
extra_args=()
if [ "${CARLA_RENDER_OFFSCREEN:-0}" = "1" ]; then
  render_args+=("-RenderOffScreen")
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
