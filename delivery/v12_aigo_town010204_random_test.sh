#!/usr/bin/env bash
set -u

API="http://127.0.0.1:8765"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/v12_aigo_town010204_random_test_${TS}"
REPORT="$HOME/v12_aigo_town010204_random_test_${TS}.tar.gz"

mkdir -p "$OUT"/{commands,health,mirror,rtsp,journal}

SCENES=("Town01" "Town02" "Town04")
VEHICLES=(
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
WEATHERS=("Sunny" "Cloudy")
TIMES=("Noon" "Morning" "Sunset")
LOCAL_STREAMS=(
  "rtsp://127.0.0.1:8554/carla_rear_left"
  "rtsp://127.0.0.1:8554/carla_rear_right"
  "rtsp://127.0.0.1:8554/carla_birdview"
)

RUNS="${1:-12}"
RUN_SECONDS="${2:-60}"

log() {
  echo "[$(date '+%F %T')] $*" | tee -a "$OUT/summary.log"
}

post_command() {
  local idx="$1" scene="$2" vehicle="$3" traffic="$4" sensors="$5" weather="$6" daytime="$7"
  PAYLOAD="$(SCENE="$scene" VEHICLE="$vehicle" TRAFFIC="$traffic" SENSORS="$sensors" WEATHER="$weather" DAYTIME="$daytime" python3 - <<'PY'
import json, os
print(json.dumps({
  "sendstate": "START",
  "scene": os.environ["SCENE"],
  "sky": os.environ["WEATHER"],
  "sunshinetime": os.environ["DAYTIME"],
  "drive_mode": "AIGO",
  "vehiclemodel": os.environ["VEHICLE"],
  "camera_view": "driver",
  "loadingtransportation": os.environ["TRAFFIC"],
  "loadingsensor": os.environ["SENSORS"],
}, ensure_ascii=False))
PY
)"
  echo "$PAYLOAD" > "$OUT/commands/${idx}_payload.json"
  curl -sS -X POST "$API/command" -H "Content-Type: application/json" -d "$PAYLOAD" \
    | python3 -m json.tool > "$OUT/commands/${idx}_response.json" 2>&1
}

capture_health() {
  local idx="$1"
  curl -sS "$API/health" | python3 -m json.tool > "$OUT/health/${idx}.json" 2>&1
  cp -f "$HOME/Carla_Project/logs/mirror_stream_status.json" "$OUT/mirror/${idx}.json" 2>/dev/null || true
}

pull_stream() {
  local idx="$1" name="$2" url="$3"
  local file="$OUT/rtsp/${idx}_${name}.log"
  if command -v ffprobe >/dev/null 2>&1; then
    timeout 10 ffprobe -v error -rtsp_transport tcp \
      -select_streams v:0 \
      -show_entries stream=codec_name,width,height,r_frame_rate \
      -of json "$url" > "$file" 2>&1
  elif command -v ffmpeg >/dev/null 2>&1; then
    timeout 10 ffmpeg -v error -rtsp_transport tcp -i "$url" -t 3 -f null - > "$file" 2>&1
  else
    echo "ffprobe/ffmpeg not found" > "$file"
  fi
}

summarize_json() {
  python3 - "$OUT" "$RUNS" <<'PY'
import json, pathlib, sys
out = pathlib.Path(sys.argv[1])
runs = int(sys.argv[2])
rows = []
for i in range(1, runs + 1):
    payload_path = out / "commands" / f"{i}_payload.json"
    health_path = out / "health" / f"{i}.json"
    mirror_path = out / "mirror" / f"{i}.json"
    payload = {}
    health = {}
    mirror = {}
    try:
        payload = json.loads(payload_path.read_text(encoding="utf-8"))
    except Exception:
        pass
    try:
        health = json.loads(health_path.read_text(encoding="utf-8"))
    except Exception:
        pass
    try:
        mirror = json.loads(mirror_path.read_text(encoding="utf-8"))
    except Exception:
        pass
    sensor = health.get("sensor_summary") or {}
    streams = mirror.get("streams") or {}
    publishing = sum(1 for v in streams.values() if isinstance(v, dict) and v.get("publishing"))
    rows.append({
        "idx": i,
        "scene": payload.get("scene"),
        "vehicle": payload.get("vehiclemodel"),
        "traffic_flag": payload.get("loadingtransportation"),
        "sensor_flag": payload.get("loadingsensor"),
        "running": health.get("running"),
        "vehicle_alive": health.get("vehicle_alive"),
        "world": health.get("world"),
        "actual_vehicle": (health.get("telemetry") or {}).get("Vehiclemodel"),
        "lidar_points": sensor.get("lidar_points"),
        "radar_targets": sensor.get("radar_targets"),
        "publishing_streams": publishing,
        "mirror_error": mirror.get("last_error"),
    })
report = out / "report.md"
with report.open("w", encoding="utf-8") as f:
    f.write("# V12 AIGO Town01/Town02/Town04 Random Test Report\n\n")
    f.write(f"- Runs: {runs}\n")
    f.write("- API: http://127.0.0.1:8765\n")
    f.write("- Local RTSP probe: rtsp://127.0.0.1:8554/carla_*\n")
    f.write("- Frontend flag convention: 0=load, 1=do not load\n\n")
    f.write("| # | Scene | Vehicle | Traffic | Sensors | Running | Alive | World | Actual | Lidar | Radar | RTSP | Mirror Error |\n")
    f.write("|---|---|---|---|---|---|---|---|---|---:|---:|---:|---|\n")
    for r in rows:
        f.write("| {idx} | {scene} | {vehicle} | {traffic_flag} | {sensor_flag} | {running} | {vehicle_alive} | {world} | {actual_vehicle} | {lidar_points} | {radar_targets} | {publishing_streams} | {mirror_error} |\n".format(**{k: ("" if v is None else v) for k, v in r.items()}))
print(report)
PY
}

log "V12 AIGO random test started: runs=$RUNS, run_seconds=$RUN_SECONDS"
for idx in $(seq 1 "$RUNS"); do
  scene="${SCENES[$((RANDOM % ${#SCENES[@]}))]}"
  vehicle="${VEHICLES[$((RANDOM % ${#VEHICLES[@]}))]}"
  traffic="$((RANDOM % 2))"
  sensors="$((RANDOM % 2))"
  weather="${WEATHERS[$((RANDOM % ${#WEATHERS[@]}))]}"
  daytime="${TIMES[$((RANDOM % ${#TIMES[@]}))]}"
  log "run $idx/$RUNS scene=$scene vehicle=$vehicle traffic=$traffic sensors=$sensors weather=$weather time=$daytime"
  post_command "$idx" "$scene" "$vehicle" "$traffic" "$sensors" "$weather" "$daytime"
  sleep "$RUN_SECONDS"
  capture_health "$idx"
  pull_stream "$idx" "rear_left" "${LOCAL_STREAMS[0]}"
  pull_stream "$idx" "rear_right" "${LOCAL_STREAMS[1]}"
  pull_stream "$idx" "birdview" "${LOCAL_STREAMS[2]}"
done

journalctl --user -u carla-engine.service --since "2 hours ago" --no-pager > "$OUT/journal/engine.log" 2>&1 || true
journalctl --user -u carla-backend.service --since "2 hours ago" --no-pager > "$OUT/journal/backend.log" 2>&1 || true
journalctl --user -u carla-video.service --since "2 hours ago" --no-pager > "$OUT/journal/video.log" 2>&1 || true
ps -ef | grep -E 'CarlaUE4|official_udp|official_demo_gateway|carla_mirror_stream|ffmpeg|mediamtx' | grep -v grep > "$OUT/processes.txt" 2>&1 || true
ss -lntp | grep -E ':2000|:8765|:8554' > "$OUT/ports.txt" 2>&1 || true

summarize_json
tar -czf "$REPORT" -C "$HOME" "$(basename "$OUT")"
log "report: $REPORT"
