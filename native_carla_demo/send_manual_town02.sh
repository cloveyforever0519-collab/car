#!/usr/bin/env bash
set -euo pipefail

curl -sS -X POST http://127.0.0.1:8765/command \
  -H "Content-Type: application/json" \
  -d '{"sendstate":"START","scene":"Town02","sky":"Sunny","sunshinetime":"Noon","drive_mode":"Manual","loadingtransportation":"1","vehiclemodel":"Lincoln MKZ","camera_view":"follow"}' \
  | python3 -m json.tool
