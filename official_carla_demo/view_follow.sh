#!/usr/bin/env bash
set -euo pipefail

curl -sS -X POST http://127.0.0.1:8765/view \
  -H "Content-Type: application/json" \
  -d '{"camera_view":"follow"}' \
  | python3 -m json.tool
