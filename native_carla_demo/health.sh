#!/usr/bin/env bash
set -euo pipefail

curl -sS http://127.0.0.1:8765/health | python3 -m json.tool
