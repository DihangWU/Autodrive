#!/usr/bin/env bash
set -euo pipefail

# 路径与可调参数
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PYTHON_BIN="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
ENTRY_FILE="$PROJECT_DIR/foundation/carla/map_convert/carla2sumo.py"

exec "$PYTHON_BIN" -u "$ENTRY_FILE" "$@"
