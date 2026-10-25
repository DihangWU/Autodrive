#!/usr/bin/env bash
set -euo pipefail

NPC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENTRY_FILE="$NPC_DIR/npcvehicules.py"
PYTHON_BIN="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"

exec "$PYTHON_BIN" -u "$ENTRY_FILE" "$@"
