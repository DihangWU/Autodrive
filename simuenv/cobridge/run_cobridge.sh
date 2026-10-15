#!/usr/bin/env bash
set -euo pipefail

BRIDGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$HOME/carla-venv/bin/python" -u "$BRIDGE_DIR/cobridge.py" "$@"
