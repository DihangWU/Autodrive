#!/usr/bin/env bash
set -euo pipefail

SUMO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

exec "$HOME/carla-venv/bin/python" -u "$SUMO_DIR/carla2sumo.py"\
    --host 127.0.0.1\
"$@"
