#!/usr/bin/env bash
set -euo pipefail
TRAFFIC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$TRAFFIC_DIR/../.." && pwd)"
exec "$HOME/carla-venv/bin/python" -u "$PROJECT_DIR/cobridge/sumo/npc_client.py" "$@"
