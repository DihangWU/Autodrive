#!/usr/bin/env bash
set -euo pipefail

RSU_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$HOME/carla-venv/bin/python" -u "$RSU_DIR/rsu.py" "$@"
