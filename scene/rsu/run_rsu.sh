#!/usr/bin/env bash
set -eo pipefail
RSU_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$HOME/carla-venv/bin/python" "$RSU_DIR/session.py" "$@"
