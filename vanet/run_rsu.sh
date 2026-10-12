#!/usr/bin/env bash
set -eo pipefail
VANET_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$HOME/carla-venv/bin/python" "$VANET_DIR/runtime/rsu_session.py" "$@"
