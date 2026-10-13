#!/usr/bin/env bash
set -eo pipefail

LINCOLN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$LINCOLN_DIR/../.." && pwd)"
cd -- "$PROJECT_DIR"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages"
exec "$HOME/carla-venv/bin/python" "$LINCOLN_DIR/ego_vehicle.py" "$@"
