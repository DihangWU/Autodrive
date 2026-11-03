#!/usr/bin/env bash
set -euo pipefail
VANET_SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VANET_PROJECT_DIRECTORY="$(cd -- "$VANET_SCRIPT_DIRECTORY/.." && pwd)"
export PYTHONPATH="$VANET_PROJECT_DIRECTORY${PYTHONPATH:+:$PYTHONPATH}"
VANET_PYTHON_EXECUTABLE="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
exec "$VANET_PYTHON_EXECUTABLE" -u -m vanet.experiments.experiment_session_management.launcher "$@"
