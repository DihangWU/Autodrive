#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages"
PYTHON="$HOME/carla-venv/bin/python"
HOST_ARG=127.0.0.1
PORT_ARG=2000
NPC_COUNT=0
START_SUMO=true
STOP_SUMO=false
EGO_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-sumo) START_SUMO=false; shift ;;
        --stop-sumo) STOP_SUMO=true; shift ;;
        --sumo-npcs) NPC_COUNT="$2"; shift 2 ;;
        --host) HOST_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --port) PORT_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --help|-h)
            echo "Usage: bash run_lincoln.sh [--spawn-index N | --pose X Y Z YAW]"
            echo "       [--sensors all|lidar] [--sumo-npcs N] [--no-sumo] [--no-rviz]"
            echo "       --stop-sumo stops only the bridge started by this launcher."
            "$PYTHON" "$PROJECT_DIR/lincoln/ego_vehicle.py" --help
            exit 0 ;;
        *) EGO_ARGS+=("$1"); shift ;;
    esac
done
"$PYTHON" -c 'import carla, rclpy, tf2_ros, numpy, tkinter, yaml'
if $STOP_SUMO; then
    exec "$PYTHON" "$PROJECT_DIR/lincoln/ensure_sumo.py" --host "$HOST_ARG" --port "$PORT_ARG" --stop
fi
if $START_SUMO; then
    "$PYTHON" "$PROJECT_DIR/lincoln/ensure_sumo.py" --host "$HOST_ARG" --port "$PORT_ARG" --npcs "$NPC_COUNT"
fi
exec "$PYTHON" "$PROJECT_DIR/lincoln/ego_vehicle.py" "${EGO_ARGS[@]}"
