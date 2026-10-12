#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$PROJECT_DIR"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages"
PYTHON="$HOME/carla-venv/bin/python"
HOST_ARG=127.0.0.1
PORT_ARG=2000
NPC_COUNT=0
START_SUMO=true
STOP_SUMO=false
POSITION_MODE=
EGO_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --pose)
            if [[ $# -lt 5 ]]; then
                echo "--pose requires four numbers: X Y Z YAW (meters, degrees)." >&2
                exit 2
            fi
            if [[ -n "$POSITION_MODE" ]]; then
                echo "Choose only one of --pose and --spawn-index." >&2
                exit 2
            fi
            if ! "$PYTHON" -c 'import math, sys; sys.exit(0 if all(math.isfinite(float(v)) for v in sys.argv[1:]) else 2)' "${@:2:4}" 2>/dev/null; then
                echo "--pose requires four finite numbers: X Y Z YAW." >&2
                exit 2
            fi
            POSITION_MODE=pose
            EGO_ARGS+=("${@:1:5}")
            shift 5 ;;
        --spawn-index)
            if [[ $# -lt 2 || ! "$2" =~ ^[0-9]+$ || -n "$POSITION_MODE" ]]; then
                echo "--spawn-index requires one non-negative integer and cannot be combined with --pose." >&2
                exit 2
            fi
            POSITION_MODE=spawn-index
            EGO_ARGS+=("$1" "$2")
            shift 2 ;;
        --no-sumo) START_SUMO=false; shift ;;
        --stop-sumo) STOP_SUMO=true; shift ;;
        --sumo-npcs) NPC_COUNT="$2"; shift 2 ;;
        --host) HOST_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --port) PORT_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --help|-h)
            echo "Usage: bash run_lincoln.sh [--spawn-index N | --pose X Y Z YAW]"
            echo "       No position arguments: randomly choose a free spawn point."
            echo "       --pose: CARLA world X/Y/Z in meters, yaw in degrees."
            echo "       [--sensors all|lidar] [--sumo-npcs N] [--no-sumo] [--no-rviz]"
            echo "       --stop-sumo stops only the bridge started by this launcher."
            "$PYTHON" "lincoln/ego_vehicle.py" --help
            exit 0 ;;
        *) EGO_ARGS+=("$1"); shift ;;
    esac
done
"$PYTHON" -c 'import carla, rclpy, tf2_ros, numpy, tkinter, yaml'
if $STOP_SUMO; then
    exec "$PYTHON" "lincoln/ensure_sumo.py" --host "$HOST_ARG" --port "$PORT_ARG" --stop
fi
if $START_SUMO; then
    "$PYTHON" "lincoln/ensure_sumo.py" --host "$HOST_ARG" --port "$PORT_ARG" --npcs "$NPC_COUNT"
fi
exec "$PYTHON" "lincoln/ego_vehicle.py" "${EGO_ARGS[@]}"
