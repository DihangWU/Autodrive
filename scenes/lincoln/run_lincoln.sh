#!/usr/bin/env bash
set -eo pipefail
LINCOLN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$LINCOLN_DIR/../.." && pwd)"
cd -- "$PROJECT_DIR"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages"
PYTHON="$HOME/carla-venv/bin/python"
# 固定实验车位只是同一车辆入口的可选参数，不另设 VANET 车辆启动器。
for option in "$@"; do
    if [[ "$option" == "--vanet-slot" || "$option" == --vanet-slot=* ]]; then
        exec "$PYTHON" "$LINCOLN_DIR/spawn_from_config.py" "$@"
    fi
done
HOST_ARG=127.0.0.1
PORT_ARG=2000
NPC_COUNT=0
START_SUMO=true
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
        --sumo-npcs) NPC_COUNT="$2"; shift 2 ;;
        --host) HOST_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --port) PORT_ARG="$2"; EGO_ARGS+=("$1" "$2"); shift 2 ;;
        --help|-h)
            echo "Usage: bash scenes/lincoln/run_lincoln.sh [--spawn-index N | --pose X Y Z YAW]"
            echo "       No position arguments: randomly choose a free spawn point."
            echo "       --pose: CARLA world X/Y/Z in meters, yaw in degrees."
            echo "       [--sensors all|lidar] [--sumo-npcs N] [--no-sumo] [--no-rviz]"
            echo "       --vanet-slot auto|A|B selects a configured experiment slot."
            echo "       Stop the shared bridge with: bash cobridge/run_cobridge.sh --stop"
            "$PYTHON" "$LINCOLN_DIR/ego_vehicle.py" --help
            exit 0 ;;
        *) EGO_ARGS+=("$1"); shift ;;
    esac
done
"$PYTHON" -c 'import carla, rclpy, tf2_ros, numpy, tkinter, yaml'
if $START_SUMO; then
    "$PYTHON" "$PROJECT_DIR/cobridge/common/bridge_manager.py" --host "$HOST_ARG" --port "$PORT_ARG" --npcs "$NPC_COUNT"
fi
exec "$PYTHON" "$LINCOLN_DIR/ego_vehicle.py" "${EGO_ARGS[@]}"
