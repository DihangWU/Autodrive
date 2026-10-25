#!/usr/bin/env bash
set -euo pipefail

LINCOLN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENTRY_FILE="$LINCOLN_DIR/add_lincoln.py"
PYTHON_BIN="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
ROS_SETUP="${AUTODRIVE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
SYSTEM_PYTHON_PACKAGES="/usr/lib/python3/dist-packages"
DDS_PROFILE_FILE="$LINCOLN_DIR/config/fastdds.xml"

# --help 无需加载 ROS；启动车辆时才准备传感器与 RViz 依赖。
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" ]]; then
    set +u
    source "$ROS_SETUP"
    set -u
    export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$SYSTEM_PYTHON_PACKAGES"
    # 在 rclpy.init 前加载；RViz 子进程继承，不覆盖用户自己的 DDS 配置。
    if [[ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" == rmw_fastrtps* &&
          -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
        export FASTDDS_DEFAULT_PROFILES_FILE="$DDS_PROFILE_FILE"
        export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE_FILE"
    fi
fi
exec "$PYTHON_BIN" -u "$ENTRY_FILE" "$@"
