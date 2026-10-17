#!/usr/bin/env bash
set -euo pipefail

# Docker / CARLA 可调参数
DOCKER_NAME="${1:-carla}"
CARLA_PORT="${2:-2000}"
DOCKER_IMAGE="${CARLA_DOCKER_IMAGE:-carlasim/carla:0.9.16}"
GPU_DEVICES="all"
NETWORK_MODE="host"
DISPLAY_ADDRESS="${DISPLAY:-:0}"
DRIVER_CAPABILITIES="graphics,utility,compute,display"
X11_HOST_DIR="/tmp/.X11-unix"
X11_CONTAINER_DIR="/tmp/.X11-unix"
X11_ACCESS="+si:localuser:root"
SHARED_MEMORY="4g"
CARLA_LAUNCHER="CarlaUE4.sh"
RESOLUTION_X=1280
RESOLUTION_Y=720
CARLA_OPTIONS=(-nosound -windowed)

STOP_TIMEOUT=5
CONTAINER_ID=""
ATTACH_PID=""

cleanup() {
    if [[ -n "$CONTAINER_ID" ]]; then
        sudo docker stop --time "$STOP_TIMEOUT" "$CONTAINER_ID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

xhost "$X11_ACCESS"
CONTAINER_ID=$(sudo docker create --rm \
    --name "$DOCKER_NAME" --gpus "$GPU_DEVICES" --network "$NETWORK_MODE" \
    -e DISPLAY="$DISPLAY_ADDRESS" -e NVIDIA_DRIVER_CAPABILITIES="$DRIVER_CAPABILITIES" \
    -v "$X11_HOST_DIR:$X11_CONTAINER_DIR:rw" --shm-size="$SHARED_MEMORY" \
    "$DOCKER_IMAGE" bash "$CARLA_LAUNCHER" "${CARLA_OPTIONS[@]}" \
    -ResX="$RESOLUTION_X" -ResY="$RESOLUTION_Y" -carla-rpc-port="$CARLA_PORT")
echo "[CARLA] 前台运行：$DOCKER_NAME，RPC 端口：$CARLA_PORT；Ctrl+C 仅退出 CARLA。"
# wait 是 shell 内建命令，Ctrl+C 时可以立即执行退出 trap，
# 不必等待 Docker 信号转发是否被 Unreal 处理。
sudo docker start --attach "$CONTAINER_ID" &
ATTACH_PID=$!
wait "$ATTACH_PID"
