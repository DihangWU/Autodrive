#!/usr/bin/env bash

# 允许 Docker 容器中的 root 用户访问 X11 显示服务器
xhost +si:localuser:root

# 启动 CARLA 0.9.16 Docker 容器
sudo docker run --rm -d \
    --name carla \
    --gpus all \
    --network host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --shm-size=4g \
    carlasim/carla:0.9.16 \
    bash CarlaUE4.sh -nosound -windowed -ResX=1280 -ResY=720

# -it启动了会占用终端窗口，-d 不会，--rm 关闭(窗口)后就删除容器
# --name 给容器命名
# -ResX=1280 -ResY=720 自定义分辨率