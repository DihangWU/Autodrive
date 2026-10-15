#!/usr/bin/env bash

# 允许 Docker 容器中的 root 用户访问 X11 显示服务器
xhost +si:localuser:root

# 启动 CARLA 0.9.16 Docker 容器
sudo docker run --rm -it \
    --name carla \
    --gpus all \
    --network host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --shm-size=4g \
    carlasim/carla:0.9.16 \
    bash CarlaUE4.sh -nosound -windowed -ResX=1280 -ResY=720 \
    -carla-rpc-port=2000

# -rm 关闭(窗口)后就删除容器；-it启动了会占用终端窗口，可以用命令行和容器交互，-d 会后台执行释放终端
    # i = interface 交互式，t = tty，分配伪终端，d = detach
# --name 给镜像实例化的容器命名
# --network host 直接使用宿主机网络硬件
# --shm-size=4g 分配 4GB 内存给容器
# carlasim/carla:0.9.16 指定使用哪个镜像
# -ResX=1280 -ResY=720 自定义分辨率
# -carla-rpc-port=2000 显式分配端口，不写默认 2000，rpc = Remote Procedure Call，允许远程访问端口