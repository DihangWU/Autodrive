1. 查看当前 docker 镜像
    ```
    sudo docker image
    ```

2. 启动 Carla
    ```
    xhost +si:localuser:root

    sudo docker run --rm -it \
    --name carla \
    --gpus all \
    --network host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --shm-size=4g \
    carlasim/carla:0.9.16 \
    bash CarlaUE4.sh -nosound -windowed -ResX=1280 -ResY=720
    ```

3. 列出可用自带地图
    ```
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(20); print('\n'.join(c.get_available_maps()))"
    ```

4. 选择地图
    ```
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(120); c.load_world('Town10HD')"
    ```

5. 启动自由视角
    ```
    ~/carla-venv/bin/python ~/Desktop/Carla/carla_free_camera.py
    ```

6. 添加 NPC 车辆
    ```
    ~/carla-venv/bin/python ~/Desktop/Carla/town10_traffic.py
    ```