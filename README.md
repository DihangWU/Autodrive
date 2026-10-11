1. 查看当前 docker 镜像
    ```shell
    sudo docker image
    ```

2. 启动 Carla
    ```shell
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
    ```shell
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(20); print('\n'.join(c.get_available_maps()))"
    ```

4. 选择地图
    ```shell
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(120); c.load_world('Town04')"
    ```

5. 启动自由视角
    ```shell
    ~/carla-venv/bin/python ~/Desktop/Carla/carla_free_camera.py
    ```

6. 添加 NPC 车辆
    ```shell
    ~/carla-venv/bin/python ~/Desktop/Carla/npc_traffic.py
    ```


7. 在连接了 CARLA 的 Python 脚本中，用这一行把所有交通信号灯设为红灯：
    ```shell
    在连接了 CARLA 的 Python 脚本中，用这一行把所有交通信号灯设为红灯：
    ```
    * 如果想一直保持红灯，需要再冻结信号灯，否则它们会继续按周期切换：
        ```shell
        for light in world.get_actors().filter("traffic.traffic_light*"):
        light.set_state(carla.TrafficLightState.Red)
        light.freeze(True)
        ```
    * 恢复正常切换
        ```shell
        for light in world.get_actors().filter("traffic.traffic_light*"):
        light.freeze(False)
        ```




### SUMO Carla 联合仿真
1. 打开 carla 的 docker
2. 切换地图
    ```shell
    ~/carla-venv/bin/python - <<'PY'
    import carla

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(120)

    world = client.load_world("Town04")

    print("当前地图：", world.get_map().name)
    print("车辆数量：", len(world.get_actors().filter("vehicle.*")))
    print("CARLA 已准备好进行 SUMO 联合仿真")
    PY
    ```
3. 启动自由视角
    ```shell
    ~/carla-venv/bin/python ~/Desktop/Carla/carla_free_camera.py
    ```

4. 启动联合仿真
    ```shell
    ~/carla-venv/bin/python \
    ~/Desktop/Carla/carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py \
    -n 100 \
    --sumo-gui \
    --step-length 0.05 \
    --tls-manager carla

    # 150辆 npc 车
    # 打开 SUMO 的图形窗口
    # 多少秒仿真一次
    # 由 CARLA 管理交通信号灯
   ```