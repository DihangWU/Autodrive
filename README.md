1. 查看当前 docker 镜像
    ```shell
    sudo docker image
    ```

2. 启停 Carla
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
    ```shell
    sudo docker ps
    sudo docker rm -f carla
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




### 第三步：加入可鼠标驾驶的林肯与传感器

启动 CARLA Docker、加载 Town04 后，在本目录运行：

```shell
bash run_lincoln.sh
```

命令会寻找空闲的标准出生点，生成一辆启用物理的 `vehicle.lincoln.mkz_2020`，
挂载 31 个传感器，打开鼠标驾驶面板和独立 RViz 窗口。
驾驶面板提供油门、左转、刹车、右转四个按钮，按住执行、松开收油滑行，不自动刹车。
右键油门按钮记录当前速度并开启定速，面板显示目标速度；再次右键、操作油门/W、刹车/S 或切换挡位退出定速。定速时允许转向，松开或失焦后仍保持定速。
定速仅自动调节油门，不自动刹车；下坡等情况下车速可能超过目标，需要手动制动。
油门按钮右侧、右转按钮上方可输入目标速度（km/h），按 Enter 开始加速并保持该速度；加速期间转向仍可使用，手动油门或刹车退出定速。输入低于当前速度的目标时收油减速，不自动制动。
油门左侧有“开启自动驾驶”按钮，点击由 CARLA Traffic Manager 接管本车，再次点击恢复手动。自动驾驶期间本脚本不发送油门/转向/刹车指令；鼠标驾驶按钮、WASD、换挡或提交目标速度会退出自动驾驶并手动接管。默认共享 TM 端口 8000，可用 `--tm-port` 指定。
窗口失去焦点时解除油门和转向输入、收油滑行，不自动制动；面板下方显示当前指令和车辆实际油门、刹车值。
驾驶面板获得焦点时也支持键盘：W 油门、S 刹车、A/D 左右转向（可与 W 同时按），Q 静止时切换前进/倒车。
键盘采用按住执行、松开解除并收油滑行；S 或刹车按钮主动制动。倒车时按 W 给油，鼠标操作仍可使用。
已有本机 `spawn_npc_sumo.py` 联合仿真时复用；没有时自动启动共享 SUMO GUI，
采用 0.05 秒步长、CARLA 管理信号灯，默认不额外生成背景车。
SUMO 自动播放，桥接把这辆 CARLA 控制车映射为二维车辆；它不是三维林肯模型。

可以在其他终端重复运行，加入更多车。各车具有唯一角色名、ROS 话题和 TF 前缀，
每个驾驶面板只控制本次生成的车辆；SUMO 仍只有一个推进者。
同时可看到每辆车的 RViz 窗口，或在一个 RViz 的 Displays 中增加其他车的主题。
RViz 默认使用原来的车后跟随视角，保留 Views 面板，不添加已保存视角预设。顶部 Move Camera 工具可旋转、平移、缩放。
RViz 默认加载 `lincoln/rviz/lincoln.rviz` 保存的布局；启动时生成本车专用临时配置，适配车辆话题/TF 前缀与传感器订阅可靠性，保留模板的窗口排布和显示设置。可用 `--rviz-config 路径` 指定其他配置。
启动时会等待 SUMO 实际推进仿真帧，并确认新车已出现在 CARLA 状态快照中；`--stop-sumo` 等待旧桥接清理退出，避免重启竞争。传感器订阅的可靠性设置自动匹配 Best Effort 发布端，避免保存的 RViz 配置导致收不到数据。
自由查看时在 Views 中选择 Orbit、Target Frame 设为 map；真实 CARLA 相机图像仍显示在摄像头拼图中。
RViz 摄像头拼图默认使用 Demo 的两列四行布局，每格带名称和视场角。四行依次为：standard/tele、rear/wide、b_left/b_right、left/right。
RViz 传感器显示按 rig 分组且默认折叠：组复选框控制整组可视化，展开后可单独开关传感器。单路摄像头 Image 显示默认关闭，勾选后打开独立图像窗口；两列四行拼图有独立开关。
这些开关仅影响 RViz 显示，不停止 CARLA 传感器采集或 ROS 发布。
`ultrasonic_rig` 下进一步按 `front`、`rear`、`left`、`right` 分组，各有 4 颗传感器，可按方向整体开关或展开后单独开关。
如需两列六行并预留两行空位，运行 `bash run_lincoln.sh --camera-rows 6`。
默认仅第一辆车跟随 CARLA 观察视角；手动让多车同时跟随可能争抢视角，
可通过“跟随本车／释放视角”按钮关闭其他窗口的跟随。
驾驶面板的“释放视角”按钮切换到嵌入式自由相机，保持释放时的位置；点击相机操作区域获得焦点后，WASD 移动相机、Q/E 升降、Shift 加速、滚轮调速、左键拖动转向。F 捕获/释放鼠标，捕获后鼠标移动直接转向；Esc 释放鼠标，R 恢复本次释放视角的初始位置。
点击“跟随本车”恢复默认车后 8 米、上方 4 米、俯角 15 度。相机区域获得焦点且处于自由模式时，WASD 不控制车辆；车辆仍可使用面板驾驶按钮或自动驾驶。窗口失焦会释放鼠标并清除相机移动键。
在当前 X11 桌面环境，自由模式下点击 CARLA 渲染窗口会自动捕获鼠标，鼠标移动直接转向，WASD/Q/E/Shift 可移动相机，R 重置。F 切换捕获，Esc 释放；离开 CARLA 窗口或切回跟随会恢复鼠标。滚轮调速仍可在面板相机操作区使用。
传感器转换与 ROS 发布在独立线程运行；跟随视角使用独立线程按仿真帧更新，保持固定跟车距离并平滑朝向。
0.05 秒仿真步长对应约 20 帧/秒，跟随视角仍受实际仿真帧率限制。
RViz 道路数据来自当前 CARLA 地图：按 0.5 米采样的车道几何与实际车道宽度生成道路面和左右边界，使用 CARLA 的标线类型、颜色与宽度，不再把车道中心路径当作标线。
地图 API 不提供 Unreal 场景材质、建筑网格和所有路面贴图；Town04 的 OpenDRIVE 未提供虚线长度/间隔，RViz 虚线以 3 米线段/3 米间隔示意，不保证与纹理逐像素一致。

```shell
# 指定 CARLA 出生点编号；被占用时直接报错
bash run_lincoln.sh --spawn-index 12

# 指定 CARLA 世界坐标（米）和 yaw（度），不要使用 ROS 的翻转后坐标
bash run_lincoln.sh --pose 10 20 0.5 90

# 新建共享 SUMO 时额外加入 100 辆背景车；复用已有桥时不改变其数量
bash run_lincoln.sh --sumo-npcs 100

# 性能对照：只装激光雷达
bash run_lincoln.sh --sensors lidar

# 已由远程/其他程序运行 SUMO 桥，不自动启动本机桥
bash run_lincoln.sh --no-sumo

# 所有车辆退出后，显式停止共享 SUMO（林肯/NPC 命令自动创建的服务）
bash run_lincoln.sh --stop-sumo
```

关闭驾驶面板或在该车终端按 Ctrl+C，会清理该车、传感器和其 RViz，其他车辆继续运行。
共享 SUMO 保留供下一辆车复用；停止共享桥会退出同步模式，关闭前先退出驾驶车辆。

### 林肯与随机 NPC 独立启动

加载 Town04 后，下面两条命令可在不同终端按任意顺序启动：

```shell
bash run_lincoln.sh
```

```shell
~/carla-venv/bin/python \
  ~/Desktop/Carla/carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py \
  -n 100 --sumo-gui --step-length 0.05 --tls-manager carla
```

第一个命令自动启动常驻共享桥接，后续命令复用它；只有桥接推进 CARLA/SUMO 并调用 TraCI。NPC 命令不再独占桥接，可重复执行，每次创建独立批次，车辆编号不会冲突。数据文件使用脚本自身目录定位，NPC 命令不要求当前目录是 Sumo。

退出林肯面板仅删除本车及传感器；NPC 终端按 Ctrl+C 仅删除本次 NPC（包括排队未入场车辆）。关闭一个终端不停止另一批车辆、CARLA 或 SUMO；异常退出的 NPC 批次也会由桥接回收。共享服务保持运行，只有显式 `bash run_lincoln.sh --stop-sumo` 或关闭 SUMO GUI 才停止它。

地图、步长、SUMO GUI 模式及信号灯配置由首个启动者确定，后来启动的命令沿用现有配置；需要改变这些配置时先停止所有车辆命令，再显式重启桥接。`--sumo-npcs` 创建的是桥接常驻批次；需要可独立退出的车流请用上面的 NPC 命令。

NPC 的 `--standalone` 可保留旧的单脚本桥接模式，供外部/多 TraCI 客户端场景使用，不用于这里的独立启动流程。
不允许车辆程序自行 `world.tick()`，不重载地图，也不重新启动 Docker。

运行依赖：本机 ROS2 Jazzy/RViz2、`~/carla-venv` 的 CARLA 0.9.16 与 NumPy、
系统 Tkinter/PyYAML，以及 `SUMO_HOME`（默认 `~/sumo-1.22.0`）。
运行时从 `/opt/ros/jazzy/setup.bash` 加载 ROS2，不依赖 CarlaDemo 的构建结果。

传感器布局提取自 CarlaDemo：1 个 10 Hz 激光雷达、8 个 640×360 相机、
6 个 20 Hz 雷达、16 个 10 Hz 超声波模拟探头。
相机在本项目 20 Hz 世界步长下采用 10 Hz，而非 Demo 的 15 Hz；
超声波沿用 `sensor.other.radar` 的近距离模拟，不是独立超声波声学模型。
RViz 显示原林肯网格、传感器安装点、当前地图车道中心线、各传感器实际点云及八路相机拼图。
八路图像按配置顺序排列，拼图用于观察，不作为严格同步的多相机算法输入。
资源来源说明见 [lincoln/assets/README.md](lincoln/assets/README.md)。

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
