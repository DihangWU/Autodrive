1. docker 操作
    * 查看查看本地已下载的镜像
        ```shell
        sudo docker images
        ```
    * 查看正在运行的 docker
        ```shell
        sudo docker ps
        ```
        * 查看查看所有docker，包括用`sudo docker stop carla`停止的
            ```shell
            sudo docker ps -a
            ```
            * 容器是一个环境，容器内的程序停止了不表示容器停止了
    * 删除指定docker(删除的 docker 镜像，不是本地 image)
        ```shell
        sudo docker rm carla
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

5. 启动自由视角(用来查看空地图，后续真正执行仿真时用其他入口调用，可以不手动执行)
    ```shell
    ~/carla-venv/bin/python carla_free_camera.py
    ```

6. 通过 SUMO 添加 NPC 车辆
    ```shell
    ~/carla-venv/bin/python \
    carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py \
    -n 200 \
    --sumo-gui \
    --step-length 0.05 \
    --tls-manager carla
    ```

7. 添加一辆高配传感器的 lincoln 轿车
    ```shell
    bash run_lincoln.sh
    ```

8. 在连接了 CARLA 的 Python 脚本中，用这一行把所有交通信号灯设为红灯：
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


## 项目说明与阅读入口

以下内容说明当前工作区实际存在的模块、实现和使用方法，并整理本次开发中的问题。上方前 80 行原有内容保留，仅按后续要求将工程内调用路径改为相对路径；其中旧命令或说明如与下文不同，以这里的说明为准。例如加入林肯的正确命令是 `bash run_lincoln.sh`，`docker rm` 删除的是容器，`docker rmi` 才删除镜像。

本项目以 Docker 中的 CARLA 0.9.16 为三维仿真服务，以 SUMO 为随机 NPC 的交通仿真器，在当前 CARLA 地图中加入可独立驾驶的林肯 MKZ 2020，发布 ROS2 传感器与地图数据供 RViz2 观察。借鉴 CarlaDemo 的车辆网格和传感器安装配置，但驾驶、桥接生命周期、ROS 发布及窗口控制由本工程独立实现，不需要运行 Demo 或依赖其 ROS 构建产物。

当前工作区中的 `carla-source/` 是单独的 Git 仓库，实际检出的源码主要是 `Co-Simulation/Sumo/` 及 CARLA 仓库顶层说明、构建文件；不是完整 Unreal 编辑器工程。Docker 中的 CARLA 服务使用镜像内的程序，本地修改 Python 桥接脚本无需重新构建镜像。

### 整体运行关系

```text
CARLA Docker：地图、物理、车辆、传感器、公共 spectator 相机
       ↑↓ CARLA Python API
共享 SUMO 桥接进程：唯一仿真推进者、唯一常规 TraCI 调用者
       ↑↓ TraCI                         ↑ 本地请求/响应文件
SUMO / SUMO GUI                  NPC 客户端：各自独立的车流批次
       ↑↓ 双向车辆映射
多个林肯客户端：各自控制本车、采集传感器、发布 ROS2
       ├─ Tk mouse drive：鼠标 / WASD / 定速 / Traffic Manager
       ├─ ROS2 → 各自 RViz2：地图、车辆、TF、点云、八路图像
       └─ 相机所有权锁 → 同一时间只有一个林肯客户端控制 spectator
```

NPC 由 SUMO 控制，桥接将位置同步到 CARLA；林肯由手动输入或 CARLA Traffic Manager 控制，桥接将位置同步到 SUMO。SUMO 中显示的是二维车辆符号，不是三维林肯模型。每辆林肯有独立角色名、ROS 命名空间和 TF 前缀。

## 环境、启动与退出

### 依赖与本机路径

| 项目 | 当前配置 |
|---|---|
| 项目根目录 | 当前检出的工程目录（目前为 `Autodrive/`），下文内部路径均相对此目录 |
| CARLA 服务 | `carlasim/carla:0.9.16`，默认 `127.0.0.1:2000` |
| Python | `~/carla-venv/bin/python`，与服务匹配的 CARLA Python 包 |
| ROS2 / RViz2 | Jazzy，`/opt/ros/jazzy/setup.bash` |
| SUMO | 默认 `~/sumo-1.22.0`，可用 `SUMO_HOME` 覆盖 |
| Python 图像与配置依赖 | NumPy、OpenCV、PyYAML、Tkinter、lxml，以及 ROS2 消息与 TF 包 |
| 桌面输入 | Tkinter；直接控制 CARLA 窗口的鼠标功能使用 X11 / Xfixes |

`run_lincoln.sh` 加载 ROS2 并将 `/usr/lib/python3/dist-packages` 加入 `PYTHONPATH`，让 venv 能访问系统安装的 ROS2/Tk/YAML 相关模块。SUMO 脚本从 `SUMO_HOME/tools` 导入 TraCI 和 sumolib。嵌套目录的 `requirements.txt` 声明 `lxml==5.4.0`、`traci==1.23.1`；这与本机 SUMO 1.22.0 的 tools 来源不完全一致，不应在未核对版本和导入路径时直接覆盖现有可运行环境。

### 推荐启动顺序

本文所有项目内部命令均在工程根目录执行，路径不依赖工程位于哪个用户的桌面。`run_lincoln.sh` 会自动切换到自身所在的根目录；代码内部通过脚本位置确定工程根目录后解析资源，避免依赖调用者当前目录。外部 Python/ROS/SUMO 安装路径不属于项目内部路径。

1. 按上方命令启动 CARLA Docker，等待服务就绪。
2. 可直接使用 CARLA 当前地图；如需换图，在没有运行车辆/桥接时加载所需地图。下面以 Town04 为例：

   ```bash
   ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(120); c.load_world('Town04')"
   ```

3. 在本工程运行林肯和随机 NPC。下面两条可以按任意顺序、在不同终端执行：

   ```bash
   # 先在终端进入工程根目录
   bash run_lincoln.sh
   ```

   ```bash
   ~/carla-venv/bin/python \
     carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py \
     -n 100 --sumo-gui --step-length 0.05 --tls-manager carla
   ```

第一条启动请求会创建常驻共享桥，后来请求复用它。林肯启动器不绑定 Town04，使用 CARLA 当前加载的地图，不负责启动 Docker 或切换地图。车辆出生点、RViz 道路数据和新建共享桥的 SUMO 路网都来自当前地图；地图仍需具备可用出生点及可转换的 OpenDRIVE 道路。

重复 `bash run_lincoln.sh` 可以加入多辆林肯。重复 NPC 命令可以加入多个独立批次。`-n` 是请求数量，SUMO 会根据道路空间逐步入场，不保证启动瞬间所有车辆已经进入地图。

共享桥的步长、地图、GUI 模式和信号灯管理方式由首次启动决定。后来的命令不会强行更改运行中的配置。修改这些设置时，先退出车辆客户端，再停止共享桥、按新配置启动；桥接期间不要调用 `load_world()`。

### 林肯启动参数

| 参数 | 功能及边界 |
|---|---|
| `--spawn-index 12` | 使用当前地图标准出生点编号，被占用则失败 |
| `--pose X Y Z YAW` | 使用 CARLA 世界坐标，位置单位米，yaw 单位度；不使用 ROS 翻转后的 y |
| `--sensors all` / `--sensors lidar` | 默认 31 个传感器；lidar 模式用于轻量运行或性能对照 |
| `--sumo-npcs 100` | 新建桥时创建常驻背景批次；复用旧桥时不追加 |
| `--no-sumo` | 不自动创建桥，用于已有外部推进者；本车不会自行 tick |
| `--no-rviz` | 不打开 RViz，仍创建传感器并发布 ROS 数据 |
| `--rviz-config 路径` | 替换默认 RViz 模板；相对路径按工程根目录解析，也支持绝对路径 |
| `--camera-rows 6` | 两列六行，实际仍八路相机，多出的四格为空 |
| `--host` / `--port` | CARLA 服务地址；自动创建桥仅支持本机 |
| `--tm-port 8000` | 本车自动驾驶使用的共享 Traffic Manager 端口 |
| `--headless --duration 10` | 无驾驶面板、运行约 10 个现实秒后退出，供验证使用 |
| `--stop-sumo` | 显式停止由启动器管理的共享桥，不加入新车 |

参数帮助：`bash run_lincoln.sh --help`。`--no-rviz` 不等于关闭传感器；`--headless` 不等于关闭 CARLA 图形渲染。

### 退出范围

| 操作 | 影响 |
|---|---|
| 关闭某个林肯面板 / 对其终端 Ctrl+C | 清理本车、31 个传感器及其 RViz；其他车和共享桥保留 |
| 对某个 NPC 客户端 Ctrl+C | 删除本次批次，包括排队未入场车辆；其他批次保留 |
| NPC 客户端异常退出 | 桥接定期检查进程身份并回收其批次 |
| `bash run_lincoln.sh --stop-sumo` | 停止共享桥并清理其同步映射，退出同步模式；应先退出车辆客户端 |
| 关闭 SUMO GUI | 共享仿真可能终止，其他客户端无法继续依赖它推进 |
| `sudo docker stop carla` | 停止 CARLA 服务，所有连接失效；不是只切断本机通信 |

共享服务以独立会话运行，关闭正常的林肯/NPC 客户端不会关闭它。关闭 SUMO GUI 或 Docker 服务不属于互不影响的客户端退出操作。

## 目录与模块实现

### 根目录

```text
Carla/
├── README.md                         本说明
├── run_lincoln.sh                    林肯一条命令入口
├── npc_traffic.py                    旧的纯 CARLA Traffic Manager 车流
├── carla_free_camera.py              独立 pygame 自由相机
├── lincoln/
│   ├── ego_vehicle.py                车辆、传感器、ROS2、驾驶、RViz 启动
│   ├── ensure_sumo.py                共享桥发现、创建、就绪检查与停止
│   ├── shared_sumo.py                本地 IPC 与 NPC 会话生命周期
│   ├── camera_ownership.py           多进程 spectator 控制权
│   ├── free_camera_controls.py       面板内自由相机输入
│   ├── carla_window_input.py         CARLA 渲染窗口的 X11 输入
│   ├── map_visualization.py          当前 CARLA 地图到 RViz Marker
│   ├── config/sensors.json           31 个传感器的安装与参数
│   ├── config/vehicle_geometry.json  车辆尺寸、中心和车轮几何
│   ├── assets/lincoln.dae            RViz 显示网格
│   ├── assets/README.md              资产来源与许可说明
│   └── rviz/lincoln.rviz             默认 RViz 模板
├── rviz/lincoln.rviz                 另一个保存的配置，非默认加载路径
└── carla-source/                     嵌套 Git 仓库，SUMO 联合仿真源码
```

根 `.gitignore` 排除了 `carla-source/` 和 `.marscode/`，因此根仓库提交不会顺带提交 SUMO 子仓库的代码。`.marscode/deviceInfo.json` 是编辑器环境文件；`.git/` 是各仓库元数据，均不参与仿真。`__pycache__/` 是运行生成的 Python 缓存，不是功能模块。

### `run_lincoln.sh`：启动入口

使用脚本所在目录定位项目，加载 ROS2、检测基本 import、解析共享桥参数，再执行 `ensure_sumo.py` 和 `ego_vehicle.py`。额外的车辆参数原样传给 Python。停止桥接使用独立分支，不生成车辆。无需手工打开 ROS 发布线程或单独运行传感器节点。

### `lincoln/ego_vehicle.py`：一车一客户端

生成 `lincoln_<10位十六进制>` 标识，使用原生 `vehicle.lincoln.mkz_2020` 蓝色车辆及 CARLA 物理。默认从当前地图 `get_spawn_points()` 筛选与现有车辆距离超过 7 米的标准出生点，再随机打乱候选顺序，最终通过 `try_spawn_actor()` 确认能否生成；自定义位置仍需用户确保是可行驶区域。

生成 RPC 返回后，等待车辆出现在状态快照且 `is_alive` 为真，避免新 actor 尚未进入客户端快照就被当作已死亡。脚本本身从不调用 `world.tick()`。

每个传感器回调只将最新数据放入容量为 1 的队列，队列满时丢弃旧帧。ROS 定时器由独立 `SingleThreadedExecutor` 线程运行，负责 TF、点云转换、图像转换和发布。Tk 主线程约每 30 ms 处理驾驶、面板和输入；另一个相机线程按 `wait_for_tick()` 更新视角。这减少传感器转换阻塞驾驶输入，但不保证线程固定到某个核心，也不会消除 CARLA 服务端主线程瓶颈。

关闭时先停止相机和 ROS 工作线程，再关闭本车 RViz、停止传感器监听并逆序销毁自身创建的 actor。输出 `SENSOR_COUNTS`，每颗传感器值为 `[收到帧数, 发布帧数]`；最新帧策略允许两者不相等。

### 车辆输入与控制

| 操作 | 行为 |
|---|---|
| 长按油门 / W | 油门 0.45，松开后收油滑行 |
| 长按左转 / A，右转 / D | 转向约 ±0.45；WASD 可组合油门与转向 |
| 长按刹车 / S | 主动制动，刹车值 1 |
| Q | 低速/接近静止时切换前进与倒车；倒车仍按 W 给油 |
| 油门右键 | 记录当前速度、开启定速；再次右键取消 |
| 速度框数字 + Enter | 输入 km/h 目标，加速到目标并保持；允许同时转向 |
| 开启自动驾驶 | 注册本车到 CARLA Traffic Manager；再次点击回到手动 |

松开按键或窗口失焦会清除手动油门/转向，默认不自动刹车；这不等于继续保持油门。定速采用按仿真时间计算的 PI 油门调节，油门上限 0.85，不自动制动。目标低于当前速度、下坡或惯性较大时，只能收油，必要时手动刹车。手动油门、刹车、换挡等会退出定速；转向不会退出定速。

自动驾驶期间不发送手动 `apply_control()`，避免双重控制。使用驾驶按钮、WASD、换挡或提交目标速度会退出自动驾驶并接管。启用时 Traffic Manager 同步模式与当前 world 一致；初始手动车辆无需调用 `set_autopilot(False)`，避免无意义地创建或联系 TM。

键盘自动重复的释放事件有短延迟取消处理，减少长按时反复收油。面板输入框、自由相机区域和驾驶按键有焦点区分，输入速度时不会误触 W/A/S/D 驾驶。

### `lincoln/config/`：车辆与传感器配置

`sensors.json` 的 `pose` 顺序为 `[x, y, z, roll, pitch, yaw]`，局部位置单位米、角度单位弧度，使用 ROS 风格 x 前、y 左、z 上。生成 CARLA sensor 时翻转 y、pitch/yaw 符号并转换为角度；结合 `vehicle_geometry.json` 的中心高度修正安装原点。

| 模组 | 组成 | 配置采样率 | 用途 |
|---|---|---|---|
| `roof_sensor_rig` | 1 个 64 线 lidar、wide/standard/tele 三相机 | 10 Hz | 车顶点云及宽/标准/长焦前向图像 |
| `surround_camera_rig` | left/right/rear/b_left/b_right 五相机 | 10 Hz | 侧方、后方和斜后方图像 |
| `corner_radar_rig` | front_left/front_right/rear_left/rear_right 四雷达 | 20 Hz | 车身四角 |
| `side_radar_rig` | side_left/side_right 两雷达 | 20 Hz | 左右侧 |
| `ultrasonic_rig` | 前、后、左、右各四颗，共 16 颗 | 10 Hz | 近距离探测模拟 |

合计 31 个传感器，含 8 路 640×360 图像、1 个激光雷达和 22 个 radar actor。超声波使用近距离 `sensor.other.radar` 模拟，不是独立声学超声波模型。采样周期被量化为仿真步长整数倍，不能超出 world 推进频率；配置 10 Hz 是数据输出节奏，不应直接当作真实硬件扫描规律。

曾使用的 `perception_rig` / `roof_lidar` 等名称已经统一为 `roof_sensor_rig`。rig 在这里表示一组固定安装的传感器模组。左右超声波 `mid_rear` 已分多次向后移动，当前两侧 x 坐标约为 `-0.901940308` 米；进一步调整应同时检查两侧 pose、RViz 安装点和实际探测覆盖。

`vehicle_geometry.json` 保存车身尺寸、几何中心、车轮中心和半径，辅助安装坐标和 Marker 展示；实际 CARLA 碰撞与车辆动力学仍来自原生蓝图。`assets/lincoln.dae` 只供 RViz 显示，不是导入 CARLA 的新动力学模型。来源说明见 [资产说明](lincoln/assets/README.md)。

### ROS2 数据与坐标

| 话题 / TF | 内容 |
|---|---|
| `/<uid>/<rig>/<sensor>/points` | `PointCloud2`，lidar 字段为 x/y/z/intensity，radar 为 x/y/z/velocity |
| `/<uid>/<rig>/<camera>/image` | RGB8 图像 |
| `/<uid>/<rig>/<camera>/camera_info` | 由分辨率和水平 FOV 计算的针孔相机内参 |
| `/<uid>/cameras/mosaic` | 八路最新图像拼图 |
| `/<uid>/markers` | 林肯网格与安装点 |
| `/<uid>/roads` | 当前 CARLA 地图的道路面和标线 |
| `map → <uid>/base_link` | 当前车辆位姿动态 TF |
| `<uid>/base_link → <uid>/<rig>/<sensor>` | 传感器固定安装 TF；相机另有 optical frame |

CARLA world 的 y 与 ROS 的 y 方向相反。车辆姿态、局部安装、lidar 点和 radar 极坐标转笛卡尔坐标均做相应转换。传感器消息使用其 CARLA 数据时间戳，车辆 TF 使用 world 快照仿真时间戳；传感器以 Best Effort QoS 发布，道路 Marker 使用 Transient Local。

拼图每格使用各相机最近一次图像，约每 0.1 现实秒尝试发布一次，不保证八路来自同一仿真帧，也没有实现严格的多传感器时间同步或完整统一 `/clock` 管理。需要算法级同步时必须另行对齐 frame/timestamp，不能把拼图作为已同步输入。

### RViz 配置与地图

默认加载 [lincoln/rviz/lincoln.rviz](lincoln/rviz/lincoln.rviz)。启动时递归替换模板中的车辆话题和 TF 前缀，修正传感器订阅为 Best Effort，并生成 `/tmp/<uid>.rviz`。因此多辆车可以共享模板但各看各的数据；保存布局时应保存到模板路径，临时文件不会成为下次默认配置。根目录 `rviz/lincoln.rviz` 不是默认模板。

八路拼图默认两列四行，顺序为 standard/tele、rear/wide、b_left/b_right、left/right。单路 Image 默认关闭，拼图默认打开。五个 rig 使用 RViz Group；`ultrasonic_rig` 内再分 front/rear/left/right。父组关闭会屏蔽子显示，子开关无法覆盖父组，这是标准组开关语义。关闭显示不停止采集或 ROS 发布。

保留 Views 面板，默认 ThirdPersonFollower，目标为本车 `base_link`。Move Camera 工具控制 RViz 视角；切换 Orbit 并以 `map` 为目标可以自由观察。RViz 相机只改变三维可视化观察方式，与 CARLA spectator、真实传感器相机图像互不相同。之前增加的独立车后跟随面板和保存视角预设已移除。Views 面板附近的空白列表属于 RViz 面板界面，不是缺失的传感器图像。

`map_visualization.py` 使用当前 CARLA 地图每 0.5 米采样的 waypoint，按 road/section/lane 分组并依 s 排序，以 lane_width 生成三角道路面；左右边界按实际标线类型、颜色与宽度生成线段，过滤不连续连接并去重。当前已经不再用车道中心线冒充车道标线。

CARLA 地图 API 不提供完整 Unreal 场景网格、护栏和材质贴图。Town04 数据缺少明确虚线间距时，RViz 使用 3 米线段/3 米间隔示意，因此无法保证与 CARLA 纹理逐像素相同。以 CARLA 为地图数据源并不等于复制完整渲染场景。

### 三个视角模块

`free_camera_controls.py` 提供 Tk 面板内相机区域；释放视角后保持原位置，点击区域取得输入焦点。WASD 平移、Q/E 升降、Shift 加速、左键拖动转向、滚轮调速、F 捕获/释放鼠标、Esc 释放、R 重置。此区域内 WASD 操作相机而不是车辆。切回跟随后恢复车后约 8 米、上方 4 米、俯角 15 度。

`carla_window_input.py` 通过 ctypes 调用 X11/Xfixes，仅在自由模式且本客户端有相机控制权时检测活动 CARLA 窗口。点击 CARLA 渲染窗口后可直接用鼠标转向及 WASD/Q/E/Shift 移动；滚轮向上加速、向下减速，与面板共用 1～300 m/s 的移动速度，通过 XInput2 监听滚轮事件，不抢占其他窗口输入；离开窗口、失去控制权或切回跟随后恢复鼠标。依赖 X11，不承诺 Wayland 等环境同样可用；不可用时保留 Tk 面板控制。

`camera_ownership.py` 按 CARLA host/port 共用所有权文件，使用 `flock` 锁和 PID＋进程启动时间识别所有者。第一辆自动取得空闲相机，新加入车辆不抢占；明确点击跟随/自由相机操作可接管。写 spectator transform 时也持有同一把跨进程锁，旧客户端失去所有权后停止更新，旧客户端退出不会删除新所有者。异常退出后过期所有权可被后续请求接管。

根 `carla_free_camera.py` 是独立 pygame 相机，默认 20 m/s、Shift 四倍速、鼠标灵敏度 0.18，提供 WASD/QE、F、Esc、R 和滚轮调速，不推进 world。它没有接入林肯的跨进程所有权协议，所以不要在林肯跟随/自由相机工作的同时运行它，否则仍可能争抢公共 spectator。

### `ensure_sumo.py` 与 `shared_sumo.py`

`ensure_sumo.py` 通过 `/proc` 查找匹配 host/port 的受支持桥接进程，使用启动锁避免两个客户端同时创建桥。没有桥时生成独立后台进程，有桥则复用；发现多个桥或无推进者却已处于同步模式时拒绝再开一个。就绪不仅看 PID，还确认 world 连续推进并匹配 `ready.json`，停止时等待旧进程完成清理。

`shared_sumo.py` 根据用户、标准化 host 和 port 生成 `/tmp/carla-lincoln-<hash>/` 私有目录。客户端写原子 JSON 请求，桥接线程读取并返回原子 JSON 响应；只有桥接线程进行 TraCI 操作。请求支持加入批次、删除批次和状态查询。批次记录其所属 PID 和启动时间，防止 PID 重用误判；定期回收死亡客户端的车辆。

运行目录中通常包括 `sumo.log`、`bridge.json`、`ready.json`、启动锁、相机所有权/锁，以及短期请求与响应文件。桥接临时 OpenDRIVE、net.xml、sumocfg 在另一个 `/tmp/tmp.../Carla/Maps/` 目录；不应把这些临时文件当作永久项目配置。

### `npc_traffic.py`：旧的纯 CARLA 车流

从当前地图标准出生点生成最多 200 辆四轮车，设置 TM 端口 8000、全局跟车距离 2.5 米，然后开启自动驾驶；退出时销毁自身车辆。它不创建 SUMO 服务、不直接调用 TraCI，也不负责同步模式配置。代码打印的 Town10HD 是旧的固定文字，实际使用的是当前地图；打印“退出后需要重启 Docker”同样不是已经验证的通用要求。

要使用共享 SUMO 随机车流，优先运行 `spawn_npc_sumo.py`，不要把这两个车流入口当作同一个方案。旧脚本与同步 world 混用前应核对 TM 同步配置。对话中提及的 `town10_traffic.py` 当前工作区没有该文件，不能将最初讨论的碰撞原因视为针对它的已完成修复。

### `carla-source/Co-Simulation/Sumo/`

| 文件 / 目录 | 功能与实现 |
|---|---|
| `spawn_npc_sumo.py` | 默认是共享桥的 NPC 客户端，创建唯一会话和批次；`--bridge-server` 是启动器使用的后台服务模式，`--standalone` 保留旧的单进程模式 |
| `run_synchronization.py` | `SimulationSynchronization` 实现 SUMO↔CARLA 双向同步；也提供指定 sumocfg 的独立入口，不应与共享桥同时推进同一 world |
| `sumo_integration/carla_simulation.py` | CARLA 连接、同步步进、actor 集合增删、物理车辆映射、信号灯状态 |
| `sumo_integration/sumo_simulation.py` | 启动 SUMO/GUI、TraCI 订阅、车辆类型/状态、moveToXY、生成/销毁和信号灯管理 |
| `sumo_integration/bridge_helper.py` | 两边坐标、车身参考点、车辆类型、灯光与交通信号状态转换 |
| `sumo_integration/constants.py` | 无效 actor ID 等公共常量；`__init__.py` 是包入口 |
| `util/netconvert_carla.py` | 将当前地图 OpenDRIVE 经 netconvert 转为 SUMO 网络，并结合 CARLA 地标处理信号灯 |
| `util/create_sumo_vtypes.py` | 生成 SUMO 车辆类型的辅助工具 |
| `util/sequential_types.py` | SUMO 转换工具使用的序列类型辅助代码 |
| `util/data/opendrive_netconvert.typ.xml` | OpenDRIVE 车道类型转换规则 |
| `data/vtypes.json` | CARLA 蓝图到 SUMO 车辆类别与参数的映射 |
| `examples/carlavtypes.rou.xml` | SUMO 车辆类型定义 |
| `examples/viewsettings.xml` | SUMO GUI 配色、道路/路口等显示设置 |
| `examples/Town01/04/05.sumocfg` | 预置示例配置，引用 examples/net 和 examples/rou；随机共享桥不直接拿它们作为当前地图网络 |
| `examples/net/`、`examples/rou/` | Town01/04/05 示例路网与路线 |
| `requirements.txt` | 上游 Python 依赖声明，使用前核对本机版本 |

嵌套仓库顶层 README、CHANGELOG、LICENSE 记录上游项目说明、版本及许可；Makefile、CMakeLists、Update 脚本、Doxyfile、mkdocs 和 CI 配置属于上游构建/文档工具，不是本项目启动 CARLA Docker 的入口。当前只保留部分源码，不能据这些文件存在就认为可以直接构建完整 CARLA。

后台桥从当前 CARLA world 导出 OpenDRIVE，生成本次临时 SUMO 路网，而非固定读取旧 Town04 示例。随机 NPC 按车辆类别筛选允许的 edge，并随机生成路线。`--filterv` 使用正则筛选蓝图，`--safe` 排除部分更容易出现事故的车型，不代表保证不碰撞；行人生成当前不支持。

同步维护 `sumo2carla_ids` 和 `carla2sumo_ids` 两张表，避免把同一辆车再次镜像到另一边。每步先处理 SUMO 状态并同步其 NPC，再推进 CARLA 并同步 CARLA 控制车辆。CARLA actor 原点与 SUMO 前保险杠参考点不同，坐标转换包含车长偏移、y 翻转、netOffset 和 yaw 转换。林肯的 `carla0`、`carla1` 等 SUMO ID 是按生成顺序分配，不能仅按“第一辆/第二辆”猜 ID。

已优化逐车重复查询：类别首次从订阅结果取值后缓存，订阅 `VAR_EDGES` 和 `VAR_ROUTE_INDEX` 读取路线与索引；改路线时更新本地缓存、销毁时清理。CARLA 控制的镜像车在 SUMO 步后又被 moveToXY，其订阅快照可能已过时，因此路线仍使用实时查询，不能为了缓存而使用失效结果。

## 对话中的问题、处理结果与排查方法

### 1. Docker 容器重名、停止和删除

曾因已有 `carla` 容器导致 `docker run --name carla` 报 Conflict。应先查清状态再决定继续使用、停止或删除，不要反复运行同一创建命令。

```bash
sudo docker ps                         # 正在运行的容器
sudo docker ps -a                       # 所有容器，包含 Exited
sudo docker images                     # 本地镜像
sudo docker logs --tail 80 carla        # 容器日志（容器仍存在时）
```

容器是镜像创建的实例。`docker stop carla` 停止其主进程；未启用 `--rm` 时容器配置和可写层通常保留，仍在 `ps -a` 中，可以 `docker start carla`。`docker rm carla` 删除停止的容器、释放名称；`docker rmi carlasim/carla:0.9.16` 删除镜像。启动带 `--rm` 的容器正常停止后会自动删除，镜像仍保留。Docker 后台服务继续运行，不等于 CARLA 容器继续运行。

图形服务曾需要 `xhost +si:localuser:root`、DISPLAY、X11 socket 和 NVIDIA 参数。sudo 密码只在对应终端输入，不写进脚本或聊天。不要用强制删除来代替正常客户端和桥接清理。

### 2. 同步模式、SUMO 暂停与车辆一闪即退

同步 world 必须有唯一推进者。SUMO GUI 暂停时，桥接可能等待它，CARLA 画面与传感器也不会正常推进；早期测试需点播放，目前受管理桥使用自动播放。

曾出现 SUMO 弹出、新林肯瞬间消失、所有 `SENSOR_COUNTS` 为零，随后 RViz 报 ROS context 无效。新车生成后未进入快照时短暂 `is_alive=False` 是重要触发点。已增加启动时桥接推进检查与车辆快照等待，并按顺序停止 ROS 工作线程。不能仅凭最后的 RViz guard condition 报错，就断定根因是传感器配置。

旧桥停止后立即开新桥还可能撞上清理/同步状态竞争，现启动器等待停止完成。若 world 已同步但没有已识别的桥，先查现有 tick 程序；不要直接开第二个 `world.tick()` 来补救。

### 3. 林肯与 NPC 启动顺序、退出互相影响

旧模式把车流和桥接生命周期绑定，退出车流会关闭 SUMO。现已拆成常驻桥＋独立会话，已联调验证“林肯先启动”“NPC 先启动”、多个 NPC 批次、单批退出、排队车辆回收以及 NPC 异常退出。

删除 NPC 时需同时清除订阅和双向映射、销毁对应 CARLA actor。早期仅移除 SUMO 车辆留下旧映射，产生过 KeyError；目前批次清理已处理这些状态。`--standalone` 仍是旧生命周期，不享有默认客户端的独立退出保证。

### 4. 长按没反应、松开/失焦刹车不符合直觉

原面板只支持按住驾驶，短点击松开就制动，曾误将长按无反应也归为点击模式。当前实现保留长按，取消松开自动刹车、失焦制动和组合前进转向按钮，改为油门/刹车/左右转向，并保留 WASD 与低速换挡。通过面板实际油门/刹车值和速度区分“未收到输入”“控制已发送但没有推进”与“视觉不顺滑”。

定速和 Traffic Manager 后续加入，已做低速目标、转向期间加速、主动制动接管和自动驾驶开关验证。定速不是完整 ACC，没有前车距离反馈和自动刹车；主动手动接管仍有必要。

### 5. 跟随一卡一卡、多车闪屏与鼠标自由视角

曾观察到释放跟随后明显更顺滑。传感器转换/ROS 发布已移到独立线程，相机改为独立按仿真帧更新，固定车身相对偏移并平滑朝向。实际 20 Hz 或更低的仿真状态仍限制跟随流畅度，这些修改不保证显示达到 60 FPS。

两辆林肯同时跟随曾反复写同一个 spectator，产生闪屏。当前已加跨进程相机所有权，第二辆不自动抢占，显式操作才交接，并通过跨进程的初次申请、接管、旧所有者清理和死亡恢复测试。已经运行的旧面板不会自动更新代码，需关闭后重新启动；独立 pygame 自由相机仍未接入此锁。

只有 Tk 区域鼠标控制不满足在 CARLA 窗口内直接观察的要求，已补 X11 活动窗口检测、鼠标捕获/恢复和键盘移动。图形会话不兼容时仍可用面板，不应把不支持 X11 的环境当成车辆故障。

### 6. RViz 排布、rig 分组、QoS 与视角

摄像头数量是 8，不是 6；roof 模组有 3 路，surround 模组有 5 路。按 Demo 实际布局最终选两列四行。两列六行只是额外留空，不增加摄像头。

平铺的雷达显示太多，已按 rig 分组，超声波再按四方向分组。父组关闭后子开关无法单独显示属于 RViz Group 的正常行为；开关只控制显示，没有节省采集成本。

保存的 RViz 模板可能含上一辆车的 namespace 和可靠性配置，启动时自动替换前缀并设置传感器 Best Effort。多车时先检查话题名和 Target Frame，而不是把第二辆车无数据直接当作传感器故障。

自由拉动 RViz 视角需要 Move Camera 和合适的 View 类型/Target Frame；ThirdPersonFollower 会随目标车移动，Orbit＋map 更适合世界观察。RViz 插件菜单里的 FPS 是第一人称观察控制器名称，并不是帧率统计。

### 7. RViz 抽搐与时间同步：尚不能认定完全解决

不同频率传感器、异步 ROS 发布、车辆 TF 与 sensor 时间差、低仿真帧率和显示负载都可能造成视觉波动。当前最新帧队列和线程拆分解决了部分阻塞，但没有完成严格的全传感器同步。

排查时先确认 CARLA 帧持续推进，再观察 TF 和消息时间戳、RViz 状态错误、点云 Decay Time 及固定坐标系。仅凭画面抽动不能断定是同步问题，也不能宣称已彻底修复。

### 8. 500 辆不实时、CPU/GPU 利用率低

Town04、目标 500 辆、0.05 秒步长的无性能分析重测曾观察到：331 辆约 0.55 倍实时、400 辆约 0.49 倍、426～443 辆约 0.45～0.47 倍；并非全部 500 辆已入场。约 440 辆时每秒推进约 9 帧，CARLA 主线程接近一个核心满载，GPU 约 25%。这些是当时场景记录，不是当前任意配置的保证。

旧性能分析中类别/路线/路线索引查询合计约 29.6 ms/步，缓存和订阅已针对这部分优化。带性能分析时约 400 辆的 0.31 倍不能与无分析重测直接相除当作准确提速比例。CARLA 主线程分段采样曾开展，但这里不凭对话缺失的数据编造最终热点占比。

14700K 的总 CPU 利用率低不表示每个核心都空闲；一个关键线程满载就可能拖慢整个同步循环。SUMO、CARLA 和 Python 各有线程/进程，不能说整套系统只使用一个核心。Python 传感器线程能减少 UI 阻塞，但 GIL、TraCI 单连接顺序及 CARLA 引擎依赖限制了随意并行。

0.05 秒是 20 Hz 仿真目标，0.01 秒是 100 Hz；步长不自动保证真实时间。每步耗时超过步长时仿真变慢；桥接只能在处理更快时补 sleep。新增 lidar/相机会增加负载。继续优化应分别测 CARLA tick、TraCI 通信、Python 转换、渲染/传感器，保持相同地图、实际车数和传感器配置，避免多个诊断脚本同时 tick。

### 9. SUMO 黑色丁字路口与 CARLA 隔离直路

现场截图显示 SUMO 的整块黑色路口，而 CARLA 对应画面有中央护栏和连续直行车道。已核对当次 SUMO 使用的 OpenDRIVE 与当前 Town04 内容完全一致；当次 netOffset 为 `(503.02, 423.76)`，同步代码包含偏移和 y 翻转。地图来源一致仍不等于路口填充形状、所有连接或护栏模型都一致。

当时第一辆 CARLA 2111 映射 `carla4`，第二辆 2155 映射 `carla5`；这些仅是该次运行 ID。第一辆所在 CARLA 路段 266 属于路口 255，第二辆在 road 43 非路口路段。两车早前的转换坐标与附近 SUMO 车道中心线吻合，但未获取同帧 SUMO 实际车辆位置，不能据此宣称所有同步误差均为零。

SUMO 窗口 `Ctrl+J` 隐藏路口形状后，截图暴露出连续直行内部车道，说明黑色整块主要遮住了内部结构。`Ctrl+J` 只切换显示，不更改通行规则或道路连接。可在 GUI 的路口显示设置里查看对应选项；官方说明见 [SUMO 路口内部连接](https://sumo.dlr.de/userdoc/Simulation/Intersections.html)。

如果只调整黑色范围，可以在 NetEdit 编辑 junction 的 `shape`；但重新计算网络可能影响内部几何，应在副本上检查。更稳妥的展示方案是隐藏原填充、另画路面和中央隔离带。如果发现 NPC 真的穿过 CARLA 护栏，则需要检查并修正车道 connection，不能靠遮盖图形解决交通逻辑。

以上是已定位现象和可选方案，目前没有改动路口 255 的形状/连接，也没有移除 CARLA 护栏；项目默认显示配置尚未因这次讨论而永久修改。临时路网每次重建，永久修改应放入网转换后处理或固定 GUI 配置流程。CARLA 静态护栏可能属于地图资源，不能假设能像普通 vehicle actor 一样运行时删除。

### 10. 自动车撞车/撞墙与交通规则

早期讨论的检查方向仍适用于 TM 车流：跟车距离和目标速度、自动变道、忽略红灯/停车标志/车辆的百分比、碰撞检测、标准出生点、world/TM 同步模式、唯一 tick 所有者，以及路线是否跨车道或经过不可行驶区域。自行 PID / apply_control 还要检查 waypoint 间距、转向参数、弯道减速与是否同时开启自动驾驶。

这些是排查方向，不代表本项目已对所有车设置了低速、关闭变道或消除了碰撞。`npc_traffic.py` 当前只明确设置 2.5 米跟车距离等代码可见参数，`--safe` 只是车型过滤。

交通灯全红的正确代码应在连接并取得 `world` 后执行：

```python
for light in world.get_actors().filter('traffic.traffic_light*'):
    light.set_state(carla.TrafficLightState.Red)
    light.freeze(True)
```

恢复时循环 `light.freeze(False)`。联合仿真中还要考虑 `--tls-manager` 每步同步信号状态，单次设红不等于整个运行期间永远保持红灯。

### 11. VSCode 未显示嵌套 Git 仓库

根仓库忽略 `carla-source/`，而该目录自身有 `.git`，需要分别查看两个仓库状态。VSCode 是否显示还取决于仓库自动扫描范围和工作区设置；可打开子目录、把它加入多根工作区或调整 Git 扫描设置，不要重新 `git init` 覆盖既有仓库。

```bash
git status --short
git -C carla-source status --short
```

## 常用只读诊断

确认当前地图与仿真是否推进，连接后等待快照再读取 actor，避免刚连接时的客户端缓存不完整：

```bash
~/carla-venv/bin/python - <<'PY'
import carla
c = carla.Client('localhost', 2000)
c.set_timeout(10)
w = c.get_world()
a = w.get_snapshot()
b = w.wait_for_tick(5)
print('地图:', w.get_map().name)
print('同步:', w.get_settings().synchronous_mode,
      '步长:', w.get_settings().fixed_delta_seconds)
print('帧:', a.frame, '->', b.frame)
for v in w.get_actors().filter('vehicle.*'):
    if v.attributes.get('role_name', '').startswith('lincoln_'):
        print(v.id, v.attributes['role_name'], v.get_transform())
PY
```

查询共享桥的车辆 ID 映射与批次，不直接创建第二个 TraCI 客户端：

```bash
# 先在终端进入工程根目录
PYTHONPATH=lincoln ~/carla-venv/bin/python - <<'PY'
from shared_sumo import request, control_folder
s = request('localhost', 2000, {'action': 'status'}, timeout=10)
print('运行目录:', control_folder('localhost', 2000))
print('CARLA -> SUMO:', s['carla2sumo'])
print('NPC 批次:', s['sessions'])
print('已入场:', len(s['sumo_vehicles']), '待入场:', len(s['pending_vehicles']))
PY
```

该状态接口当前不返回逐车位置误差；需要定量核对时应由现有桥线程在同一步采集 CARLA/SUMO 状态，而不是并发调用 TraCI 或凭两个窗口不同时间的截图下结论。

ROS2 只读检查，可在已加载 ROS2 的终端运行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic info /实际车辆uid/roof_sensor_rig/lidar/points --verbose
ros2 topic hz /实际车辆uid/roof_sensor_rig/lidar/points
```

将 `实际车辆uid` 换为启动输出的 `lincoln_...`。先确认正确 namespace、消息频率与 QoS，再检查 RViz 的 Fixed Frame、Target Frame 和显示开关。诊断应保持唯一 tick 所有者，不用重启 Docker 作为每个问题的默认解法。
