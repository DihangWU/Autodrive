# Autodrive

本工程正在逐步重构。目前已实现：启动 CARLA、切换地图、读取当前地图，通过 CARLA 官方工具转换为 SUMO 路网，并在 SUMO GUI 中显示；同时恢复了独立 Lincoln 车辆、完整传感器、ROS2/RViz、驾驶面板和自由视角。

所有示例命令默认在工程根目录执行。当前地图显示程序使用 `~/carla-venv/bin/python`，启动后在后台运行。

### 结构调整基准

以 `f511f80`（`更新 README`，分支 `575`）为功能与运行设置基准。
结构调整主要迁移目录、理清模块归属并修改路径引用；不借此重新设计运行节奏，
已有明确要求的功能修复单独保留和说明。
基准版本由共享 SUMO 联仿桥统一推进 CARLA，默认同步步长 `0.05` 秒，
每轮不足 50 毫秒时等待；车辆不调用 `world.tick()`，桥退出时恢复异步。
旧版 Lincoln 入口会自动确保共享桥已启动，这一启动关系与当前独立入口不同。
当前 `start_sync.sh` 是临时的 CARLA 时钟入口，`start_bridge.sh` 仅显示 SUMO 地图，
两者尚未恢复基准版本完整的车辆双向同步与共享桥功能。
当前跟随相机已恢复基准算法，撤掉结构调整期间为异步模式新增的位置预测。

## 1. 快速启动

### 启动 CARLA

```bash
bash carla/init/start_carla.sh
```

脚本启动 `carlasim/carla:0.9.16` 容器，名称为 `carla`。容器在后台运行，打开 1280×720 的 CARLA 窗口，使用 NVIDIA GPU 和宿主机网络。

等待 CARLA 服务启动完成，再选择 Town04：

```bash
bash carla/init/choose_town04.sh
```

该脚本通过 Python API 连接 `localhost:2000`，调用 `load_world('Town04')`，成功后打印当前地图名称。它只切换地图，不启动 CARLA。

### 打开 SUMO 地图

```bash
bash cobridge/start_bridge.sh
```

脚本立即返回终端，打印后台进程 PID 和日志路径。程序随后读取 CARLA 当前地图、转换路网，并打开 SUMO GUI。关闭启动终端不会停止程序。

后台进程启动并不表示地图转换已经成功。查看实际结果：

```bash
tail -n 50 cobridge/runtime/map_bridge.log
```

关闭 SUMO 窗口即可结束本次地图显示程序，CARLA 继续运行。

### 启动同步时钟

为使用固定节奏的跟随视角，在加载地图之后、添加车辆之前运行：

```bash
bash cobridge/start_sync.sh
```

看到 `SYNC_READY` 后，在另一个终端添加 Lincoln。时钟独立于车辆，默认以
`0.05` 秒固定仿真步长推进 CARLA，每轮不足 50 毫秒时等待，最多按真实时间
1 倍速运行；运行较慢时不会连续补帧追赶。固定 20 次仿真推进/秒不等于保证
屏幕渲染帧率，也不保证所有速度下都没有视觉跳动。

该命令在前台运行，Ctrl+C 或 SIGTERM 会恢复启动前的 world 设置。
先关闭车辆，再停止同步时钟；下次使用同步模式时仍先启动时钟，再生成车辆。
已有车辆应先退出重开，让自动驾驶与相机采用同步设置。
同一个 CARLA 服务只启动一个时钟：本入口拒绝重复启动，也拒绝接管已经处于
同步模式的服务。未来接入负责 `world.tick()` 的 SUMO 联仿桥时，应先停止此时钟。
当前 `start_bridge.sh` 只显示地图，可以与此时钟同时使用。

指定步长或短时验证：

```bash
bash cobridge/start_sync.sh --step-length 0.05
bash cobridge/start_sync.sh --duration 10
```

以上是两种独立用法，不要同时启动。强制杀进程或 CARLA 失联可能无法恢复设置；
这种情况下重启 CARLA，重新按地图、时钟、车辆的顺序启动。
不启动此时钟时，车辆仍可使用 CARLA 原有异步模式。

已在 Town04 验证固定步长、重复启动拒绝、SIGINT/SIGTERM 退出恢复，以及
驾驶面板、RViz 和全部 31 个传感器的运行与清理。一次 `--headless --no-rviz`
短时测试在清理 actor 后出现原生进程退出异常；有界面的复测正常退出，
该无界面退出问题尚未定位。同步模式的高速跟随观感仍需驾驶对比。

### 添加 Lincoln

CARLA 加载地图后，启动车辆：

```bash
bash carla/Lincoln/run_lincoln.sh
```

默认打开驾驶面板和 RViz，并保留全部 31 个传感器。每次调用生成一辆车，关闭驾驶面板或在其终端按 Ctrl+C 清理本车。无需先打开 SUMO；车辆操作详见第 7 节。

## 2. 当前功能范围

| 功能 | 当前状态 |
|---|---|
| 启动 CARLA、切换到 Town04 | 已实现 |
| 从当前 CARLA 地图导出 OpenDRIVE | 已实现 |
| 使用官方工具生成 SUMO 路网 | 已实现 |
| 导入车道类型、权限、编号映射和连接关系 | 已实现 |
| 根据 CARLA 地标生成 SUMO 信号灯信息 | 已实现 |
| 加载官方 SUMO GUI 显示配置 | 已实现，白色背景 |
| CARLA 与 SUMO 车辆生成、运动和销毁同步 | 尚未接入当前入口 |
| 实时同步两边的信号灯状态 | 尚未接入 |
| Lincoln、完整传感器、ROS2、RViz、驾驶与自由视角 | 已恢复，入口位于 `carla/Lincoln/` |
| NPC 批次管理、RSU、OMNeT++/Veins 通信 | 尚未重建 |

当前 `start_bridge.sh` 启动的是地图导出和显示流程，还不是完整的实时同步桥。它不调用 `world.tick()`，不修改 CARLA 同步模式，不建立 TraCI 连接。

## 3. 地图如何从 CARLA 到 SUMO

```text
正在运行的 CARLA
    → Python API 获取当前地图
    → to_opendrive() 导出 OpenDRIVE
    → 官方 netconvert_carla.py
        → netconvert + 官方车道类型配置
        → 处理 CARLA 交通信号灯地标及控制连接
    → 生成 .net.xml 和 .sumocfg
    → SUMO GUI 加载路网与官方 viewsettings.xml
```

地图来源是正在运行的 CARLA，不固定为 Town04，也不直接使用官方目录里预置的 `examples/net/Town04.net.xml`。

每次启动都会重新导出和转换。程序打开 GUI 后，不会持续监听 CARLA 地图变化。切换地图时，先关闭旧 SUMO 窗口，在 CARLA 中选好地图，再重新运行 `bash cobridge/start_bridge.sh`。

### 路网包含的信息

- 车道类型和通行权限。
- OpenDRIVE 道路、车道的原始编号映射，保存在 `origId` 参数中。
- 道路与车道之间的连接关系。
- 交通信号灯控制器、受控连接和 CARLA 地标映射。

SUMO 自身的道路和车道 ID 不保证与 CARLA ID 完全相同，`origId` 用于记录原始映射。信息保存在路网里，不代表 GUI 默认会显示全部编号标签。

当前调用官方工具时启用了 `--guess-tls`，用于推断路口信号灯的控制连接。SUMO 中生成的是默认配时，尚未读取运行中的 CARLA 实时灯色。

### 界面与坐标

SUMO GUI 加载官方 `examples/viewsettings.xml`，使用白色背景及其中的道路、车道等显示设置。它展示二维路网，不复制 CARLA 的三维建筑、纹理或摄像头画面。

SUMO 坐标经过路网转换处理，可能与 CARLA 世界坐标不同。转换生成的 `.net.xml` 中包含 `<location>`、`netOffset` 等信息；当前界面没有额外显示 CARLA 坐标，也没有加入时间倍率或复制 CARLA 坐标功能。

## 4. 目录与文件职责

```text
Autodrive/
├── README.md
├── how_to_run.md                       # 简要启动命令
├── carla/
│   ├── init/
│   │   ├── start_carla.sh              # 启动 CARLA Docker 容器
│   │   ├── choose_town04.sh            # 通过 API 加载 Town04
│   │   └── notes.md                    # CARLA 使用笔记
│   └── Lincoln/
│       ├── run_lincoln.sh             # 单车启动入口，加载 ROS2 环境
│       ├── ego_vehicle.py             # 车辆、传感器、ROS2、驾驶面板
│       ├── camera_ownership.py        # 多车 spectator 归属
│       ├── free_camera_controls.py    # 面板自由相机
│       ├── carla_window_input.py      # CARLA 窗口的 X11 输入
│       ├── map_visualization.py       # CARLA 地图到 RViz Marker
│       ├── config/                   # 传感器和车辆几何配置
│       ├── assets/                   # RViz 车辆网格及许可说明
│       └── rviz/                     # RViz 布局模板
└── cobridge/
    ├── start_sync.sh                  # 独立同步时钟入口（前台）
    ├── sync/sync_clock.py             # 固定步长推进、限速和退出恢复
    ├── start_bridge.sh                 # 后台启动地图显示程序
    ├── .gitignore                      # 忽略生成文件及 Python 缓存
    ├── map/
    │   ├── map_bridge.py               # 地图导出、转换及 GUI 启动
    │   └── notes.md                    # 地图模块笔记
    ├── carla-source-official/          # 引入的 CARLA 官方源码
    │   ├── LICENSE
    │   └── Co-Simulation/Sumo/
    │       ├── util/
    │       │   ├── netconvert_carla.py
    │       │   └── data/opendrive_netconvert.typ.xml
    │       ├── examples/               # 示例路网、车辆类型、GUI 配置
    │       ├── data/                   # 车辆类型数据
    │       ├── sumo_integration/       # 官方 CARLA/SUMO 同步适配器
    │       ├── run_synchronization.py  # 官方同步入口，当前未调用
    │       └── spawn_npc_sumo.py       # 官方车流入口，当前未调用
    └── runtime/                       # 自动生成，不提交到 Git
        ├── map_bridge.log
        └── Town04/                    # 实际名称取决于当前地图
            ├── Town04.xodr
            ├── Town04.net.xml
            ├── Town04.sumocfg
            └── netconvert.log
```

`map_bridge.py` 保留在 `cobridge/map/`；运行文件统一写入 `cobridge/runtime/`。路径根据脚本位置计算，不依赖启动时的工作目录。

## 5. 环境与依赖

当前本机使用的环境：

| 项目 | 配置 |
|---|---|
| CARLA 服务 | Docker 镜像 `carlasim/carla:0.9.16` |
| CARLA Python API | `carla 0.9.16` |
| Python 解释器 | `~/carla-venv/bin/python` |
| SUMO | `1.22.0`，默认安装目录 `~/sumo-1.22.0` |
| XML 处理 | Python 标准库及虚拟环境中的 `lxml` |
| 车辆可视化 | ROS2 Jazzy、RViz2、Tkinter、NumPy、OpenCV 和 PyYAML |
| 显示环境 | 当前 CARLA 启动脚本使用 X11、`DISPLAY` 和 `xhost` |

地图转换需要 `netconvert`、SUMO 的 `sumolib` 和 CARLA Python API；打开窗口还需要 `sumo-gui`。当前流程没有建立 TraCI 连接。

程序寻找 SUMO 可执行文件的顺序：

1. 若设置了 `SUMO_HOME`，先查找其 `bin/`。
2. 未设置时，先查找 `~/sumo-1.22.0/bin/`。
3. 找不到时，再查找系统 `PATH`。

使用其他 SUMO 安装目录：

```bash
SUMO_HOME=/你的/SUMO/安装目录 bash cobridge/start_bridge.sh
```

程序会根据选中的 `netconvert` 安装位置，为官方转换子进程设置 `SUMO_HOME` 和 `PATH`。

## 6. 参数与直接运行

查看帮助，帮助内容在当前终端打印：

```bash
bash cobridge/start_bridge.sh --help
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--host` | `localhost` | CARLA 服务地址 |
| `--port` | `2000` | CARLA RPC 端口 |
| `--timeout` | `30` | CARLA API 超时时间，单位秒 |
| `--export-only` | 关闭 | 只生成路网和配置，不打开 SUMO GUI |

例如：

```bash
bash cobridge/start_bridge.sh --host localhost --port 2000 --timeout 60
```

只导出路网时，启动脚本仍会在后台执行，结果查看日志：

```bash
bash cobridge/start_bridge.sh --export-only
```

调试时可以直接在前台运行 Python，实时查看输出：

```bash
~/carla-venv/bin/python cobridge/map/map_bridge.py
```

只导出并等待完成：

```bash
~/carla-venv/bin/python cobridge/map/map_bridge.py --export-only
```

地图选择脚本的 CARLA API 超时为 120 秒，与地图显示程序的默认 30 秒分开设置。

## 7. Lincoln 车辆、传感器与驾驶

### 启动一辆车

先启动 CARLA 并选好地图，然后执行：

```bash
bash carla/Lincoln/run_lincoln.sh
```

每次调用生成一辆蓝色 `vehicle.lincoln.mkz_2020`、完整传感器组、一个鼠标驾驶面板和对应的 RViz2 窗口。启动器加载 ROS2 Jazzy 环境，直接连接 CARLA；当前不自动启动 SUMO，也不依赖地图显示程序。

车辆入口在前台运行，便于查看状态和用 Ctrl+C 退出；地图显示入口仍在后台运行。关闭驾驶面板也会清理本车、传感器和本车的 RViz。关闭 RViz 窗口本身不会删除车辆。

### 出生位置与多车

无位置参数时，从当前地图的标准出生点中随机选择，先排除距离现有车辆不足约 7 米的位置，再由 CARLA 的 `try_spawn_actor` 判断能否生成。

指定标准出生点编号：

```bash
bash carla/Lincoln/run_lincoln.sh --spawn-index 12
```

指定 CARLA 世界坐标：

```bash
bash carla/Lincoln/run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
```

`--pose` 依次是 x、y、z（米）、yaw（度），与 `--spawn-index` 互斥。固定位置被占用会报错，不会改成随机位置。

在两个终端运行以下命令，可在 Town04 的两个不同位置生成两辆车：

```bash
# 终端一
bash carla/Lincoln/run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
```

```bash
# 终端二
bash carla/Lincoln/run_lincoln.sh --pose 235.597504 -172.815598 0.695892 180.326126
```

每辆车使用独立的 `lincoln_<10位十六进制>` 角色名、ROS 节点名、话题命名空间和 TF 前缀。当前尚未建立 SUMO 映射，因此还没有由桥分配的 `lincoln0`、`lincoln1` 等 SUMO 名称。删除其中一辆只清理它自己的对象。

### 完整传感器配置

旧版 `sensors.json`、`vehicle_geometry.json` 和 RViz 模板已恢复。默认创建 31 个 CARLA 原生传感器 actor，未削减原来的安装数量和参数。

| 模组 | 数量与类型 | 配置频率 | 主要参数 |
|---|---|---|---|
| `roof_sensor_rig` | 1 个激光雷达、3 路相机 | 10 Hz | 激光范围 100 米、64 通道；相机 FOV 135°、60°、20° |
| `surround_camera_rig` | 5 路相机 | 10 Hz | 左右和后方 100°，后侧两路 120° |
| `corner_radar_rig` | 4 个雷达 | 20 Hz | 范围 30 米 |
| `side_radar_rig` | 2 个雷达 | 20 Hz | 范围 30 米 |
| `ultrasonic_rig` | 16 个近距雷达 | 10 Hz | 范围 3 米，用于原超声安装位置的替代模拟 |

全部相机为 640×360。总计 8 路相机、1 个激光雷达和 22 个雷达节点。`ultrasonic_rig` 不是 CARLA 的原生超声传感器，而是配置成近距探测的 `sensor.other.radar`。

传感器安装参数集中在 `carla/Lincoln/config/sensors.json`，车辆尺寸与参考中心位于 `carla/Lincoln/config/vehicle_geometry.json`。位置和角度由程序转换到 CARLA 传感器坐标，配置中的角度使用弧度。

实际数据频率受 CARLA 仿真步长和处理速度影响。每个传感器使用容量为 1 的最新帧队列，处理不及时时覆盖旧数据，避免积压。退出时输出 `SENSOR_COUNTS`，每项为 `[接收帧数, 发布帧数]`；两者可能不同。

### 鼠标、键盘与自动驾驶

| 操作 | 功能 |
|---|---|
| 按住面板的油门、刹车、左转、右转按钮 | 手动驾驶 |
| 驾驶面板获得焦点时按 W / S / A / D | 油门 / 刹车 / 左转 / 右转，支持组合按键 |
| Q 或“当前：前进/倒车”按钮 | 低速时切换前进和倒车，速度高于约 0.5 m/s 时不切换 |
| 右键油门按钮 | 以当前速度进入定速，再次右键取消 |
| 输入目标速度并按 Enter | 按 km/h 设置定速目标 |
| “开启自动驾驶” | 交给 CARLA Traffic Manager 控制 |
| “关闭自动驾驶”或手动驾驶操作 | 返回手动控制 |

松开驾驶按钮或失去驾驶焦点时清除手动按键状态；未启用定速时收油滑行。定速控制根据速度误差调整油门，手动油门或刹车会取消定速。倒车也会取消定速。

```text
鼠标 / 键盘驾驶面板
    → ego_vehicle.py 计算 VehicleControl
    → CARLA 车辆控制接口
    → CARLA 更新车辆运动
```

自动驾驶由 CARLA Traffic Manager 执行，默认端口为 8000。车辆脚本不通过 SUMO 驾驶 Lincoln。

Traffic Manager 仍在**第一次点击“开启自动驾驶”时**创建或连接，不在后台提前准备。连接成功后，本车保留该实例，后续切换直接复用。

正常关闭窗口、按一次 Ctrl+C 或 `--duration` 到期时，先取消本车自动驾驶并清理车辆和传感器，再对本进程创建的 Traffic Manager 调用 `shut_down()`，注销 CARLA 中的连接记录，避免下一次启动先等待已经失效的旧连接。只连接到其他进程的 TM 时，不调用其 `shut_down()`，以免停止对方的自动驾驶服务；归属由 `traffic_manager.py` 检查本进程的 Linux TCP 监听套接字确定。

同一端口的 TM 仍由最初创建它的进程承载，退出创建者会结束该 TM；本次修复没有将其改成独立常驻服务。多车共享时先退出连接者、最后退出创建者。旧版本退出或强制终止留下的记录，升级后的第一次连接仍可能等待一次恢复，之后正常退出会主动注销。

本次用完整 31 个传感器、驾驶面板和 RViz 回归：共享连接者正常退出未注销创建者的 TM；创建者按一次 Ctrl+C 约 0.565 秒完成退出，重启后的首次开启自动驾驶约 0.047 秒。修复前同流程重连约 4.15 秒。实测数值随负载变化，正常退出后的旧连接超时等待已在该流程中消除。

### 跟随视角、自由视角和多车归属

点击“释放视角”进入自由相机，点击“跟随本车”恢复车后约 8 米、上方约 4 米的跟随视角。

跟随实现沿用 `f511f80`：独立线程等待 CARLA 仿真帧，使用该帧的车辆位置，
保持车后 8 米、上方 4 米的偏移；只对朝向进行时间常数为 0.10 秒的平滑，
俯角为 -15°。没有额外的位置预测，不改变车辆运动或传感器配置。
修改后需退出并重新启动对应 Lincoln 客户端，已启动的进程不会自动加载新代码。

自由相机控制区域支持：

- WASD 平移，Q/E 下降和上升，Shift 四倍加速。
- 按住左键拖动转向，滚轮调整移动速度，范围约 1～300 m/s。
- F 捕获或释放鼠标，Esc 释放，R 恢复进入自由模式时的位置。

在 X11 环境中，自由模式还支持点击 CARLA 渲染窗口后直接控制相机。它使用 X11、Xfixes 和 XInput2；接口不可用时保留面板自由相机控制。

CARLA 的 spectator 相机由所有车辆共享。第一辆车取得空闲视角，新车不会自动抢占；在另一辆车的面板切换视角时可以接管。`camera_ownership.py` 通过按服务地址区分的跨进程锁、PID 和进程启动时间保证视角归属，旧车退出不会清除新车的所有权。

### ROS2 与 RViz

```text
CARLA 原生传感器回调
    → Lincoln 最新帧队列
    → ROS2 发布线程
    → 图像、点云、CameraInfo、TF 和 Marker
    → 本车 RViz2 订阅并显示
```

常用话题以 `/<车辆角色名>/` 开头：

| 话题后缀 | 消息 / 内容 |
|---|---|
| `<传感器名>/image` | `sensor_msgs/Image`，相机 RGB 图像 |
| `<相机名>/camera_info` | `sensor_msgs/CameraInfo`，相机内参 |
| `<传感器名>/points` | `sensor_msgs/PointCloud2`，激光或雷达点云 |
| `cameras/mosaic` | 8 路相机拼图，默认两列四行 |
| `markers` | 车辆模型与传感器安装位置 |
| `roads` | 当前 CARLA 地图的路面与车道线 Marker |

TF 使用全局 `map` 和每车独立的 `<角色名>/base_link`、传感器 frame、相机 optical frame。激光点包含 `intensity` 字段，雷达点包含 `velocity` 字段。

RViz 道路来源于当前 CARLA 地图的 waypoint 和车道标线信息，不读取 SUMO 路网或三维建筑纹理。坐标从 CARLA 的 y 向右转换为 ROS 的 y 向左，因此 ROS 地图中的 y 与 CARLA 世界 y 符号相反；`--pose` 始终使用 CARLA 坐标。

默认布局位于 `carla/Lincoln/rviz/lincoln.rviz`，启动时替换车辆命名空间并在 `/tmp/<角色名>.rviz` 生成本车配置。车辆网格位于 `assets/lincoln.dae`，其来源与许可见 `assets/README.md`。

#### 激光点云的完整扫描显示

激光点云沿用旧版 RViz 单层布局：`roof_sensor_rig → lidar` 使用 `AxisColor` 按高度着色，`Decay Time: 0` 保留最新一批完整点云，直到下一批替换。没有添加当前/历史双色层，也没有长时间历史累积。

原来的设置用 `sensor_tick≈0.1` 降低激光采集频率。在当前异步 CARLA 中，原生激光雷达只按被执行那一帧的 world 步长推进扫描角度，跳过中间帧会导致实际扫完前方视场的时间变长，每条消息只包含一小段角度。因此现在把激光雷达的原生 `sensor_tick` 设为 `0`，每帧推进扫描；`lidar_scan.py` 收集至少一个扫描周期的片段，在完整扫描后按配置中的 `rate` 限制输出，供原有 `/roof_sensor_rig/lidar/points` 话题和 RViz 使用。

当前异步模式下，脚本沿用 `dt=0.05` 的参数基准，设置 `rotation_frequency=20 Hz`，覆盖一次 120° 水平视场的目标时间约为 **0.05 仿真秒**。配置中的 `rate=10 Hz` 控制完整点云的输出上限，目标约每 **0.1 仿真秒**更新一次；实际输出会向完整扫描结束时刻及 ROS 发布调度对齐。扫描周期、点云输出周期和现实时间下看到的更新速度不同，现实速度还取决于仿真倍率。

车辆移动时，各片段先按它们各自的测量位姿转换到最后一帧的传感器坐标系，再合并，避免直接叠加移动坐标中的点。每个输出的时间戳使用最后一帧测量时间；这属于扫描片段的位姿补偿，不包含对行人、车辆等动态目标自身运动的补偿。

此修复保留激光安装位置、100 米范围、64 通道、120° 水平视场、原有点率和着色，也保留全部相机和其他雷达配置。已运行的车辆仍使用启动时的代码，退出对应车辆客户端后重新执行启动命令才能应用。

当前 Town04 异步实测中，ROS 收到 56 批完整激光点云，每批左右边界均接近 ±60°，平均输出间隔约 0.105 仿真秒；已验证片段位姿补偿、原 RViz 单层布局加载以及测试车辆退出清理。此结果是本次运行的测量，不保证不同负载下达到相同的现实帧率。

### Lincoln 参数

```bash
bash carla/Lincoln/run_lincoln.sh --help
```

| 参数 | 默认值 / 用途 |
|---|---|
| `--host` / `--port` | `127.0.0.1` / `2000`，CARLA 服务 |
| `--tm-port` | `8000`，Traffic Manager |
| `--pose X Y Z YAW` | 指定 CARLA 出生坐标与朝向 |
| `--spawn-index N` | 指定标准出生点编号 |
| `--sensors all` | 默认完整 31 个传感器 |
| `--sensors lidar` | 显式选择仅激光模式，完整配置文件不改变 |
| `--no-rviz` | 不打开 RViz，传感器和 ROS 发布仍运行 |
| `--rviz-config PATH` | 替换 RViz 模板，相对路径以工程根目录为基准 |
| `--camera-rows 4` / `6` | 两列四行或六行，六行的额外两行为空 |
| `--headless` | 不打开驾驶面板，仍生成车辆与传感器；可结合 `--no-rviz` |
| `--duration N` | 在启动后约 N 个现实秒结束，默认 0 表示持续运行 |

例如，保留完整传感器但不打开 RViz：

```bash
bash carla/Lincoln/run_lincoln.sh --no-rviz
```

无面板、无 RViz 的短时验证：

```bash
bash carla/Lincoln/run_lincoln.sh --headless --no-rviz --duration 20
```

车辆脚本不调用 `world.tick()`、不改变 world 同步模式。当前可直接使用 CARLA 异步模式；未来接入同步桥后，由独立桥负责推进时钟。若 world 已处于同步模式却没有任何进程推进帧，车辆初始化会超时。

### 文件职责与退出

- `ego_vehicle.py`：车辆、传感器、ROS 发布、面板、驾驶及本车资源生命周期。
- `camera_ownership.py`：多车共享 spectator 的归属和锁，不依赖 SUMO 控制协议。
- `traffic_manager.py`：识别本进程拥有的 TM 监听端口，供退出时安全注销使用。
- `free_camera_controls.py`、`carla_window_input.py`：面板和 CARLA 窗口的自由相机输入。
- `map_visualization.py`：CARLA 道路到 ROS Marker 的转换。
- `lidar_scan.py`：逐帧激光片段拼接、位姿补偿和完整点云输出限频。
- `config/`：完整传感器参数与车辆几何。
- `rviz/`、`assets/`：RViz 模板及车辆网格。

关闭驾驶面板、正常 Ctrl+C 或 `--duration` 到期时，脚本停止工作线程、关闭本车 RViz、停止传感器监听并销毁本车创建的 actor，同时释放其视角归属。强制结束进程无法保证执行这些清理步骤。

切换 CARLA 地图前先退出车辆客户端，再停止同步时钟（如已启动）。当前 SUMO 窗口只显示静态道路，Lincoln 不会自动出现在 SUMO 中；车辆映射由后续的独立同步桥实现。

## 8. 日志、退出与清理

| 文件 | 内容 |
|---|---|
| `cobridge/runtime/map_bridge.log` | 后台程序输出、CARLA 连接错误、GUI 输出 |
| `cobridge/runtime/<地图名>/netconvert.log` | 官方转换工具与 netconvert 的输出、警告和错误 |
| `<地图名>.xodr` | 从本次 CARLA 地图导出的 OpenDRIVE |
| `<地图名>.net.xml` | 本次转换生成的 SUMO 路网 |
| `<地图名>.sumocfg` | 路网和 GUI 显示配置的引用 |

同一地图再次启动时会覆盖其生成文件和转换日志；每次后台启动都会覆盖 `map_bridge.log`。

查看 Town04 的转换日志：

```bash
tail -n 50 cobridge/runtime/Town04/netconvert.log
```

关闭 SUMO 窗口会结束对应地图显示程序，不关闭 CARLA。后台模式下，启动终端的 Ctrl+C 不用于停止已经返回的程序。当前没有独立的桥停止命令，也没有防止重复启动的单实例检查；关闭旧窗口后再启动，避免重复窗口和共享输出文件。

停止 CARLA 容器：

```bash
sudo docker stop carla
```

当前容器使用 `--rm`，退出后会自动删除容器，Docker 镜像仍保留。后台启动模式下不需要保持原终端打开。

关闭地图显示程序后，可删除全部生成文件和日志，下次启动会重新生成：

```bash
rm -rf cobridge/runtime
```

此命令只清理运行产物，不删除地图程序或官方源码。

## 9. 常见问题

### CARLA 仍显示 Town10

当前启动脚本没有指定地图，启动后单独运行：

```bash
bash carla/init/choose_town04.sh
```

先等 CARLA 服务启动完成。切换成功后，脚本会打印实际地图名称。

### CARLA 连接超时

确认容器仍在运行，并查看日志：

```bash
sudo docker ps
sudo docker logs --tail 50 carla
```

确认 CARLA 已完成启动，且 `--host`、`--port` 与服务一致。地图显示程序使用 `--timeout` 调整 API 超时，但它不会自动等待一个尚未启动的服务。

### 容器名称 carla 已被占用

检查现有容器：

```bash
sudo docker ps -a
```

已运行的 CARLA 可以继续使用，无需重复启动；需要重新启动时，先停止现有容器。

### 启动后没有 SUMO 窗口

先查看 `map_bridge.log`，再查看对应地图的 `netconvert.log`。检查 CARLA 连接、SUMO 安装位置、官方文件是否完整，以及 `DISPLAY` 是否有效。

### 路网转换出现警告

警告不一定表示失败。程序会检查官方转换工具的退出码；转换失败会在主日志中提示转换日志的位置。成功转换仍可能包含几何或地标警告，具体道路问题应结合日志和地图检查。

### SUMO 与 CARLA 的图形或信号灯不同

SUMO 显示的是转换后的二维路网，CARLA 显示三维世界。当前没有车辆或灯色的实时同步；原始编号映射也不代表两边使用完全相同的内部 ID。

### Lincoln 无法生成或没有数据

检查 CARLA 是否正在推进帧。若为同步模式，需要独立的时钟推进者；当前地图显示程序不提供这一功能。固定出生点被占用时，换一个位置或使用无参数的随机出生。

RViz 没有数据时，查看车辆终端中的 `VEHICLE`、`RVIZ_CONFIG` 输出，确认话题命名空间属于当前车辆。使用 `--no-rviz` 只关闭窗口，不关闭传感器。

## 10. 官方工具来源

- [CARLA 官方仓库](https://github.com/carla-simulator/carla)
- [CARLA 0.9.16 SUMO 联仿目录](https://github.com/carla-simulator/carla/tree/0.9.16/Co-Simulation/Sumo)
- 本地许可：[cobridge/carla-source-official/LICENSE](cobridge/carla-source-official/LICENSE)

当前工程从本地官方目录调用转换脚本，保留官方车道类型配置和显示配置。官方同步适配器及车流脚本已引入，但尚未接入当前运行流程。保留官方许可文件及源码版权声明。
