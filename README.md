# Autodrive

CARLA、SUMO 和 CoBridge 分别运行。CARLA 负责三维场景、物理和传感器；SUMO 负责路网与交通仿真；CoBridge 负责时间推进、已登记 Lincoln 的 CARLA → SUMO 同步，以及 SUMO → CARLA 红绿灯同步。RSU 保留现有独立进程，生命周期由它自己管理。详细基础架构见 [architecture.md](architecture.md)。

## 启动

每个终端先加载快捷命令：

```bash
source /home/maple/Desktop/Autodrive/setup.bash
```

分别在独立终端运行：

```bash
# CARLA：启动并保留当前地图，需切换 Town04 时再执行 choose_town04
run_carla

# 首次准备地图，或更换地图后：读取 CARLA 当前地图生成 SUMO 配置
map_carla2sumo

# 加载已经转换好的地图配置
run_sumo

# 启动共享时钟；CARLA/SUMO 就绪后自动接入
run_cobridge

# 按需添加 RSU
run_rsu

# 按需添加一辆 Lincoln；再次执行会创建另一辆
add_lincoln
```

`run_cobridge`、`run_sumo`、`run_rsu`、`add_lincoln` 保持前台运行，在各自终端按 Ctrl+C 退出。Lincoln 不会自动启动、关闭或切换 CARLA、SUMO、CoBridge，也不调用 `world.tick()`。SUMO 与 CoBridge 默认步长为 0.05 秒。

## SUMO 自动控制两端红绿灯

正常运行 `run_cobridge` 即启用，无需额外手动切换。CARLA 和 SUMO 完整接入且地图指纹一致后，桥会读取本次 SUMO 路网 `.net.xml` 中的 `tlLogic/param linkSignalID:<连接索引>`，通过 OpenDRIVE 地标对应 CARLA 信号灯实体。实现位于 `foundation/cobridge/traffic_lights/sumo_traffic_lights.py`，复用官方 `SumoTLLogic` 和 `BridgeHelper.get_carla_traffic_light_state()`，不修改官方源码，也不另开 SUMO 连接或推进线程。

CARLA 0.9.16 的冻结接口作用于全场红绿灯，因此接管前必须覆盖全部 CARLA 信号灯实体，并检查 SUMO 控制器的映射；缺失的 actor ID、地标或控制器会明确报告，保持 CARLA 原控制，每秒重试。匹配成功后打印接管数量，冻结 CARLA 自主切灯；SUMO 继续执行现有程序，桥订阅实际灯色，每帧一次获取所有控制器的订阅结果，并只向发生变化的 CARLA 灯写入状态；退出时还原接管前的 SUMO 订阅变量。后续通过 SUMO 手动切相位、切程序或设置灯色也会被读取。不同程序对同一连接的地标映射必须一致，运行时新增或改写路网映射需要重启桥。

保留原有推进顺序：在 CARLA 每次 tick 和 TM 控制计算之前，应用 SUMO 上一已完成步骤的灯色，再推进 CARLA 和 SUMO。因此这里是按 0.05 秒步长耦合，不是两端渲染画面的原子刷新；SUMO 本轮发生的相位变化会用于下一次 CARLA 帧。SUMO 自主车流依据 SUMO 信号灯通行，CARLA TM 依据同步后的 CARLA 灯行驶；手动驾驶及自定义算法仍需自行遵守信号。

同一个 CARLA 灯若对应不同灯色的 SUMO 连接，会沿用官方保守取红灯规则，并报告冲突 actor ID；无法把多个车道方向的灯色无损显示为一盏实体灯。灯色枚举转换沿用官方规则，特殊的闪烁/右转信号可能转换为 Unknown，不能等同于普通绿灯。

SUMO 失联、桥主动脱离或 Ctrl+C 正常退出时，恢复保存的 CARLA 灯色和接管前的全场冻结状态，原来的红黄绿时长未修改；CARLA 世界已切换时不会向旧世界写入。恢复的是原灯色和冻结状态，不回放接管期间的 CARLA 相位，也不精确重置内部计时器。信号灯 RPC 失败时会尝试恢复并报告；SIGKILL、CARLA 崩溃等情况下无法保证执行清理。桥状态查询的 `traffic_lights` 字段可查看接管状态、覆盖数量和冲突信息。

Town04 实测匹配 43/43 个 CARLA 红绿灯、12 个 SUMO 控制器，手动改变 SUMO 相位后 CARLA 灯色随之改变。灯色切换边界曾观察到一帧快照差异，随后连续 29 帧、1247 次比较一致；RPC 写入和快照更新不是原子操作。正常退出后，CARLA 恢复原先异步运行和未冻结状态；SUMO 脱离后恢复、重新接入后再接管均通过。订阅读取版本在空载 Town04 的桥步进耗时中位数约 20.6 ms，取代逐个管理请求约 77.6 ms 的实现；有车辆和传感器时需另行测量。离线回归覆盖缺失映射、程序映射冲突、同灯多连接冲突、读取/写入失败恢复、原冻结状态保留及地图切换。启动中的旧桥不会加载此功能，需正常退出后重新运行 `run_cobridge`。

## SUMO GUI 的低延迟构建

普通 `run_sumo` 现在自动使用本工程专用的 SUMO GUI。调用链为：

```text
run_sumo
  → foundation/sumo/init/sumo_init.sh
  → foundation/sumo/init/sumo_init.py
  → foundation/sumo/init/build_low_latency_sumo_gui.py
      ensure_low_latency_sumo_gui()
  → 启动 .runtime/sumo/gui/sumo-gui
```

构建脚本与启动入口同放在 `foundation/sumo/init/`，运行产物仍放在 `.runtime/sumo/gui/`。构建脚本顶部使用中文 `__doc__` 说明职责；脚本仅负责构建，进程管理仍由原来的 `sumo_init.py` 完成。用户不需要额外执行构建命令。

原因是 SUMO 1.22.0 的 Linux GUI 在 `GUIRunThread::tryStep()` 中，默认会每秒为重绘预留一次 100 ms 休眠。这段等待会使 TraCI 的 `simulationStep()` 偶发变慢；CoBridge 串行推进两端，因而也会拖慢 CARLA 新帧和点云。此问题曾在旧 Git `833f60e` 中修正。

当前修复只在项目内复制 `GUIRunThread.cpp`，将该处 `sleep(100)` 改为 `sleep(10)`，编译这一个源文件，复制并更新 GUI 静态库，然后复用现有 CMake 链接参数生成独立可执行文件。系统 SUMO 的原始源码、静态库与 `bin/sumo-gui` 均不改写，其他 GUI 功能和地图数据保持原样。

### 构建环境与缓存

需要 Linux、本机 SUMO 完整源码、已经完成的 GUI CMake 编译产物，以及原编译器和 `ar`。当前本机配置示例：

```bash
export SUMO_HOME="$HOME/sumo-1.22.0"
# 可选：默认即为 $SUMO_HOME/build/cmake-build
export SUMO_BUILD_DIR="$SUMO_HOME/build/cmake-build"
run_sumo
```

构建依赖 SUMO 源码中的 `src/gui/GUIRunThread.cpp`、CMake 目录中的 `src/gui/CMakeFiles/gui.dir/flags.make`、`src/CMakeFiles/sumo-gui.dir/link.txt` 及相应库和对象。只有发行版可执行文件、没有源码或编译产物时，无法使用这种增量构建方式；脚本会明确报错，不会静默退回带 100 ms 等待的原版 GUI。

首次运行会提示正在构建。后续根据构建脚本、源码、编译/链接参数和依赖库/对象的文件信息检查缓存，输入未变则直接返回项目 GUI 路径。缓存校验仍需保留源码及原 CMake 编译产物。文件锁避免重复并发构建；编译和链接成功后原子替换可执行文件与缓存签名，失败时保留原产物并报告日志位置。

```text
.runtime/sumo/gui/
├── sumo-gui          # 项目专用 GUI
├── build/            # 修改后的源码副本、对象和 GUI 静态库副本
├── build.log         # 编译、链接命令及错误输出
├── build.lock        # 构建锁
└── signature         # 构建输入签名
```

无窗口模式继续使用原生 `sumo`，不执行 GUI 构建，也不要求 GUI 的编译产物：

```bash
run_sumo --headless
```

如需提前构建，可在工程根目录执行（通常不需要）：

```bash
~/carla-venv/bin/python foundation/sumo/init/build_low_latency_sumo_gui.py
```

已打开的旧 SUMO 窗口不会自动更换程序。正常退出旧 `run_sumo` 后重新运行即可生效；首次构建耗时不计入随后 SUMO 进程的 20 秒启动等待。

临时副本的完整联仿验证中，CARLA 帧间隔峰值约从 150 ms 降至 57 ms，点云最长到达间隔约从 213 ms 降至 127 ms。这说明周期性长阻塞明显减少，但不是对所有显示抖动的零延迟保证。正式构建已验证首次编译、缓存复用、输入变化重建、失败不更新签名、GUI 实际启动路径及 `--headless` 跳过构建。 正式产物的独立对照测试中，80 次步进调用在预热后，原版最长等待约 100.3 ms、4 次超过 50 ms；项目 GUI 最长约 10.3 ms、没有超过 50 ms 的调用。

## Lincoln 的启动和出生位置

完整入口只保留两个文件：

```text
modules/lincoln/add_lincoln.py
modules/lincoln/add_lincoln.sh
```

快捷命令和完整脚本等价：

```bash
add_lincoln
bash modules/lincoln/add_lincoln.sh
bash modules/lincoln/add_lincoln.sh --help
```

无位置参数时随机挑选未占用的 CARLA 出生点，避开距离其他车辆 7 米以内的位置；最后由 CARLA 的碰撞检查决定能否生成。可同时启动多份命令，每份只管理自己的车辆、传感器、驾驶面板和 RViz。

指定 **CARLA 世界坐标**（米）及 yaw（度）：

```bash
add_lincoln --pose 199.390472 -204.544281 0.519592 91.314423
```

也可以指定 CARLA 地图出生点编号：

```bash
add_lincoln --spawn-index 10
```

`--pose` 与 `--spawn-index` 互斥。显式坐标不会自动改写 z 或吸附到其他道路：立交场景的实际层级由坐标和碰撞几何决定。指定位置被占用、与地形碰撞或索引无效时会报错，不会默默换到随机位置。旧版坐标应在当前地图实际检查，不能保证任意历史坐标均可生成。

## 驾驶与自定义算法接口

默认手动驾驶，保留旧版鼠标面板、WASD、前进/倒车切换、定速、CARLA 跟随/自由视角。面板的“开启自动驾驶”使用默认 CARLA Traffic Manager。启动时直接开启：

```bash
add_lincoln --autopilot
```

手动按住油门按钮或驾驶模式下按 W 时，前进油门为 0.75，倒车保持 0.45；可在 `modules/lincoln/driving/manual_driving.py` 文件头调整两档油门。这里设置的是油门比例，实际加速度由车辆物理模型决定，定速与自动驾驶控制保持各自原有逻辑。

Lincoln 只打开一个综合面板，左侧为车辆驾驶区，速度及指令状态位于驾驶区底部，与右侧操作提示的最后一行底线对齐；两区随窗口高度伸展并保留相同的底部留白。右侧键盘提示保持单行显示。无需另行启动 `foundation/carla/mouse_move/`；后者的原代码保持不变，Lincoln 直接复用其控件布局、水平移动计算、视角预设和调速功能，不启动该工具自己的窗口或相机进程。

| 模式 | 键盘与面板行为 |
|---|---|
| 跟随本车 | WASD 驾驶，Q 切换前进/倒车；保留油门、刹车、转向、目标速度、右键定速及自动驾驶按钮。右侧自由视角区整体隐藏，窗口随之收窄。 |
| 点击“释放视角”后 | 右侧自由视角区展开；面板内 WASD 控制相机水平前后左右移动，Q 下降、E 上升，Shift 临时加速；不把这些按键发送为驾驶指令。车辆驾驶按钮及自动驾驶功能仍可使用，释放视角本身不关闭自动驾驶或定速。 |
| 点击“跟随本车”后 | 取消自由视角的临时操作及自动前进，恢复跟随和车辆键盘驾驶。 |

自由视角区提供左转、前进、右转、左移、后退、右移、升降、抬头低头、恢复水平、45° 看地面、速度滑块、加减速及停止。“贴靠 CARLA”位于左侧驾驶区右下角，在跟随和自由视角两种模式下均可使用。展开自由视角时，它与右侧速度减、速度加、停止三个按钮尺寸相同并处于同一水平行；右侧三个按钮等宽铺满滑轨下方。左侧速度及指令文字保持在驾驶区底部居中，不与贴靠按钮挤在同一行。前进按钮右键开启/取消持续前进；持续前进中转向、调速、切换视角预设和失焦不取消自动前进，后退、停止或切回车辆跟随会取消。任意俯仰角度下前后左右均水平移动，45° 俯视是一次性预设。

Lincoln 综合面板已移除“拖动视角”小块，转向使用左转、右转、抬头和低头按钮；左右转头速度为 180°/秒，抬头低头保持 45°/秒。独立自由视角面板保持原样。滚轮在综合面板内调整自由相机速度。键盘操作需要综合面板拥有焦点，输入目标速度时不拦截文字。取消原先直接捕获 CARLA 窗口键鼠、隐藏鼠标和回中逻辑，因此不再要求在 CARLA 窗口操作。临时按住操作在松开或面板失焦后清理；自由视角由 Lincoln 原有控制线程写入 spectator，不新增线程、进程或仿真时钟。自由位姿未改变时不反复写回相机。

默认连接 `127.0.0.1:2000`，Traffic Manager 首选端口为 `8000`：

```bash
add_lincoln --host 127.0.0.1 --port 2000 --tm-port 8000 --autopilot
```

每辆 Lincoln 启动时，在前台从首选端口起寻找空闲端口并创建自己的 TM 服务；同机多车通常依次使用 8000、8001 等，实际端口会打印到终端。这样退出一辆车不会带走另一辆车的 TM 服务。模块检查世界的同步模式变化，并向桥登记实际端口。当前自动驾驶同步要求 Lincoln 和 CoBridge 在同一主机运行；CARLA 服务本身可通过 `--host` 指定。

默认 CARLA 控制器的冷启动已从“开启自动驾驶”按钮移到车辆启动阶段。即使默认手动驾驶，启动时也会完成 TM 创建与地图准备，并提前向桥登记实际端口。桥运行时，车辆在显示驾驶面板或执行 `--autopilot` 前等待 `tm_ready`：桥已接入相同世界并完成该端口的连接。手动/自动切换不再拆除、重建这条 TM 连接，车辆退出时才释放。

这个调整复用已有登记线程，没有新增线程或模块。启动终端会显示“正在准备 Traffic Manager”以及“CoBridge 的 TM 连接已准备”。初始化成本仍存在，只是移到启动阶段；桥不存在时完成本地准备后继续独立运行，不会无限等待。桥已接入但准备失败时最多等待 15 秒并报告错误；旧桥没有 `tm_ready` 字段时明确要求重启 CoBridge，避免把尚未准备好的状态当作成功。

修改后需重新启动 CoBridge 和 Lincoln。初始化与自动驾驶启用耗时记录在 `.runtime/lincoln/lincoln.log`。本次真实 CARLA/SUMO、31 个传感器和 RViz 实测：车辆初始化及准备合计约 7.5 秒，首次开启自动驾驶约 0.3 ms，第二次约 8 ms，未观察到切换引起的数秒全局停顿。该数据不保证故障时网络 RPC 绝不等待。

复测的 LiDAR 发布间隔中位数约 100 ms、最长约 213 ms，仍有抖动。本次 TM 修正没有再调整 LiDAR 采样参数，也没有声称点云连续性已完全修复。

替换算法的接口位于 `modules/lincoln/driving/controller.py`。自定义类继承 `DrivingController`，实现：

| 方法 | 职责 |
|---|---|
| `enable()` | 接管本车控制，准备算法所需资源 |
| `disable()` | 释放本车控制，可重复调用 |
| `update(snapshot)` | 每个被观察到的新 CARLA 帧调用，返回 `carla.VehicleControl`；已由外部控制器施加控制时返回 `None` |
| `close()` | 退出时释放资源，默认调用 `disable()` |

构造参数是 `(client, world, vehicle, args)`。算法不可调用 `world.tick()` 或修改世界时钟，`update()` 应及时返回。未来可在此接口接入自有 Python 算法、ROS 2 控制订阅或其他控制后端，当前未新增自动驾驶算法或官方 ROS Bridge。

例如将自己的实现放入 `modules/lincoln/driving/my_driver.py`，类名为 `MyDriver`：

```bash
add_lincoln --driver modules.lincoln.driving.my_driver:MyDriver --autopilot
```

此命令是接口用法示例，仓库未附带 `my_driver.py`。切换回手动驾驶会关闭所选控制器对车辆的控制；不要让手动、TM 和自有算法同时施加控制。

## 传感器、ROS 2 与 RViz

从 Git `f511f80` 迁移原有 `sensors.json`、`vehicle_geometry.json`、Lincoln `.dae` 模型和 RViz 配置。默认保留 **31 个原生传感器 actor**：1 个 LiDAR、8 个相机、6 个角/侧向雷达，以及 16 个用于模拟超声波覆盖的短距雷达；后者仍是 CARLA radar，不是真实声学超声波仿真。

每辆车使用独立 `lincoln_<随机标识>` ROS 命名空间，发布 TF、图像、CameraInfo、PointCloud2、车辆模型、安装点与道路标记；RViz 模板在启动时按本车命名空间生成副本。ROS 的 Y 轴方向与 CARLA 相反，消息转换沿用旧实现。SUMO 名称从当前空闲编号 0 起独立分配为 `lincoln0`、`lincoln1` 等，删除后编号可复用，桥重启时优先恢复原名；不能将其直接当作 ROS 命名空间。

```bash
# 保留全部传感器，只关闭 RViz 窗口
add_lincoln --no-rviz

# 只生成 LiDAR，不打开面板和 RViz
add_lincoln --sensors lidar --headless --no-rviz

# 纯车辆连通性测试：显式关闭传感器、面板和 RViz
add_lincoln --sensors none --headless --no-rviz

# 运行 10 秒后自动清理（运行时长从初始化完成后计）
add_lincoln --headless --no-rviz --duration 10

# 使用自己的 RViz 布局
add_lincoln --rviz-config /完整路径/my_layout.rviz
```

`--headless` 只关闭驾驶面板，不隐式关闭传感器或 RViz。`--no-rviz` 时不构建无订阅者的道路背景；其余传感器仍按所选配置运行。传感器采样频率按世界步长量化，实际回调频率受仿真帧率影响，不代表一定达到对应现实频率。原始话题的相机与雷达队列只保留最新帧；LiDAR 原始队列保留最新一轮完整扫描，慢发布会丢弃过时数据。严格同帧显示另有最多 4 帧的有界缓存，见下文。运行中更改世界步长后应重新启动车辆，重新量化传感器配置。

启动脚本加载 `/opt/ros/jazzy/setup.bash`，默认使用 `$HOME/carla-venv/bin/python`。解释器可通过 `AUTODRIVE_PYTHON` 覆盖，ROS 环境脚本通过 `AUTODRIVE_ROS_SETUP` 覆盖。需要 CARLA API、ROS 2 Jazzy 的 rclpy/消息包/TF、RViz2、NumPy、OpenCV、PyYAML 和 Tk。使用 `--help` 无需加载 ROS。

## 行驶跟随与 LiDAR 点云保持

车辆 TF 改为在收到 CARLA 新帧后立即发布，与相机和点云转换分开；新帧线程不再额外等待 30 ms。车辆模型及传感器安装点启用 RViz `frame_locked`，随 TF 更新位置。默认 RViz 模板刷新率为 60；自定义模板仍按用户自己的刷新率运行。本次没有增加显示插值、预测或额外仿真时钟，实际场景卡顿仍受 CARLA 帧率影响。

为避免行驶时车模先前进、点云仍停留在上一帧，启用 LiDAR 后，RViz 车模和安装点改为绑定独立的 `<车辆命名空间>/lidar_display_base_link`。此显示 TF 只在发布选定 LiDAR 的完整点云时更新：用该帧的传感器世界位姿去除固定安装位姿，得到采集时刻的车辆位置和朝向，时间戳也与点云相同。两次点云发布之间，显示车模和跟随目标保持不动；真实 `base_link` 仍随 CARLA 新帧更新，用于传感器坐标变换及后续算法。默认 RViz 跟随目标自动切换到显示 TF；自定义配置中指定的其他跟随目标保留。多 LiDAR 时以配置中的第一个为显示基准。原始传感器话题始终保留各自采样时刻；同步模式下雷达显示改为下述严格同帧方式。没有 LiDAR（如 `--sensors none`）时仍使用实时车辆 TF。

该方式会让显示车模随点云按默认 10 Hz 更新，并带有采集到发布的延迟；它不提高仿真频率，也不保证多个 ROS 话题在 RViz 同一渲染帧原子更新。严格同帧显示会提高短距雷达原生采样频率，但不改变原始 ROS 话题配置频率。没有增加线程或历史位姿队列。正常重启 Lincoln 后生效。

重启后行驶验证：约 12 秒采样中，车辆起止位置相距约 150 米，匹配到 115 组时间戳一致的点云、显示 TF 和车模消息；显示位置与同一时刻真实车辆 TF 的最大差值约 0.07 毫米。订阅启动和结束边界、不同话题的投递可能造成计数不完全一致，该结果不代表所有消息均无丢失。用户观察确认车模压过旧点云的现象有改善；尚未确认所有视觉顿挫均已消失。

LiDAR 的采集转速现在匹配初始化时的物理步长：CoBridge 为 0.05 秒时，LiDAR 原生扫描频率为 20 Hz，每个物理帧覆盖完整前方 120°。ROS 输出频率仍使用 `sensors.json` 的 `rate=10 Hz`，每隔一个完整扫描发布一次，保留最新点云直到下一次替换。`Decay Time` 保持 0，不堆积数秒历史数据。

此前将两个相隔 0.05 秒的半视野拼成完整点云，虽然角度覆盖齐全，但车辆运动会让两段扫描环来自不同采样中心，在固定方向留下接缝。现在同步模式直接发布同一原生扫描帧，不再对两个半帧进行拼接或运动补偿。物理帧短于配置的单轮扫描周期（例如异步模式）时，才复用原有拼包和逐包运动补偿逻辑，转换到末包的传感器坐标系。异步运动扫描仍可能有采样接缝，不能等同于同步单帧采集。

64 线、120° 视野、100 米距离、每轮每线 720 个采样的配置不变；以 0.05 秒步长计算，原生 `rotation_frequency=20`、`points_per_second=921600`、`sensor_tick=0`。相对此前 10 Hz 分段采集，LiDAR 原生射线计算量约增加一倍；即使 ROS 只输出 10 Hz，也仍计算每个物理帧的完整扫描。这项计算开销已在修改前说明并经确认。没有增加线程、文件或新的历史点云缓存。

31 个传感器的 JSON 配置保持原样；相机采样设置不变，其他雷达的原生采样在严格同帧显示模式下按下述规则调整。日志中 LiDAR 接收次数按物理帧计，发布次数按 ROS 输出频率计。采集方式和频率在初始化时确定，改变世界步长后应重启 Lincoln。点云是离散激光命中，真实遮挡、未命中和点间距不会通过插值补成虚假数据。

同步单帧采集实测：重启后确认原生转速为 20 Hz、ROS 输出为 10 Hz，行驶中前方 24 个角度区间均有覆盖。另采集 6 帧、每帧约 3.4 万点，99% 点的扫描束俯仰角偏差小于 0.00035°，此前旧半帧约 0.05° 的成片偏差不再出现。该检查验证了跨帧采样中心造成的接缝已修正，不代表遮挡和离散采样形成的所有空隙都会消失，也不等于消除了仿真时间上的停顿。

修正后需正常退出并重启 Lincoln，使传感器与 RViz 配置重新加载；CARLA、SUMO、CoBridge 可以保持运行。这次点云修正只调整 Lincoln 文件及本文档；后续 TM 前置准备另在桥的 Lincoln 专属登记状态中增加 `tm_ready`，公共模块及桥主循环保持原样。两项修正均未新增工程文件。

本机异步模式、全传感器行驶采样：前方 120° 按 24 个角度区间统计，修正前点云覆盖中位数为 3 个区间，修正后最少和中位数均为 24 个；点数中位数约从 3,100 增至 45,700。TF 到达间隔中位数约从 20 ms 降至 9.5 ms。该结果是本次运行采样，不代表固定步长仿真会超过自身帧率，也不等同于已确认所有视觉卡顿消失。另已通过 0.05 秒同步步长下的双车自动驾驶、投影删除和桥重启恢复回归验证。

## 所有雷达与车模严格同帧显示

先启动 CoBridge，使 CARLA 采用同步模式，再启动 Lincoln。同步模式下自动启用，不增加启动参数。默认 0.05 秒物理步长下，6 个角/侧向雷达原生仍为 20 Hz，16 个短距雷达由原生 10 Hz 改为每物理帧采样（20 Hz）；此项额外采样开销已获确认。安装位置、视野、范围和 `points_per_second` 配置算法不变，未修改 `sensors.json`。原生回调频率变化会影响每包命中点数及计算负载，不能保证随机雷达每帧点数完全相同。

数据分为两个用途：

| 话题 | 用途与频率 |
|---|---|
| `/<车辆>/<传感器>/points` | 原始数据接口；角/侧雷达保持 20 Hz，短距雷达保持 10 Hz，LiDAR 保持 10 Hz。保留真实采集时间戳、原传感器坐标系、强度或速度字段，不将显示时钟用于算法数据。 |
| `/<车辆>/<传感器>/display_points` | RViz 专用；以 LiDAR 的完整单物理帧为基准，默认全部 23 个雷达显示均为 10 Hz。只选择相同 CARLA `frame` 且时间戳相同的数据，不插值、不重标时间、不用前后帧凑齐。 |

每个传感器回调使用最多 4 包的非阻塞显示队列，ROS 发布线程另保留最多 4 个候选物理帧。全部传感器到齐后，只发布最新完整帧，再更新同时间戳的显示车模、安装点和跟随 TF；晚到的旧帧不会让显示倒退。缺失任一雷达时保留上一组显示，丢弃过期候选，不暂停 CARLA、CoBridge 或原始数据发布。相机、真实 `base_link`、驾驶控制和两端时钟不参加这一显示等待。增加显示数据发布和有限缓存会消耗一定 CPU、内存及 DDS 带宽，不新增线程、进程或原生传感器 actor。

RViz 启动器只把本车已生成的雷达点云显示项切换到 `display_points`，保留每个传感器的独立开关和外观；用户模板本身不被改写，传感器 QoS 仍为 Best Effort。数据已按采集帧成组对齐，但多个 DDS 话题不构成原子渲染事务，网络丢包或 RViz 刷新边界仍可能短暂显示上一条消息。

启动 Lincoln 时若 CARLA 尚未进入同步模式，或未配置 LiDAR，则保留原来的独立显示方式。之后启动桥或改变世界步长、同步模式，需要正常重启 Lincoln，使原生参数和生成的 RViz 配置一起生效。异步拼接的多物理帧 LiDAR 不参与严格同帧显示；运行中退出同步模式可能让显示保持最后一组完整数据，原始话题继续输出。

实现复用 `modules/lincoln/sensors/sensor_streams.py` 的回调和发布线程，组件初始化位于 `runtime/lincoln_node.py`，RViz 话题转换位于 `visualization/rviz_launcher.py`，退出时由 `vehicle/vehicle_cleanup.py` 清空显示缓存。未添加新的工程文件或修改 CoBridge、官方代码。

重启实测：22 个原生 radar 的 `sensor_tick` 均为 0，RViz 的 23 个点云显示项全部使用 `display_points`。一轮行驶约 16.3 米的采样中，匹配到 84 组完整的 23 路雷达、显示 TF 和车模消息，采集时间戳一致；显示车模与同时间戳真实车辆 TF 的位置差最大约 0.03 毫米。原始话题间隔中位数分别为 LiDAR 100 ms、角雷达 50 ms、短距雷达 100 ms；各显示话题的现实到达间隔中位数约 103 ms，RViz 日志未出现警告或错误。部分显示消息未被诊断订阅者收到，不能据此宣称无丢包或原子渲染。用户确认其他雷达相对车身的拖延和跳动明显改善。7 项离线专项检查及原有 15 项回归通过。

## RViz 车轮滚动与前轮转向

RViz 将车身和四轮分别显示，原始 `modules/lincoln/assets/lincoln.dae` 不改写。`visualization/wheel_meshes.py` 从原模型的断开组件中提取轮胎、轮毂、轮栓和制动盘，生成去除这些可动组件的车身及四轮 DAE。固定制动钳保留在原车身网格，不额外模拟制动钳转向、车门或其他关节动画。模型的原材质、车漆和玻璃保持原样，不用简化圆柱代替原车轮。

生成文件缓存于 `.runtime/lincoln/wheel_meshes/<哈希>/`，包含 `body.dae`、四个车轮 DAE 和 `meshes.json`。哈希包含原模型、几何配置和生成器代码，多车共享文件锁和缓存。第一次需要生成，之后直接复用；删除缓存后会在下次启动时重建，清理前应退出使用这些资源的 Lincoln/RViz。

车辆启动时按骨骼名称定位四轮；有车模订阅者时，沿用现有控制线程，每个 CARLA 新帧只调用一次 `get_bone_relative_transforms()`，取得四轮实际滚动、前轮转向及悬挂位移。没有车模订阅者时跳过额外的骨骼 RPC。它不使用固定转速或按油门猜测轮速，前进、倒车、打滑和停车由 CARLA 原生骨骼姿态决定，也不调用任何车轮设置或物理控制接口。

轮姿态按物理帧保留最多 8 组，显示时优先取与雷达/车身相同帧的姿态；读取期间若世界帧已变化，则不把结果标为旧帧。缺少同帧轮姿态时保持上次姿态，不阻塞点云、车身或仿真时钟。四个轮子与车身使用相同显示 TF 和消息时间戳，不新增线程、时钟、JointState 节点或传感器。接口读取本身不是带服务端时间戳的原子事务，不宣称所有 RPC 边界下都能精确对齐；该动画只用于显示。

重启 Lincoln 即生效，CARLA、SUMO、CoBridge 无需重启。默认动画随车模约 10 Hz 更新，高速旋转可能出现采样导致的视觉倒转，不能用 RViz 的表观旋转速度判断真实轮速。

验证结果：7 项专项检查通过，覆盖原模型未改写、缓存复用、拆分前后三角形总数和索引完整性、四轮初始中心、实际骨骼姿态转换、跨帧数据拒绝、无订阅时跳过读取及缓存上限；原有 15 项回归通过。本机实测骨骼 RPC 中位数约 0.10 ms，10 秒采样得到 96 组完整四轮消息，四轮分别在 95 次相邻更新中发生姿态变化，车速约 8 m/s，RViz 无资源加载警告或错误。读接口有少量额外开销，整体性能仍取决于仿真负载。

## RViz 车道线背景

默认显示项现为 `CARLA lane markings`，仍订阅本车的 `/roads` 话题。只按当前 CARLA 地图绘制车道标线，不生成覆盖整条车道的实心路面，避免近似路面遮挡地面点云、重叠面片闪烁以及挡住其他车道的标线。用户自己的 RViz 模板名称不会被改写。

车道线改用沿道路横向展开的窄 `TRIANGLE_LIST` 标线带，保留原有中心位置、颜色、宽度、立交高度和约 3 厘米的显示抬高，虚线仍按示意的 3 米线段/3 米空隙绘制。窄带宽度来自标线定义（如 0.15 米），不是车道宽度。每个三角形显式指定颜色，使 RViz 不受场景灯光影响而将白线调暗；三角形正面统一朝上。道路背景只在车辆启动时生成一次，不随转弯重新裁剪。Town04 全图检查了 463 个车道分组、66,641 对相邻采样点，没有线段被距离连续性过滤跳过；移除实心面前后，58,009 段标线几何一致。后续将这 58,009 段转换为 116,018 个窄标线三角形，几何检查全部朝上；没有通过额外补线改变地图含义。地图未定义的标线、路缘/草地及虚线原本的空隙仍不补成实线。

移除实心面解决了闪烁，但剩余缺线来自原先 `LINE_LIST` 的相机朝向线带渲染：本机实际使用的 Ogre 1.12.10 会缓存该线带的顶点，相机跟随车辆改变位置和方向后，静态标线可能投影变窄或近乎消失。改用贴地标线几何后不再依赖这种相机朝向计算。相关实现可参见 [RViz LineListMarker](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/marker/markers/line_list_marker.cpp) 与 [Ogre BillboardChain](https://github.com/OGRECave/ogre/blob/v1.12.10/OgreMain/src/OgreBillboardChain.cpp)。本工程没有改动 ROS 或 Ogre 安装文件。

实际 RViz 已更新为窄标线带，用户确认车辆附近、弯道内侧及转弯时的缺线恢复正常。4 项专项检查通过，覆盖正反向车道宽度和朝向、虚线间隔、不连接断开的车道以及不虚构未知标线。

正常重启 Lincoln 后永久生效，无需重启 CARLA、SUMO 或 CoBridge。诊断期间通过临时道路 Marker 更新验证现有 RViz；旧车辆进程的道路发布缓存仍包含旧绘制方式，重新打开 RViz 前应重启 Lincoln，以加载新版背景。

## CARLA → CoBridge → SUMO

```text
add_lincoln
  → Lincoln 直接调用 CARLA API 创建实体和传感器
  → 向 CoBridge 的 socket 登记 actor ID、world ID 和 role_name
  → CoBridge 每帧读取 CARLA 当前快照
  → 桥端将车辆原点换算为车头中心，再转换为 SUMO 坐标和朝向
  → 通过现有 SUMO 通用接口创建/更新 SUMO 车辆
```

模块端不连接 SUMO、不上传可能过时的位置流。桥端代码全部放在 `foundation/cobridge/lincoln/`，现有桥主文件仅增加登记分发、帧同步和退出清理等接入点；SUMO TraCI 层、RSU 和官方 CARLA 工具不改写。

偏移取自本次转换路网的 `netOffset`。SUMO 的位置参考点是车头中心，CARLA 的参考点是 actor 原点；桥会处理包围盒位置和长度，不能直接把两端位置数值当作同一点比较。朝向为 `(CARLA yaw + 90) % 360`。SUMO 车辆由 CARLA 决定运动，禁用 SUMO 的自主换道和安全速度约束，每帧通过 `moveToXY` 更新，不把 SUMO 控制回传 CARLA。本轮不实现 SUMO NPC → CARLA、OMNeT++ 或立交上下层的显式车道 ID 匹配；红绿灯控制单独由 SUMO 同步到 CARLA，见上文。

| 场景 | 行为 |
|---|---|
| 桥或 SUMO 未启动 | CARLA 车辆、传感器和驾驶仍可使用；后台等待登记/同步 |
| 桥或 SUMO 后启动 | 自动接入后，按当前 CARLA 位置创建 SUMO 投影 |
| 一辆 Lincoln 正常退出 | 请求桥移除对应 SUMO 投影，清理自己的 CARLA actor/传感器和窗口 |
| 模块连接异常断开 | 桥移除对应登记和 SUMO 投影；SIGKILL 无法执行模块自身的 CARLA 清理 |
| CoBridge 正常退出 | 只清理 SUMO 投影，CARLA Lincoln 仍运行；恢复仿真器原时钟归属 |
| CoBridge 重启 | 模块重新登记，优先恢复原 `lincolnN` 名称，无冲突时保留 |
| SUMO 重启 | 匹配地图、步长后重新创建已登记车辆的投影 |
| CARLA 断开 | 停止更新并冻结已有 SUMO 投影，等待恢复；世界变化时校验 world ID，拒绝将旧 actor ID 用于新世界 |

桥不会覆盖已经存在的普通 SUMO 同名车辆；清理仅针对带 Lincoln 投影归属参数的车辆。桥正常退出不会清空 SUMO 普通车流。临时连接中断可能造成投影删除后再创建，不保证不中断；两端 RPC 不是原子事务。

## 同步自动驾驶为何需要 TM 时钟接入

CARLA 0.9.16 的远程 TM 句柄并不会把 `SynchronousTick()` 转发给其服务端（[官方实现](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/TrafficManagerRemote.cpp#L264)），因此仅在车辆进程设置 TM 同步模式、再由另一个进程调用 `world.tick()`，车辆可能保持静止。

本工程让 Lincoln 保留驾驶控制和 TM 服务，CoBridge 在每个 CARLA 帧后，通过官方 TM 服务端的 `synchronous_tick` RPC 驱动一次控制计算（[官方服务接口](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/TrafficManagerServer.h#L244)），再同步 SUMO。桥中的实现只支持同步模式切换和控制计算两种请求，使用固定范围的 MessagePack-RPC 编解码，不引入额外第三方包，不额外调用 `world.tick()`。

这是针对 CARLA 0.9.16 的接口适配，升级 CARLA 后应重新核对协议和行为。自定义算法默认不使用 TM，不走这条路径。桥不可用时，CARLA 恢复异步后，车辆控制器也恢复 TM 异步运行。

## Lincoln 文件分工

```text
modules/lincoln/
├── add_lincoln.py                  # 参数与应用入口
├── add_lincoln.sh                  # ROS/Python 环境与脚本入口
├── runtime/
│   ├── arguments.py                # 参数解析和校验
│   ├── application.py              # 工作线程、窗口与退出协调
│   ├── lincoln_node.py             # ROS Node 及功能组件组装
│   └── paths.py                    # 资源、运行产物路径
├── vehicle/
│   ├── spawn_vehicle.py            # CARLA 生成和出生点检查
│   └── vehicle_cleanup.py          # 控制器、流、actor 与原生引用清理
├── driving/
│   ├── controller.py               # 可替换算法接口与加载
│   ├── carla_autopilot.py          # CARLA Traffic Manager
│   ├── traffic_manager_port.py      # 多车 TM 端口分配与互斥
│   └── manual_driving.py           # 手动与定速驾驶
├── integration/bridge_registration.py # 向桥登记、状态查询与重连
├── sensors/
│   ├── sensor_streams.py           # 原生传感器与 ROS 发布
│   └── ros_messages.py             # 图像、点云、时间和坐标转换
├── visualization/
│   ├── vehicle_markers.py          # 车身、四轮动画与传感器安装点
│   ├── wheel_meshes.py             # 原车轮拆分与派生模型缓存
│   ├── road_map.py                 # CARLA 车道标线，不绘制实心路面
│   ├── spectator.py                # CARLA 跟随视角
│   ├── camera_ownership.py         # 多车 spectator 控制权
│   └── rviz_launcher.py            # RViz 配置和窗口
├── ui/
│   ├── driving_panel.py            # 综合面板及驾驶/相机键盘路由
│   └── free_camera_controls.py     # 内嵌自由视角，复用独立工具控件及算法
├── config/                         # 传感器、车辆几何与 RViz 模板
├── assets/lincoln.dae               # 原车辆模型
└── tests/                          # 登记、归属和错误隔离测试

foundation/cobridge/lincoln/
├── vehicle_registry.py             # 桥端登记、断线、逐帧同步和生命周期
├── vehicle_projection.py           # SUMO 类型、路线、坐标及投影操作
└── traffic_manager_clock.py        # 桥端同步驱动各车的 TM 控制计算
```

Lincoln 根目录仅保留两个启动文件；Python 自动生成的 `__pycache__` 为运行缓存。旧版 `ensure_sumo.py`、`shared_sumo.py` 和旧 `run_lincoln.sh` 不迁入，避免车辆重新持有共享桥的生命周期。

## 日志与检查

- `.runtime/lincoln/lincoln.log`：车辆运行、桥登记及退出时 31 个传感器的收发统计。统计不打印到终端。
- `.runtime/lincoln/<车辆命名空间>/lincoln.rviz`、`rviz.log`：本车生成的 RViz 配置和输出。
- `.runtime/lincoln/camera/`：同一 CARLA 端点的 spectator 归属锁与状态。
- `.runtime/cobridge/bridge.log`：桥接入和车辆投影同步错误。
- `.runtime/sumo/`：SUMO 运行层与底层仿真日志，沿用原路径。

离线回归测试：

```bash
~/carla-venv/bin/python -m unittest discover -s modules/lincoln/tests -v
```

更新代码后重新加载 `setup.bash`，并正常退出、重启 CoBridge 与 Lincoln，旧进程不会热加载新增协议。

本轮验证结果：

- 15 项离线回归通过，覆盖登记归属、断线、世界校验、自定义控制器接口、TM 协议拆包、端口占用，以及手动模式下的 TM 提前准备和桥端就绪确认。
- 在当前 Town04 实测单车默认 31 个传感器运行，统计中所有传感器均有收发记录；独立于桥运行和退出均通过。
- 两辆携带 LiDAR 的 Lincoln 在真实 CARLA/SUMO 联仿中生成并映射为 `lincoln0/1`；自动驾驶与位置同步通过，采样误差约 0～5 厘米（非同一原子时刻的两端查询）。
- 单车退出的双端删除、另一辆车保持运行、桥退出后保留 CARLA 车辆，以及桥重启后恢复同名投影均通过；收尾时恢复 CARLA 原设置。
- 默认鼠标面板、完整传感器和 RViz 的启动及定时退出通过。生成 RViz 临时配置时，将显示项的 `Value` 设为 `false`，并保留原来的 `Enabled`：RViz 先加载 Topic/QoS，再按 `Enabled` 启用，避免配置中的 `Value: true` 提前创建默认 Reliable 订阅。传感器话题仍使用 `Best Effort`，道路和车辆标记的可靠性设置保留；关闭的显示项不会被自动开启。无需修改 ROS 安装文件、用户模板或屏蔽日志，重启 Lincoln 后生成的新配置生效。
- 使用独立临时点云和图像发布端进行 RViz 启动对比：修改前触发 2 次 QoS 不兼容事件，修改后初始订阅即为 `BEST_EFFORT`，不兼容事件为 0。该结果验证本工程配置加载过程；其他手动创建的 Reliable 订阅仍会正常报告不兼容。

尚未进行长时间压力测试、多主机部署、自定义算法闭环或立交车道匹配验证。
