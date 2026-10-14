# Autodrive

CARLA 0.9.16、SUMO、ROS2/RViz2 与 OMNeT++/Veins 联仿工程。本次以 **`f511f80 — 更新 README`** 为功能基准，直接迁移代码并调整目录、导入和调用路径。

Lincoln 按要求从基准提交重新迁移，未保留此前重构新增的激光拼接、相机位置预测和 Traffic Manager 清理补丁。原有的传感器配置、跟随算法及基准限制一并保留。
所有示例默认从工程根目录执行；模块 `.sh` 入口也支持从其他工作目录调用。`how_to_run.md` 按要求保留，本 README 是完整调用说明。

## 1. 启动与退出

### CARLA 和地图

```bash
./carla/init/start_carla.sh
./carla/init/choose_town04.sh
```

等待 CARLA 服务就绪后再选图。现有脚本使用 `carlasim/carla:0.9.16`，打开 1280×720 的窗口。
切换地图前，应先退出车辆、NPC、RSU、OMNeT++，再停止共享桥。

### 完整共享联仿桥

```bash
./cobridge/run_cobridge.sh
```

根据 CARLA 当前地图导出 OpenDRIVE，调用官方转换工具生成本次 SUMO 路网，打开工程定制 SUMO GUI，并双向同步车辆。
入口等待桥就绪后返回终端，桥在后台常驻；重复调用复用现有桥。这一入口现在启动完整联仿，不再仅显示地图。

```bash
./cobridge/run_cobridge.sh --status
./cobridge/run_cobridge.sh --help
```

默认 CARLA/TM 同步模式、`0.05` 秒固定步长。每轮不足 50 毫秒时等待，运行较慢时仿真倍率会下降。
共享桥是唯一 `world.tick()` 推进者和常规 TraCI 调用者，不要另开同步时钟或官方同步脚本。
临时 `cobridge/start_sync.sh`、`cobridge/sync/` 已删除。

首次启动可指定下列选项，已有桥的设置不会被后来的车辆/NPC 客户端强行改变：

```bash
./cobridge/run_cobridge.sh --step-length 0.05 --tls-manager carla
# 不打开 SUMO GUI：
./cobridge/run_cobridge.sh --no-gui
```

以上是不同启动方式，改变参数前先停止旧桥。

### Lincoln：唯一车辆入口

```bash
./scenes/lincoln/run_lincoln.sh
```

无参数随机生成一辆 Lincoln，打开驾驶面板与 RViz，默认保留全部 31 个传感器。
指定两处不同的 CARLA 世界坐标，分别在两个终端运行：

```bash
./scenes/lincoln/run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
./scenes/lincoln/run_lincoln.sh --pose 235.597504 -172.815598 0.695892 180.326126
```

`--pose` 是 x/y/z（米）和 yaw（度）；固定位置被占用时报错，不会随机换位置。
随机模式先排除距现有车约 7 米以内的标准出生点。也支持 `--spawn-index 12`。

基准中的固定实验车位功能并入同一入口：

```bash
./scenes/lincoln/run_lincoln.sh --vanet-slot A
./scenes/lincoln/run_lincoln.sh --vanet-slot B
./scenes/lincoln/run_lincoln.sh --vanet-slot auto
```

三条是可选用法，读取 `vanet/town04.json` 并验证道路、占用和车位锁；该功能限定配置中的地图。
**不再提供 `vanet/run_lincoln.sh`。** 默认 Lincoln 仍会自动确保共享桥已启动，但推荐显式先启动桥以便查看状态。

### SUMO 随机车流

```bash
./sumo/traffic/run_npc.sh -n 200 --sumo-gui --step-length 0.05 --tls-manager carla
```

NPC 默认不联网。让当前批次参与通信：

```bash
./sumo/traffic/run_npc.sh -n 200 --sumo-gui --step-length 0.05 --tls-manager carla --vanet
```

每个终端管理一个独立批次，Ctrl+C 只删除本批（含等待入场车辆），桥和其他批次保留。异常退出的批次也会回收。
`-n` 是请求数量，SUMO 按道路空间逐步放入，不保证瞬间全部入场。`--filterv` 按正则筛选车型，`--safe` 排除部分车型；基准暂不支持行人生成。
纯 CARLA 的 `npc_traffic.py` 按要求不迁移，不保留第二套车流入口。

### OMNeT++ 与 RSU

首次使用或修改无线模型后编译：

```bash
./omnet/build.sh
./omnet/run_omnet.sh
```

OMNeT++ 模型、构建、启动会话和背景导出已迁入根目录 `omnet/`，不保留旧路径启动器。
共享配置为 `vanet/town04.json`，其中 `rsu_config` 引用 `scenes/rsu/config/town04.json` 的 RSU 位置列表；桥端协议和跨模块测试仍在 `vanet/`。

在 Qtenv 点击 Run/Fast/Express 后开始推进通信。Qtenv 暂停会阻塞共享仿真时钟，这是基准的锁步机制。
共享桥启动后，OMNeT++ 可以先于车辆运行；没有 RSU 时车辆仍能直连。

在独立终端添加 RSU：

```bash
./scenes/rsu/run_rsu.sh
```

Ctrl+C 删除 RSU，OMNeT++ 保持运行；再次执行重新创建。关闭 OMNeT++ 不删除独立 RSU。
RSU 同时出现在 CARLA、SUMO 和运行中的 OMNeT++，CARLA 塔身具有有效碰撞几何，可被雷达和激光探测。

### 正常退出

先退出车辆/NPC/RSU/OMNeT++ 客户端，再执行：

```bash
./cobridge/run_cobridge.sh --stop
```

桥退出时恢复 CARLA 异步模式。删除单辆车、单个 NPC 批次或 RSU 不停止桥。
关闭 SUMO GUI 可能结束共享仿真，因此它不是独立的查看器。换地图、关闭 CARLA 前同样按此顺序清理。

## 2. 目录与调用关系

```text
Autodrive/
├── carla/
│   ├── init/                  CARLA 启动、选图，保留原结构
│   └── carla_free_camera.py   手动启动的独立自由视角工具
├── cobridge/
│   ├── run_cobridge.sh        共享桥启动、查询、停止
│   ├── common/
│   │   ├── bridge_manager.py  桥启动、复用、查询和停止
│   │   ├── service.py         联仿主服务与循环
│   │   ├── synchronization.py 双向车辆、信号灯与时钟同步
│   │   ├── ipc.py             客户端与桥的本地请求及会话
│   │   ├── performance.py     各阶段耗时统计
│   │   └── paths.py           工程路径集中定义
│   ├── carla/
│   │   └── communication_view.py CARLA 通信覆盖层进程管理
│   ├── sumo/
│   │   ├── traci_client.py    SUMO/TraCI 操作
│   │   ├── npc_client.py      NPC 批次客户端与复用的参数解析
│   │   └── tools/            车型定义生成与分配工具
│   ├── omnet/
│   │   └── transport.py      节点快照、逐帧 ACK 与通信结果
│   ├── map/                   独立地图导出与预览
│   ├── carla-source-official/  官方目录，迁移未修改
│   └── runtime/               独立地图导出产物与日志
├── sumo/
│   ├── traffic/run_npc.sh     NPC 批次入口
│   └── gui/                   定制 GUI 构建与产物
├── scenes/
│   ├── lincoln/              车辆、传感器、驾驶面板、RViz、车位选择
│   ├── vanet_visual/         场景通信连线覆盖层
│   └── rsu/
│       ├── run_rsu.sh         RSU 唯一启动入口
│       ├── session.py         会话登记与退出清理
│       ├── manager.py         桥调用的 RSU 场景管理
│       ├── carla/             带碰撞的实体、塔标记与名称显示
│       ├── sumo/rsu.png       SUMO POI 图标
│       └── config/town04.json RSU 位置列表
├── omnet/
│   ├── build.sh、run_omnet.sh   OMNeT++ 构建和唯一启动入口
│   ├── runtime/               OMNeT++ 会话管理
│   ├── tools/                 静态道路背景导出
│   ├── Vanet.cc、Vanet.ned、RangePhy.cc、RadioRange.h
│   ├── omnetpp.ini、radio.xml、antenna.xml、roads.txt
│   └── results/               实验输出，迁移保留原结果
├── vanet/
│   ├── town04.json            RSU 配置引用、车位、通信范围与端口
│   ├── common/config.py       配置读取与校验
│   ├── tools/、tests/、validation/
│   └── README.md
├── README.md
└── how_to_run.md
```

1. Lincoln 的鼠标/WASD/自动驾驶指令直接发给 CARLA。
2. 共享桥把 CARLA 控制的车同步到 SUMO，把 SUMO NPC 同步到 CARLA；双向映射防止重复镜像。
3. CARLA 传感器回调通过 Lincoln 最新帧队列和独立 ROS 发布线程交给 RViz；RViz 道路也来自 CARLA 当前地图。
4. 桥向 OMNeT++ 发送节点位置快照，等待 ACK 后推进下一步。OMNeT++ 不另起 SUMO，不另连 TraCI。
5. RSU 会话向桥登记，桥调用 `scenes/rsu/manager.py`，协调 CARLA 实体、SUMO POI 和通信节点。
6. OMNeT++ 返回真实握手/业务事件，覆盖层将连线画在 CARLA 主窗口，不进入传感器数据。

场景对象和场景显示统一归入 `scenes/`：Lincoln 使用 `scenes/lincoln/`（目录名统一小写），RSU 使用 `scenes/rsu/`，通信覆盖层使用 `scenes/vanet_visual/`。车辆网格、传感器配置、RViz 模板以及已有 `runtime/` 日志随 Lincoln 目录移动；共享桥从 `cobridge/common/paths.py` 获取新路径。`carla/` 保留 CARLA 启动、选图及独立自由视角工具。

旧 `scene/`、`carla/Lincoln/`、`carla/vanet_visual/` 不保留兼容入口。迁移前已启动的车辆、RSU、覆盖层和共享桥不会自动加载新代码；正常退出客户端并停止旧桥后，从本文的新路径重新启动。

### 代码复用与官方边界

`cobridge/carla-source-official/Co-Simulation/Sumo/` 文件保持不变。
旧版带工程修改的 `spawn_npc_sumo.py` 拆为 `cobridge/common/service.py` 与 `cobridge/sumo/npc_client.py`；主服务负责联仿循环，NPC 客户端负责批次请求和退出清理。原 `run_synchronization.py` 改为 `common/synchronization.py`，原 `sumo_simulation.py` 改为 `sumo/traci_client.py`。保留原许可证和同步逻辑，调用官方未修改的转换器、CARLA 包装器、常量及资源。
辅助车型工具分别为 `cobridge/sumo/tools/create_vehicle_types.py` 和 `assign_vehicle_types.py`，以工程副本保留。
后续直接修改官方目录必须先确认。官方原始 NPC 脚本不具备本工程共享会话与可选入网功能，请使用项目入口。

`common/bridge_manager.py` 复用旧桥管理实现，`common/ipc.py` 提供原 `shared_sumo.py` 的跨模块请求与会话功能。原 `vanet/runtime/` 中的 TCP 传输、CARLA 显示进程管理和性能统计分别归入 `cobridge/omnet/transport.py`、`cobridge/carla/communication_view.py` 与 `cobridge/common/performance.py`。
NPC shell 只定位实现，不复制车流算法。没有保留旧根目录入口、重复 VANET 车辆入口或空壳路径兼容模块。

`cobridge/` 根目录的程序入口只有 `run_cobridge.sh`，共享 Python 实现在 `common/`，软件接口按服务对象分别放入 `carla/`、`sumo/`、`omnet/`。管理器启动主服务；主服务调用同步器、SUMO 接口、OMNeT++ 传输以及 `scenes/rsu/manager.py`。NPC 客户端不另连 TraCI、不另起仿真时钟，通过 IPC 提交批次；默认不联网，`--vanet` 为该批车辆开启联网。NPC 客户端和主服务复用参数解析，避免两份参数定义发生偏差。

迁移后，正在运行的桥仍使用已加载的旧模块：先正常退出客户端，再执行 `./cobridge/run_cobridge.sh --stop`，随后重新启动桥和客户端。管理器能识别旧桥进程以完成正常清理，不保留旧启动文件。车辆、NPC、RSU 和 OMNeT++ 的 shell 命令保持现有用法。

## 3. Lincoln 功能与设置

| 参数 | 功能 |
|---|---|
| `--pose X Y Z YAW` / `--spawn-index N` | CARLA 坐标或标准出生点 |
| `--vanet-slot auto\|A\|B` | 配置中的固定实验车位 |
| `--sensors all\|lidar` | 默认全部传感器，lidar 供性能对照 |
| `--no-rviz` | 不打开 RViz，仍采集和发布 |
| `--headless --duration 10` | 无面板、定时退出 |
| `--camera-rows 4\|6` | 两列拼图，实际均为八路相机 |
| `--rviz-config PATH` | 自定义布局模板 |
| `--host / --port / --tm-port` | 服务地址和 TM 端口 |
| `--no-sumo` | 跳过自动确保桥就绪，供已有外部时钟使用 |
| `--sumo-npcs N` | 首次创建桥时附加默认不联网的常驻 NPC 批次 |

鼠标按住油门/刹车/转向，WASD 驾驶，低速 Q 换挡，松开油门滑行。油门右键启停定速，刹车接管；定速不是 ACC，没有前车自动避让。自动驾驶由 CARLA Traffic Manager 实现。

跟随视角保持车后 8 米、上方 4 米、俯角 -15°，只对朝向做 0.10 秒时间常数的平滑，独立按仿真帧更新。
跨进程所有权保证新车不自动抢镜头，显式点击“跟随本车”接管。
自由视角支持 WASD、Q/E、Shift 加速、左键转向、滚轮调速、F 捕获、Esc 释放、R 复位；CARLA 窗口控制依赖 X11。

独立空地图观察工具：

```bash
~/carla-venv/bin/python carla/carla_free_camera.py
```

它沿用基准实现，不接入多车相机锁，不要与其他跟随控制同时使用。

`scenes/lincoln/config/sensors.json` 定义 8 路 640×360 相机、1 个 64 线激光、6 个中距雷达、16 个用近距 radar 模拟的超声节点。
相机/超声等通常 10 Hz，中距雷达 20 Hz，实际频率受步长与负载影响。
RViz 包括地图、车辆模型、rig 分组点云和八路拼图，传感器采用 Best Effort。话题和 TF 以每车 `lincoln_<uid>` 隔离，ROS 对 CARLA 的 y/yaw 做符号转换；出生参数填写 CARLA 坐标。
RViz 的车辆模型与传感器安装标记启用 `frame_locked`，在每次显示更新时跟随当前 TF；避免模型仅在收到 Marker 消息时定位，而跟随镜头已使用更新后的 TF，造成相对跳动。该显示修正保留全部 31 个传感器、原采样频率、点云时间戳和 0.05 秒仿真步长；物理姿态仍按实际仿真帧更新。已运行的 Lincoln 需要退出后重新启动，才能加载新的标记设置。
车辆动态 TF 和模型标记由已有的仿真帧线程在收到新 CARLA 快照时发布，不再由传感器发布线程的 20 毫秒定时器轮询。这样避免 50 毫秒仿真帧被轮询成 40/60 毫秒交替发布，并减少点云转换或相机拼图阻塞车辆姿态更新的影响。传感器消息仍由原 ROS 发布线程处理，不新增仿真时钟，也不对物理姿态插值或预测。
RViz 默认按 60 Hz 刷新，并跟随每车的 `visual_base_link`：`scenes/lincoln/display_pose.py` 将相邻真实快照的显示位置和朝向插值，在单独的显示线程中发布显示 TF。显示比物理姿态晚一个仿真步（默认约 50 毫秒仿真时间），不会预测超过最后一个真实快照；仿真暂停时会停在最后一个已知姿态。车辆模型和安装标记使用该显示坐标系，传感器消息及其 `base_link` TF 保留真实时间戳和姿态。使用自定义 RViz 模板时，将跟随视角的 Target Frame 设为本车 `visual_base_link` 才会使用平滑显示；选择 `base_link` 可观察原始逐帧姿态。此功能不修改 CARLA/SUMO 时钟、车辆控制或 31 个传感器配置。 显示时钟连续推进，按约 0.25 秒的平滑时间缓慢校正到达偏差，每次校正限制在本次推进量的 10% 内，避免收到新快照时重新对时造成忽快忽慢；输入帧中断时仍不超出最后一个真实姿态。

不启动仿真即可检查显示插值、转向角跨界、暂停和时间重置：

```bash
~/carla-venv/bin/python scenes/lincoln/test_display_pose.py
```
本次恢复基准的传感器和着色实现，重构期间新增的激光拼接、地面颜色修改不包含在内。

## 4. RSU、通信和坐标

RSU 的代码、图标和位置定义集中在 `scenes/rsu/`。修改 [scenes/rsu/config/town04.json](scenes/rsu/config/town04.json) 的 `rsus` 列表：复制一项，设唯一 `rsu1` 等 ID，修改 `x/y/z/base_z`；退出 RSU 命令后重新启动即可，无需重启 OMNeT++。
坐标使用 CARLA 世界坐标，`z` 是顶部高度，`base_z` 是塔脚高度。配置详解见 [vanet/README.md](vanet/README.md)。

`vanet/town04.json` 的 `rsu_config` 按该配置文件所在目录解析，默认值为 `../scenes/rsu/config/town04.json`。统一加载器读取并校验 RSU 列表，再交给 CARLA、SUMO 和 OMNeT++；列表只维护一份。自定义 `--vanet-config PATH` 可以引用其他同地图的 RSU 配置，也可以提供完整的内嵌 `rsus` 列表，但不能同时使用两种定义。

从旧目录升级后，先正常退出车辆、NPC、RSU 和 OMNeT++，执行 `./cobridge/run_cobridge.sh --stop`，再启动共享桥和客户端。已运行的桥不会自动重新加载迁移后的 Python 模块。旧 `carla/rsu/` 和 `sumo/gui/assets/rsu.png` 不保留兼容入口。

默认车车 40 米、车与 RSU 220 米，通过 `communication.vehicle_range`、`communication.rsu_range` 统一调整，也支持单个节点例外。超范围不连通，V2V 与 RSU 转发可以共存。

网络为 **OMNeT++ 6.1 + Veins 5.3.1 / 802.11p，不使用 INET**。
HELLO/ACK 确认双向邻居；RSU 转发车辆原始 probe。基准实现广播连通性，不含车辆多跳、跨 RSU 单播路由、IP 接入或切换协议。
握手连线灰色，发送半段蓝色、接收半段黄色，中点分界；双向业务绿色。半透明细线由 Qt/X11 覆盖层显示，不生成传感器可见几何；RSU 实体塔身则能被探测。

SUMO 专用 RSU 图标位于 `scenes/rsu/sumo/rsu.png`，由 RSU 场景管理器创建 POI 时引用。
SUMO GUI 同时显示 SUMO/CARLA 鼠标坐标、右键复制 CARLA 坐标和底部仿真/现实时间倍率。
鼠标坐标从已载入路网的 netOffset 转换；车辆同步另包含车长参考点、y 翻转及 yaw 转换。
VANET 的配置 offset 和静态 Town04 背景沿用基准，OMNeT++ 道路从官方示例 Town04 路网导出到 `omnet/roads.txt`，不代表任意地图实时背景。

## 5. 独立地图功能与依赖

原来的地图导出/预览保留为显式选项，不承担联仿：

```bash
./cobridge/run_cobridge.sh --map-only --export-only
./cobridge/run_cobridge.sh --map-only
```

第一条仅导出，第二条打开独立预览。文件在 `cobridge/runtime/<地图名>/`。

| 依赖 | 默认位置 |
|---|---|
| CARLA Python | `~/carla-venv/bin/python` |
| ROS2/RViz2 | `/opt/ros/jazzy/setup.bash` |
| SUMO | `~/sumo-1.22.0`，可设 `SUMO_HOME` |
| OMNeT++ | `~/omnetpp-6.1`，可设 `OMNET_ROOT` |
| Veins | `~/veins-5.3.1`，可设 `VEINS_ROOT` |

需 NumPy、OpenCV、PyYAML、Tkinter、lxml、ROS2 消息/TF 和 Qt/X11。
定制 GUI 复用 SUMO 的 CMake 构建库，可用 `SUMO_BUILD_DIR` 指定；产物只放工程内 `sumo/gui/`，不覆盖系统 SUMO。

```bash
python3 sumo/gui/build.py
./omnet/run_omnet.sh -u Cmdenv
```

大场景建议 Cmdenv 或 Qtenv Express；普通逐包动画、完整事件日志会明显增加开销。保留基准性能优化不代表承诺 200/500 辆车实时运行。

## 6. 日志和排查

桥启动时打印 PID 与日志路径。查询本机默认地址的运行目录：

```bash
~/carla-venv/bin/python - <<'PY'
from cobridge.common.ipc import control_folder
print(control_folder('127.0.0.1', 2000))
PY
```

目录位于 `/tmp/carla-lincoln-<地址哈希>/`，沿用基准 IPC 命名，但源码和生命周期已由 cobridge 管理。
`sumo.log` 是桥日志，`bridge.json`、`ready.json` 和请求文件是状态，运行时不要删除。
RViz 实例配置在 `/tmp/lincoln_<uid>.rviz`，源模板在 `scenes/lincoln/rviz/`。

Lincoln 退出时的 31 个传感器统计写入 `scenes/lincoln/runtime/lincoln_<uid>.log`，
不再整段打印到终端。每辆车独立日志，每行一个 JSON 记录，包含退出统计时间戳、
车辆角色名和 `counts`；各传感器的数组依次为 `[收到的帧数, 向 ROS 发布的帧数]`。
目录自动创建并由 Git 忽略。若写入失败，仅提示一条警告，继续清理车辆。

```bash
ls -lt scenes/lincoln/runtime/
tail -n 1 scenes/lincoln/runtime/lincoln_实际uid.log
```

OMNeT++ 结果在 `omnet/results/`，默认关闭完整 `.elog` 事件日志。
先关闭 OMNeT++，再查看或删除大日志：

```bash
du -sh omnet/results
find omnet/results -type f -name '*.elog' -print
find omnet/results -type f -name '*.elog' -delete
```

CSV、`.sca/.vec` 是实验数据，按需备份后再删。桥停止后可清理对应 `sumo.log`，不要在运行中清空 IPC 目录。

查询映射、批次和入场车辆：

```bash
./cobridge/run_cobridge.sh --status
```

ROS 检查前加载环境：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
# 将 uid 换成实际车辆角色名
ros2 topic hz /lincoln_uid/roof_sensor_rig/lidar/points
```

保持唯一时钟，不用第二个 TraCI/tick 诊断程序干扰运行。Qtenv 暂停会暂停联仿，不应把它误判为 CARLA 卡死。

## 7. 本次迁移验证

- OMNeT++/Veins 和定制 SUMO GUI 编译通过。
- 基准自带的 TCP 等待、无线范围、V2V/RSU 转发、多/动态 RSU、NPC 入网、握手/覆盖层、交通灯缓存和场景生命周期测试通过。
- 真实 Town04：两辆 Lincoln、62 个传感器、独立实验车位，映射为 `lincoln0/lincoln1`。
- 两个独立 NPC 批次，实际通信节点只包含显式 `--vanet` 的批次。
- 真实 RSU 实体、SUMO 标记、OMNeT++ 握手和覆盖层状态反馈；删除 RSU 后网络和共享时钟继续运行。
- 真实雷达与语义激光命中塔身，测试实体清理无残留；车辆/NPC 退出后桥仍运行。
- 官方文件逐项校验保持不变。

没有声称全传感器严格同步、无卡顿或大量车辆固定实时倍率；基准限制仍保留。
部分回归命令：

```bash
~/carla-venv/bin/python vanet/tests/test_vehicle_sync.py
~/carla-venv/bin/python vanet/tests/test_radio_range.py
~/carla-venv/bin/python vanet/tests/test_dynamic_rsu.py
~/carla-venv/bin/python vanet/tests/test_communication.py
# 需已运行 CARLA/共享桥且未创建 RSU：
~/carla-venv/bin/python vanet/tests/test_rsu_sensors.py
```
