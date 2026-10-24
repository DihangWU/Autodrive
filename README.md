# Autodrive

CARLA、SUMO 和 CoBridge 分别运行。CARLA 负责三维场景、物理和传感器；SUMO 负责路网与交通仿真；CoBridge 负责时间推进和已登记 Lincoln 的 CARLA → SUMO 同步。RSU 保留现有独立进程，生命周期由它自己管理。详细基础架构见 [architecture.md](architecture.md)。

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

默认连接 `127.0.0.1:2000`，Traffic Manager 首选端口为 `8000`：

```bash
add_lincoln --host 127.0.0.1 --port 2000 --tm-port 8000 --autopilot
```

每辆 Lincoln 首次启用时，在调用线程从首选端口起寻找空闲端口并创建自己的 TM 服务；同机多车通常依次使用 8000、8001 等，实际端口会打印到终端。这样退出一辆车不会带走另一辆车的 TM 服务。模块检查世界的同步模式变化，并向桥登记实际端口。当前自动驾驶同步要求 Lincoln 和 CoBridge 在同一主机运行；CARLA 服务本身可通过 `--host` 指定。

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

`--headless` 只关闭驾驶面板，不隐式关闭传感器或 RViz。`--no-rviz` 时不构建无订阅者的道路背景；其余传感器仍按所选配置运行。传感器采样频率按世界步长量化，实际回调频率受仿真帧率影响，不代表一定达到对应现实频率。短期队列只保留最新帧，慢发布会丢弃过时帧。运行中更改世界步长后应重新启动车辆，重新量化传感器配置。

启动脚本加载 `/opt/ros/jazzy/setup.bash`，默认使用 `$HOME/carla-venv/bin/python`。解释器可通过 `AUTODRIVE_PYTHON` 覆盖，ROS 环境脚本通过 `AUTODRIVE_ROS_SETUP` 覆盖。需要 CARLA API、ROS 2 Jazzy 的 rclpy/消息包/TF、RViz2、NumPy、OpenCV、PyYAML 和 Tk。使用 `--help` 无需加载 ROS。

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

偏移取自本次转换路网的 `netOffset`。SUMO 的位置参考点是车头中心，CARLA 的参考点是 actor 原点；桥会处理包围盒位置和长度，不能直接把两端位置数值当作同一点比较。朝向为 `(CARLA yaw + 90) % 360`。SUMO 车辆由 CARLA 决定运动，禁用 SUMO 的自主换道和安全速度约束，每帧通过 `moveToXY` 更新，不把 SUMO 控制回传 CARLA。本轮不实现 SUMO NPC → CARLA、交通灯同步、OMNeT++ 或立交上下层的显式车道 ID 匹配。

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
│   ├── vehicle_markers.py          # 模型与传感器安装点
│   ├── road_map.py                 # CARLA 道路背景
│   ├── spectator.py                # CARLA 跟随视角
│   ├── camera_ownership.py         # 多车 spectator 控制权
│   └── rviz_launcher.py            # RViz 配置和窗口
├── ui/
│   ├── driving_panel.py            # 驾驶面板
│   ├── free_camera_controls.py     # 面板自由视角
│   └── carla_window_input.py       # CARLA 窗口输入支持
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

- 13 项离线回归通过，覆盖登记归属、断线、世界校验、自定义控制器接口、TM 协议拆包及端口占用。
- 在当前 Town04 实测单车默认 31 个传感器运行，统计中所有传感器均有收发记录；独立于桥运行和退出均通过。
- 两辆携带 LiDAR 的 Lincoln 在真实 CARLA/SUMO 联仿中生成并映射为 `lincoln0/1`；自动驾驶与位置同步通过，采样误差约 0～5 厘米（非同一原子时刻的两端查询）。
- 单车退出的双端删除、另一辆车保持运行、桥退出后保留 CARLA 车辆，以及桥重启后恢复同名投影均通过；收尾时恢复 CARLA 原设置。
- 默认鼠标面板、完整传感器和 RViz 的启动及定时退出通过；ROS 图查询确认 RViz 的 LiDAR 最终订阅为 `BEST_EFFORT`。RViz 启动切换 QoS 时可能短暂打印兼容性警告，不代表最终订阅配置仍不兼容。

尚未进行长时间压力测试、多主机部署、自定义算法闭环或立交车道匹配验证。
