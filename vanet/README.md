# Town04 Lincoln VANET 连通性实验

本目录使用本机 OMNeT++ 6.1 + Veins 5.3.1，不依赖 INET。OBU 是 Veins 802.11p 通信节点，不是 CARLA sensor。原有 SUMO 共享桥仍是唯一仿真推进者和 TraCI 调用者。

## 目录结构

常改配置和常用入口保留在 `vanet/` 顶层，Python 实现按功能归类：

```text
vanet/
├── README.md                 # 本目录说明
├── town04.json               # 出生点、RSU、统一通信半径与网络配置
├── run_lincoln.sh            # 车辆启动入口
├── run_omnet.sh              # OMNeT++ 启动入口（不自动创建 RSU）
├── run_rsu.sh                # 独立 RSU 创建/清理入口
├── build.sh                  # OMNeT++ 编译及道路导出入口
├── runtime/                  # 联仿运行逻辑
│   ├── launch.py             # 车辆启动与参数处理
│   ├── spawn.py              # 出生点选择与预留
│   ├── bridge.py             # 全部车辆的 TCP 快照、通信反馈及逐帧 ACK
│   ├── performance.py        # 可选的分段耗时记录
│   ├── omnet_session.py      # OMNeT++ 会话生命周期
│   └── rsu_session.py        # 独立 RSU 会话生命周期
├── scene/                    # RSU 场景与通信显示
│   ├── markers.py            # SUMO/CARLA 标记创建与清理
│   ├── communication.py      # 通信覆盖层启动、状态与清理
│   ├── communication_overlay.py # CARLA 主窗口透明连线
│   ├── rsu_marker.py         # CARLA 彩色高塔标识
│   ├── rsu_label.py          # 巨大洋红色 RSU 名称
│   └── rsu_body.py           # CARLA 实体塔身与碰撞几何
├── common/                   # 公共配置与资源路径
│   ├── config.py             # 统一配置校验、半径与单节点覆盖参数
│   └── paths.py              # 工程、配置、图片等路径
├── sumo_gui/                 # SUMO 双坐标显示与复制菜单
│   ├── build.py              # 工程内 GUI 构建
│   ├── build/                # 临时编译产物，Git 忽略
│   └── bin/                  # 编译后的 GUI，Git 忽略
├── omnet/                    # NED/C++ 模型、无线配置、道路背景
│   ├── RangePhy.cc           # 越界无线信号阻断
│   └── RadioRange.h          # V2V / RSU 共用半径规则
├── tools/                    # 导出与诊断工具
│   ├── export_roads.py
│   └── diagnose_carla.py
├── tests/                    # 通信、多 RSU、场景生命周期测试
├── assets/                   # RSU 图片等静态资源
└── validation/               # 已保存的验证记录
```

顶层的 `__init__.py` 和 `.gitignore` 用于 Python 包和 Git 管理。配置仍只需修改 `town04.json`；公共路径由 `common/paths.py` 按文件位置解析，不依赖终端当前目录。本文相对命令均从工程根目录执行，原有入口不变：

```bash
bash vanet/run_lincoln.sh
bash vanet/run_omnet.sh
bash vanet/run_rsu.sh   # 可选；不运行时只观察车辆通信
bash vanet/build.sh
```

工具可单独运行，例如 `python3 vanet/tools/export_roads.py`。`~/carla-venv/bin/python vanet/tools/diagnose_carla.py` 会独占 CARLA 时钟，仅用于已停止共享桥的隔离诊断，不与日常联仿同时运行。测试命令见下方“验证”。

## 当前状态

早期未设置硬通信半径时，已通过两辆真实 Lincoln + 一个 RSU 的 CARLA / SUMO / Veins 联调，每辆保留原有 31 个传感器。SUMO 车辆和 OMNeT++ OBU 统一命名为 `lincoln0`、`lincoln1`。两车在不同入口，实测间距约 48.1 米；5 秒仿真内，两车双向直达各成功接收 5 包，RSU 分别收到两车各 5 包，两车也均收到 RSU 广播及转发。结果保存在 `validation/live-smoke.json`。Qtenv 已检查实际收包气泡和通信连线。

无线独立测试、真实 TCP 握手与合成快照中的节点生成/销毁测试也已通过。移动过程、真实车辆销毁/重生后的完整联调和大规模指标分析仍待后续验证。

## 根目录车辆启动入口

随机生成一辆：

```bash
bash run_lincoln.sh
```

在 CARLA 世界坐标指定位置生成一辆（位置单位米，yaw 单位度）：

```bash
bash run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
```

另一实验车位：

```bash
bash run_lincoln.sh --pose 235.597504 -172.815598 0.695892 180.326126
```

这些坐标用于 Town04。固定点被占用时会报错，不随机换点。坐标参数不完整、非有限数字或与 --spawn-index 冲突，会在启动共享桥前报错。

## 路口与位置

SUMO 状态栏在原 x/y 位置同时显示 `SUMO x/y | CARLA x/y`。右键菜单的 `Copy SUMO coordinates` 和 `Copy CARLA coordinates` 分别复制右键落点的两种坐标，格式为 `x, y`，保留六位小数。高度 z 和朝向 yaw 需另行确定。

显示与复制直接使用当前加载路网的 `netOffset` 换算：`CARLA.x = SUMO.x - offset.x`、`CARLA.y = offset.y - SUMO.y`。此功能仅增加界面文字和菜单，不翻转地图，也不改变路网、车辆或 OMNeT++ 的坐标逻辑。SUMO 的鼠标落点与车辆中心、车头参考点是不同概念。

实现放在 `vanet/sumo_gui/`，生成本工程专用的 `bin/sumo-gui`，不修改系统 SUMO。首次启动带 GUI 的共享桥时自动构建并启用，也可预先执行 `python3 vanet/sumo_gui/build.py`；复用本机 SUMO 1.22.0 源码和 CMake 编译库，可用 `SUMO_HOME` / `SUMO_BUILD_DIR` 指定目录。已运行的旧窗口需先正常退出车辆及 OMNeT++，执行 `bash run_lincoln.sh --stop-sumo`，再重新启动车辆后生效。

候选为工程示例 Town04 路网的十字路口 **924**。四条入口为 -17.0.00、10.0.00、18.0.00、-9.0.00，入口长度约 36–69 米，适合小范围相遇测试。车位沿两个垂直入口向上游取 25 米。

| 对象 | CARLA x / y（米） | yaw（度） |
|---|---|---|
| Lincoln A（车道修正后） | 199.390 / -204.544 | 91.314 |
| Lincoln B（车道修正后） | 235.598 / -172.816 | 180.326 |
| RSU（对角路侧草地、红砖墙前） | 193.000 / -179.000 | 固定 |

配置种子坐标集中在 town04.json。车辆启动时使用当前 CARLA 驾驶车道 waypoint 修正高度和方向；检查横向偏差不超过 4 米、方向差不超过 35 度且不在路口内部，否则停止。RSU 在 OMNeT++ 中是固定无线节点；CARLA 同位置显示橙色塔架、青色顶部框和圆环、黄色天线及巨大洋红色 RSU 名称（如 `rsu0`）。彩色标识仍使用世界调试绘图；同位置另有带碰撞的实体塔身，供 LiDAR / Radar 探测。当前基座已核对位于原路口对角的草地、红砖墙前；四个城区道路采样点到塔顶的视线检测无障碍，约 211 米外实景可见顶部标识。

RSU 的唯一位置定义是 `town04.json` 的 `rsus` 列表，单位为 CARLA 世界坐标米。独立命令 `bash vanet/run_rsu.sh` 每次启动读取 JSON，将配置快照交给共享桥：CARLA 创建实体塔身和标识，SUMO 显示图标，已运行的 OMNeT++ 在下一帧创建对应无线节点。单独启动车辆或 OMNeT++ 都不会生成 RSU。

在 RSU 命令终端按 Ctrl+C，实体和 SUMO 图标由共享桥移除，OMNeT++ 在下一帧删除 RSU 节点及连线，继续车辆之间的通信。关闭 OMNeT++ 不会关闭独立 RSU 命令；之后重新打开 OMNeT++ 也会接入仍在运行的 RSU。绘图在恢复推进后的两个步长内消失（默认约 0.1 仿真秒）。Qtenv 暂停时可执行 RSU 创建/清理，网络节点和绘图的仿真更新需恢复 Run/Fast 后显示。修改 JSON 后重启 RSU 命令即可，OMNeT++ 可以保持打开。

所有实体和图标操作仍由现有桥线程执行，不增加 TraCI 客户端或 CARLA 时钟拥有者；RSU 命令终止或崩溃也会触发清理。首次升级独立 RSU 功能时，需要正常关闭 OMNeT++、RSU 命令和车辆，执行 `bash run_lincoln.sh --stop-sumo`，再重新启动车辆及 OMNeT++，使共享桥加载新协议。无需重启 CARLA 或修改地图。

桥传输的 SUMO 坐标使用 x=CARLA.x+503.02、y=-CARLA.y+423.76。OMNeT++ 内统一转换为 x'=x-xMin+40、y'=yMax-y+40，修正 Qtenv 画布 Y 轴向下与 SUMO Y 轴向上的差异；道路、车辆、RSU 和通信连线使用同一坐标系，距离保持不变。

roads.txt 包含 `carla-source/Co-Simulation/Sumo/examples/net/Town04.net.xml` 的全部 632 条车道及路口内部连接。`build.sh` 自动调用 `tools/export_roads.py` 生成该文件，首行记录源路网边界。它是全图静态道路背景，车辆位置实时更新；Qtenv 使用 Ctrl + 鼠标滚轮缩放，通过滚动条移动视野以寻找路口。第一版信道仅包含路径损耗和噪声，不自动导入 CARLA 建筑物。

实体塔身使用 CARLA 自带的 `/Engine/BasicShapes/Cylinder.Cylinder` 静态网格，从 `base_z` 连续覆盖到 `z`，直径不超过 1.2 米。由于运行时网格仅支持等比缩放，塔身由多个相接圆柱构成（当前约 60 米塔身为 50 段），关闭物理运动和重力、开启碰撞。它是可见实体，会遮挡传感器射线，也会阻挡车辆；顶部彩色框、光环和天线仍是辅助绘图，没有碰撞。不会修改 CARLA 系统地图文件。实体由独立 RSU 命令管理，命令正常退出、终止或崩溃均由共享桥清理，部分创建失败会回滚已生成部件。

修改 RSU 坐标后只需在 RSU 终端按 Ctrl+C，再执行 `bash vanet/run_rsu.sh`；OMNeT++ 无需重启。传感器有回波仍要求实体在量程和视场内、且未被其他物体遮挡；点云中可见不等于感知算法会将其识别为“RSU”。

RSU 当前基座绝对高度为 0.39 米，主体顶部/通信节点绝对高度为 60 米，装饰天线和光环最高约 68 米。名称用约 8 米高的世界空间字体向四面绘制，放在黄色天线顶部上方 5 米处，与灯光分开；名称绘图的最高处约 81 米。顶部光环半径为 5 米，用于远处识别；每个列表项的 `base_z`、`beacon_radius` 控制这些显示尺寸。建筑和树木仍可能遮挡个别视角。

## 添加多个 RSU

已扩展完成并编译通过，现有高塔保留为 **`rsu0`**，位置和高度不变。

以后在 [town04.json](town04.json) 的 **`rsus` 数组**中复制一项：

- 将 `id` 改为唯一名称，如 `rsu1`。
- 修改 `x、y、z` 及塔脚高度 `base_z`。
- 保存后重新运行 `bash vanet/run_rsu.sh`，OMNeT++、CARLA、SUMO 三个界面会统一创建；退出该 RSU 命令后全部清理，OMNeT++ 继续运行。

例如下面的第二项（坐标为格式示例，请按目标地点替换）：

```json
"rsus": [
  {
    "id": "rsu0",
    "x": 193.0, "y": -179.0, "z": 60.0,
    "base_z": 0.39, "beacon_radius": 5.0
  },
  {
    "id": "rsu1",
    "x": 235.0, "y": -150.0, "z": 30.0,
    "base_z": 0.4, "beacon_radius": 3.0
  }
]
```

- `id` 必须唯一、以 `rsu` 开头，后续只用字母、数字或下划线。该名称同时用于 OMNeT++ 节点、SUMO POI、CARLA 文字和收发日志。
- `x/y/z` 是 CARLA 世界绝对坐标（米），`z` 为通信节点及主体顶部高度；当前网络高度范围为 0–100 米。
- `base_z` 为可选的塔脚绝对高度，省略时使用邻近车道高度估算；放在坡地上建议明确填写。
- `beacon_radius` 为可选的顶部光环半径，省略时为 1.6 米。
- 保存后在 RSU 终端按 Ctrl+C，重新执行 `bash vanet/run_rsu.sh`，不必重新编译或关闭 OMNeT++。删除列表项后重启 RSU 命令即可移除对应 RSU；空列表 `[]` 表示没有可生成的 RSU，命令会直接提示退出。

多个 RSU 分别使用真实 Veins 无线收发，均可转发收到的车辆原始广播；转发前随机等待 0–20 毫秒，减少同时收到同一包后同步发送造成的碰撞。不会继续转发其他 RSU 的转发包，避免循环。当前功能不包含跨 RSU 单播路由或接入切换。

单纯修改 RSU 列表无需重启车辆或共享桥，重启 RSU 命令即可。

## 启动

1. 按工程原说明启动 CARLA Docker，加载 Town04。运行中不切换地图。
2. 首次升级 NPC 通信功能时，先正常退出车辆、NPC、RSU 命令及 OMNeT++，再执行 `bash run_lincoln.sh --stop-sumo`。重新启动车辆或 NPC 后加载新桥；无需重启 CARLA。新桥默认提供通信接口，NPC 先启动也能直接接入 OMNeT++。
3. 编译一次：

   bash vanet/build.sh

4. 两个终端分别启动以下命令，两辆车依次选择固定空闲车位 A、B：

   bash vanet/run_lincoln.sh

   每次命令只生成一辆车，需执行两次。VANET 入口计算车位坐标后，调用根 run_lincoln.sh --pose X Y Z YAW；不再向 ego_vehicle.py 传 --vanet-slot。

   首次启动的共享桥默认从 JSON 的 `network.port` 读取通信端口，提供仅本机可访问的接口；没有 OMNeT++ 接入时继续独立推进。自动车位使用进程锁和存活进程预留，并检查附近已有车辆，防止两个启动器同时选中同一车位。车位都占用时提示失败，不随机跳到其他道路。原 bash run_lincoln.sh 的随机出生行为保留。
5. 两辆车就绪后启动网络界面：

   bash vanet/run_omnet.sh

   在 Qtenv 点 Run（F5），展开 Town04Vanet，观察 lincoln0、lincoln1。此时不创建 RSU；两车需要相距不超过 40 米才能直连，可用现有 Lincoln 面板低速靠近。新车加入时也会自动创建 OBU。首次接入之后，每个 CARLA 步长内的网络事件处理完成才 ACK 放行下一步。
6. 需要观察 RSU 转发时，在另一个终端运行：

   bash vanet/run_rsu.sh

   RSU 会同时加入 CARLA、SUMO 和已经运行的 OMNeT++。在该终端按 Ctrl+C 移除，车辆和 OMNeT++ 保持运行。也可以先运行 RSU 命令，再打开 OMNeT++。

Qtenv 暂停会暂停共享仿真推进，这是同步联仿的预期行为。退出网络界面后，Python 桥检测断开，车辆仿真恢复独立推进。CARLA/桥停止时网络接口会报错，需重新启动网络实验。启动新车辆前请保持 Qtenv 正在运行，原共享桥就绪检查需要看到帧推进。

等待共享桥下一帧时，OMNeT++ 会继续处理界面操作，Stop 可以暂停等待，Run/Fast 从同一个同步边界恢复。收到完整快照后才更新车辆和 RSU，暂停不会重复发送 ACK；场景清理较慢时也不会因旧的 10 秒接收超时而终止网络。删除 RSU 后，若两车仍超出配置的 V2V 半径，无通信连线是预期行为，仿真时间仍应继续前进。

普通 Run 会播放详细动画，实际运行速度可能很慢；Fast 模式适合较快推进。默认 Live 实验没有仿真时长限制，可以常开；若需要固定时长，可传 `--sim-time-limit=300s`。车辆编号按共享桥本次运行的生成顺序递增；销毁后新建车辆会取得新的编号。

OMNET_ROOT、VEINS_ROOT 可覆盖安装路径。VANET 端口默认自动读取 town04.json 的 network.port。`omnetpp.ini` 不再保存 RSU 坐标，修改位置只改 JSON。CARLA 非默认地址可给网络启动器传 `--carla-host=<地址> --carla-port=<端口>`。

## 通信范围与车辆接入

每辆完成 CARLA/SUMO 同步的车辆都会独立创建 OBU，包括 Lincoln、SUMO NPC 和其他已同步车辆，不需要等第二辆车或 RSU 加入。无线范围按地图平面 x/y 距离计算，高度仍参与原有 Veins 路径损耗。以下为默认配置：

| 链路 | 最大平面距离 | 行为 |
|---|---|---|
| 车辆 ↔ 车辆 | 40 米 | 范围内可双向直连，与有无 RSU 无关 |
| 车辆 ↔ RSU | 220 米 | 双向使用同一个 RSU 覆盖半径 |
| RSU ↔ RSU | 220 米 | 可接收广播，仍不转发其他 RSU 的转发包 |

边界计入范围，超出半径的无线信号在本工程的 `RangePhy80211p` 中归零，不能收包或造成干扰。范围内仍使用真实 Veins 802.11p MAC/PHY，因此碰撞或信道噪声可能造成个别丢包。两个车只要都在同一 RSU 的 220 米范围内，即使彼此超过 40 米，也能经该 RSU 转发；两车相距不超过 40 米时，直连和 RSU 转发同时保留。未加入车辆间多跳转发或跨 RSU 路由。

无需 RSU 时，只运行车辆和 OMNeT++，不运行 `run_rsu.sh`；若 RSU 命令已运行，在该终端按 Ctrl+C 即可，不需要修改 JSON 或关闭 OMNeT++。现有两个固定出生点相距约 48 米，在新规则下不会显示 V2V 直连；驶入 40 米内即可直连，或保持双方处于同一 RSU 覆盖范围内经 RSU 通信。车辆和 RSU 位置无需为此次范围修改而改变。

**所有默认半径只在 `town04.json` 的 `communication` 中修改，单位米：**

```json
"communication": {
  "vehicle_range": 40.0,
  "rsu_range": 220.0,
  "range_overrides": {}
}
```

- `vehicle_range`：全部 Lincoln、NPC 及其他车辆的默认 V2V 半径。
- `rsu_range`：全部 RSU 的默认覆盖半径，车与 RSU 双向通信均按该 RSU 的半径判断。
- `range_overrides`：按**完整节点名称**单独指定半径；没列出的节点继续使用默认值。例如 `{"lincoln0": 25.0, "npc_abcd_0": 20.0, "rsu0": 260.0}`。`npc_abcd_0` 是名称示例，替换成 SUMO/OMNeT++ 中实际显示的 NPC ID；新的 NPC 批次会生成新的批次 ID。

车辆与车辆、RSU 与 RSU 的有效链路半径取双方较小值，保持双向规则一致。例如把一辆车改为 60 米、另一辆仍为 40 米，两者仍需相距不超过 40 米。车辆的半径覆盖只影响 V2V，车与 RSU 仍使用 RSU 半径。

保存后**重新打开 OMNeT++** 即应用于现有和之后加入的所有节点，无需重新编译或重启车辆。OMNeT++ 的范围虚线、实际无线收包、握手、CARLA 连线全部使用同一套半径判断；`omnetpp.ini` 不再保存另一份半径。RSU 的增删和位置变化仍只需操作独立 RSU 命令。

## NPC 通信与动画

原来的命令即可生成参与通信的 NPC，也可以在没有 Lincoln 时先执行：

```bash
~/carla-venv/bin/python \
carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py \
-n 200 \
--sumo-gui \
--step-length 0.05 \
--tls-manager carla
```

另开终端执行 `bash vanet/run_omnet.sh`，在 Qtenv 点 Run/Fast；若 OMNeT++ 已在运行，NPC 生成后自动加入。NPC 沿用 SUMO 的 `npc_<批次ID>_<序号>` 名称，Lincoln 仍使用 `lincoln0`、`lincoln1` 等名称。只有已实际生成并完成 CARLA/SUMO 映射的车辆才会加入，SUMO 等待插入的车辆暂不创建 OBU。

NPC 支持与其他 NPC、Lincoln 及 RSU 通信，并使用相同的 OMNeT++ 收发动画和 CARLA 半透明灰色/蓝黄/绿色连线。NPC 离开路网或在 NPC 命令终端按 Ctrl+C 后，对应 OBU 和连线自动删除；该命令只移除本批 NPC，OMNeT++ 可以继续运行。200 辆时建议用 Express（F7）推进，并取消 Express 窗口的 `auto-update inspectors`；需要查看地图时可以手动刷新或用 Fast（F6）。普通 Run 的逐包动画会显著放慢运行。大场景的提示简化规则见下节。

首次升级需要按“启动”第 2 步重启共享桥一次；之后改通信半径只需重开 OMNeT++。默认不需要设置 `VANET_PORT`，高级端口覆盖仍保留该环境变量。

## 大规模运行与性能

默认关闭完整 `.elog` 事件记录，保留所有节点的通信 CSV、`.vec/.sca` 指标以及 CARLA 连线。CSV 使用缓冲写入，在每次 probe 定时器触发（默认约 1 仿真秒）和节点正常结束时刷新；强制杀进程可能丢失尚未刷出的尾部记录。

### 清理仿真日志

先正常关闭 OMNeT++，再从工程根目录查看结果目录大小和事件日志：

```bash
du -sh vanet/omnet/results
find vanet/omnet/results -maxdepth 1 -type f -name '*.elog' -ls
```

确认不再需要事件回放后，只删除可能长到数 GB 的 `.elog`：

```bash
find vanet/omnet/results -maxdepth 1 -type f -name '*.elog' -delete
```

如果还要清理逐车通信记录，可另执行下面一条；它会删除所有 `.csv` 中的逐包收发时间、序号和延迟，之后无法再用这些文件计算投递率等指标。`.vec/.sca` 汇总指标不受影响：

```bash
find vanet/omnet/results -maxdepth 1 -type f -name '*.csv' -delete
```

默认 Live 配置已关闭 `.elog`；只有启用 `PacketDebug` 等事件记录配置才会再次生成。当前结果目录的空间占用也可能主要来自大量 CSV，先用 `du` 查看再决定删哪类文件。

当车辆与 RSU 合计超过 20 个节点时，自动停止 OMNeT++ 临时逐包连线和 TX/RX 气泡；地图、持续的范围虚线、实际无线通信和所有节点的 CARLA 半透明通信线仍保留。回到 20 个及以下时自动恢复提示。可只调此阈值：

```bash
bash vanet/run_omnet.sh --*.manager.packetAnimationLimit=50
```

设为 `0` 可关闭提示，`-1` 表示不限节点数。需要完整底层事件回放及不限数量的逐包提示时使用 `bash vanet/run_omnet.sh -c PacketDebug`；此模式可能非常慢并生成 GB 级日志。

无线范围仍为 JSON 中的半径。超出范围、原本功率为零的帧，现在在进入接收解码和信道跟踪之前丢弃，避免无效事件；范围内继续使用真实 Veins MAC/PHY、碰撞、HELLO/ACK 和 RSU 转发。覆盖关系每帧计算一次供显示与反馈复用；CARLA 覆盖层在数据、视角和窗口位置都未变化时不重复重绘。

无线模型和 Qtenv 等待刷新调整在重新打开 OMNeT++ 后生效；下面的交通灯缓存及分段计时位于共享桥，需要在正常退出车辆、NPC、RSU 和 OMNeT++ 后按“启动”第 2 步重启共享桥。已有运行中的大日志不会自动删除。

在本机的独立密集测试中，200 个 NPC OBU + 1 个 RSU 推进 3 仿真秒：修改前约 **73.6 秒**，只关闭事件日志约 **23.0 秒**，全部优化后约 **4.2 秒**。修改前后全部 **64,805 条**握手/业务 CSV 记录（含收发时间、序号、延迟）和逐帧连线反馈一致，其中业务 RX 为 39,383 条。测试使用同一随机种子、静止合成坐标和 Cmdenv，不包含 CARLA 渲染或真实车辆推进；这不是整套联仿的帧率保证。

可复测原零功率处理路径与提前过滤路径的性能及逐条结果一致性：

```bash
python3 vanet/tests/benchmark_radio.py
```

该测试使用隔离端口，不连接正在运行的 CARLA/SUMO。两条路径均关闭事件日志，因此其加速比只比较范围过滤，不包含关闭日志的收益。整套联仿仍按 0.05 秒步长等待网络 ACK；OMNeT++ 或 CARLA 任一主线程耗时增加都会降低现实推进速度，多核 CPU 的总体占用率低并不代表没有瓶颈。

需要定位本机瓶颈时，在**启动新的共享桥之前**设置 `VANET_PROFILE_PATH`，例如 `VANET_PROFILE_PATH=/tmp/vanet-phases.jsonl bash run_lincoln.sh`。桥每 100 帧追加一行 JSON，分别记录 SUMO 推进、SUMO→CARLA 车辆同步、CARLA `world.tick()`、CARLA→SUMO 同步、等待 OMNeT++ ACK 的平均值和 P95（毫秒），以及车辆数量。已在运行的桥不会因后来设置环境变量而开始记录；停止全部客户端后按现有步骤重启桥才生效。记录文件在 `/tmp`，不进入工程仓库。

本机 Town04、0.05 秒步长的分段实测：约 49 辆 NPC、没有 Lincoln 或 OMNeT++ 时，SUMO→CARLA 车辆更新约 1 毫秒/步，CARLA `world.tick()` 约 10 毫秒/步。再加入一辆保留原有 **31 个传感器**的 Lincoln，CARLA `world.tick()` 约 33 毫秒/步，已接近单步预算。CARLA 控制的交通灯原先每帧重复通过 TraCI 写同一灯态，使 CARLA→SUMO 阶段在空场景也约 5–6 毫秒；现在缓存每个 SUMO 灯控连接上次发送的状态，仅首次及灯态变化时写入，空场景约 0.25 毫秒/步。没有改动车辆传感器配置。

同一批约 48–49 辆 NPC 接入网络后，Qtenv Fast 的网络等待约 15–19 毫秒/步，Cmdenv 稳定后约 1.5–2.3 毫秒/步；Qtenv 界面绘制是这部分差值的主要来源。Qtenv 等待桥快照时默认最多每 100 毫秒处理一次 GUI idle，仍可操作 Stop/Run；可用 `--*.manager.guiIdleInterval=0s` 恢复每次轮询都刷新。做无界面性能对照可运行 `OMNET_UI=Cmdenv bash vanet/run_omnet.sh --cmdenv-express-mode=true`，普通 `bash vanet/run_omnet.sh` 仍默认打开 Qtenv。含 49 辆 NPC、上述 Lincoln 和 Cmdenv 的总执行耗时约 45 毫秒/步，逼近 50 毫秒目标。

另一次真实 200 辆 NPC 请求中，路网同时容纳约 186–193 辆；未接入 OMNeT++ 时，约 193 辆的每步实际执行约 48 毫秒，其中 CARLA `world.tick()` 约 31 毫秒、SUMO 推进约 13 毫秒、SUMO→CARLA 同步约 3 毫秒。接入 Cmdenv 后，约 186 辆稳定运行时网络等待约 17 毫秒，合计约 65 毫秒/步，即不计其它启动耗时也低于 20 步/秒。这里的 `sumo_step` 包含 SUMO GUI 周期性约 100 毫秒的阻塞；车辆数是实际进入路网并映射到 CARLA 的数量，不是请求的 200 辆。结果随机器、车流位置和界面状态变化；保留传感器的 Lincoln 未加入这次 200 NPC 测试。

新 NPC 初次映射时若位置短暂落在 Town04 的 OMNeT++ 地图边界外，该帧暂不创建 OBU；回到有效区域后自动接入。Veins 初始化时也使用已转换的 Qtenv 地图坐标做边界检查，允许道路附近合法的负 SUMO 坐标。两处处理避免单辆车的初始位置导致整个网络停止，不改动车辆或 SUMO 路径。

## 看什么

每个节点约每秒发送一个广播 probe；RSU 收到车辆原始 probe 后只转发一次。节点另发送 HELLO 广播，并以 MAC 单播 ACK 确认邻居。业务数据仍是应用层广播连通性测试，尚未实现业务单播路由、IP 路由、跨 RSU 或接入切换。

- 范围内的节点持续显示虚线：蓝色为 V2V，紫色为涉及 RSU 的链路。每个 CARLA 快照更新位置，驶出范围或车辆销毁后删除，驶入范围或加入新车后自动建立。虚线表示范围内可尝试通信，不代表每个数据包都投递成功。
- 小场景的发送/接收节点显示 TX/RX 气泡，数量阈值见上节。
- 开启逐包提示时，**实际收包成功**画绿色直达连线；经 RSU 转发的接收画橙色连线，0.25 仿真秒后消失。未收到的包不会画成功连线。
- CSV 在 omnet/results/，包含发送方、接收方、原始来源、序号、仿真时间、端到端延迟及 direct/relay。
- .vec/.sca 记录收发计数、转发计数与延迟；可选的 .elog 支持事件回放，默认关闭。

转发报文保留原始序号和发送时间。发送方可能收到自己的 RSU 转发副本，分析时按 origin 排除；direct/relay 要分别计算，不能把所有 RX 相加当成投递率。CSV 每个模块实例独立文件，同一运行配置重跑会覆盖同名输出，重要结果先复制保存。

## CARLA 中的通信连线

启动 OMNeT++ 并点击 Run/Fast 后，CARLA 主窗口会自动显示已完成双向 HELLO/ACK 握手的节点连线，包括 NPC、Lincoln 及 RSU。范围来自 JSON 的 `communication` 配置及单节点覆盖；超出范围或节点删除立即移除连线，握手连续 3 仿真秒未更新也会失效。无需另外运行显示命令。

| 状态 | 半透明连线颜色 |
|---|---|
| 已握手，近期无业务收包 | 整条灰色 |
| 单向业务传递成功 | 发信端到世界坐标中点为蓝色，中点到收信端为黄色 |
| 两方向均有业务传递成功 | 整条绿色 |

颜色按真实 Veins 成功收包反馈更新，每次业务收包保留 350 毫秒**仿真时间**，随后恢复灰色；在这段时间内两个方向均有成功收包即显示绿色。HELLO/ACK 控制包不会触发业务颜色，也不计入 probe 收发统计。它们在 CSV 中另记为 `HELLO_TX / HELLO_RX / ACK_TX / ACK_RX`，`path=handshake`，分析业务投递率时排除。实际 RSU 转发按车辆—RSU 的各段链路显示，不画超过直连范围的车辆间虚拟链路。

线宽约 1.5 像素，灰线 Alpha 约 43%，彩色线约 49%。位置随节点和 CARLA 观察视角更新，车辆端在车顶附近，RSU 端在配置的通信节点高度 `z`。连线通过跟随 CARLA 游戏窗口的 Qt/X11 透明覆盖层绘制，鼠标操作穿透；没有创建世界几何或传感器可探测实体。因此 CARLA 主窗口可见，车辆的 Camera、LiDAR、Radar 及传感器录制数据中均没有这些线。覆盖层不增加 TraCI 客户端，也不推进 CARLA 时钟；退出 OMNeT++ 后自动清理。

当前 CARLA Shipping 包的原生 HUD 绘线会将亮度和 Alpha 同时放大，无法保留所需半透明效果，因此使用本工程覆盖层，未修改 CARLA 系统文件（见 [CARLA 0.9.16 HUD 绘制源码](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Util/DebugShapeDrawer.cpp)）。覆盖层使用本机已有 PyQt5 和 X11，适用于当前有界面的 CARLA 主窗口；不用于无窗口渲染或其他远程视频窗口。状态及错误日志位于共享桥控制目录的 `vanet-communication.json / vanet-communication.log`。

**已有旧共享桥首次升级时**：正常退出 OMNeT++、RSU、NPC 命令及车辆，执行 `bash run_lincoln.sh --stop-sumo`，再重新启动车辆或 NPC 和 OMNeT++，可选启动 RSU。共享桥需要加载 NPC 接入与通信反馈实现；无需重启 CARLA。之后沿用原来的启动命令即可。

## 验证

不依赖 CARLA 的无线与增删测试：

   bash vanet/run_omnet.sh -u Cmdenv -c Smoke

真实 TCP 桥协议与合成车辆快照测试：

   python3 vanet/tests/test_transport.py

慢速同步回归：`python3 vanet/tests/test_transport_wait.py` 将 RSU 删除快照分段发送，并在中途延迟 11 秒；验证同一次 OMNeT++ 运行继续完成删除、重新添加与再次删除，V2V 保持通信，每帧恰好一个 ACK，仿真时间不跳步。Qtenv 另已实测在部分快照到达时 Stop，补齐数据后保持暂停，再 Run/Fast 正常恢复。

通信范围回归：`python3 vanet/tests/test_radio_range.py` 使用真实 Veins MAC/PHY 和隔离的 TCP 合成快照，验证无 RSU 的 V2V、单车与 RSU、40/220 米边界、越界阻断、RSU 转发、直连与转发并存、三车独立接入及移动时连线断开/恢复。不连接真实 CARLA，也不接管共享桥。

临时越界和负 SUMO 坐标接入也包含在通信范围回归中：OBU 跳过错误初始位置，进入地图后创建并恢复双向收包；合法的负 SUMO x 可创建车辆和 RSU。`~/carla-venv/bin/python vanet/tests/test_traffic_light_sync.py` 验证交通灯首次设置、连续不变时省略 TraCI 重发以及状态变化时继续同步。

NPC 与统一半径回归：`python3 vanet/tests/test_npc_radio.py` 验证 NPC/Lincoln 混合收发、JSON 默认值与节点覆盖对 PHY/握手/连线的一致作用，以及 200 个 NPC OBU 中删除并重新加入 50 个节点后的通信与动画反馈。该回归使用真实 Veins 和合成快照；上面的 200 辆请求实测则运行在真实 CARLA/SUMO 场景。

车辆映射回归：`~/carla-venv/bin/python vanet/tests/test_vehicle_sync.py` 使用实际同步适配器、模拟世界与真实 TCP，验证没有 `VANET_PORT` 时仍创建接口、CARLA/SUMO 两个方向的车辆映射、NPC 生成/销毁、未完成映射车辆的排除及动画反馈；只有现有同步时钟推进仿真。

动态 RSU 回归：`python3 vanet/tests/test_dynamic_rsu.py` 在同一个真实 Veins 运行中，验证无 RSU 开始、动态添加/删除、同名重新加入、多个 RSU 和位置变化；所有阶段均保留 V2V 收包，不重启网络。

CARLA 连线状态回归：`python3 vanet/tests/test_communication.py` 用真实 Veins HELLO/ACK 和 TCP 反馈验证灰色、两个单向方向、双向绿色、未完成握手不画线、越界和动态 RSU；同时检查世界坐标中点、半透明颜色及视角投影。CARLA 主窗口显示已用临时 RGB 相机实测，桌面连线可见而传感器图像没有对应线条；测试相机和覆盖层随后清理。

Smoke 的第二辆车在 5 秒删除、6 秒重新生成。合成快照测试连接同一个 Python VanetBridge 类，与 OMNeT++ 执行逐帧 ACK，并检查 V2V、V2I、RSU 转发收包；它不会连接 CARLA 或启动 SUMO。

`~/carla-venv/bin/python vanet/tests/test_scene_lifecycle.py` 使用独立的无界面 SUMO 和模拟 CARLA，验证 JSON 坐标传递、独立 RSU 命令的实体/标记创建、Ctrl+C/终止/崩溃及部分创建失败时的清理，并检查 RSU 与 OMNeT++ 生命周期互不关闭。Smoke 配置与合成快照测试不显示真实场景标记；其他自建离线测试可显式传 `--no-scene-markers`。

真实传感器验证：关闭 OMNeT++ 和 RSU 命令，保持共享桥正常推进，运行 `~/carla-venv/bin/python vanet/tests/test_rsu_sensors.py`。脚本在当前默认 `rsu0` 位置短暂生成实体塔身，并从 10 米外用语义 LiDAR 的 Actor ID 和 Radar 距离验证回波，随后清理自己的实体和传感器；不接管仿真时钟。其他 RSU 位置可能需要调整测试视点以避开遮挡。

## 原工程的接入改动

- run_lincoln.sh：无参数仍随机出生；显式校验并传递 --pose X Y Z YAW。
- vanet/runtime/launch.py：将实验车位转换为 --pose 后调用根启动器，SUMO 就绪检查只由根启动器执行一次；释放查询客户端后用关闭继承文件描述符的子进程启动车辆，父进程保持车位预留。
- lincoln/ego_vehicle.py：使用原有 --pose 接口，无 VANET 专用修改。
- carla-source/Co-Simulation/Sumo/run_synchronization.py：VANET_PORT 设置时启用本目录 TCP 端点，每步同步完车辆之后发送快照，退出时关闭端点。
- carla-source/Co-Simulation/Sumo/sumo_integration/sumo_simulation.py：支持按前缀独立编号，Lincoln 映射使用 lincoln0、lincoln1；网络节点沿用该名称。
- carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py：桥线程检查独立 RSU 会话，统一添加和移除 CARLA / SUMO RSU 实体与标记，并将实际 RSU 列表加入网络快照；Qtenv 暂停等待 ACK 时也处理控制请求和 RSU 清理。
- carla-source/Co-Simulation/Sumo/run_synchronization.py：默认启用本机 VANET 接口，合并 CARLA→SUMO 与 SUMO→CARLA 的车辆映射，使 NPC 和 Lincoln 一并接入。

其他新实现均在本目录；不启动第二套 SUMO、不建立第二个 TraCI 客户端，也不由 OBU 调用 world.tick()。

多 RSU 回归：`python3 vanet/tests/test_multi_rsu.py` 验证两节点无线收发、空列表及无效配置；`~/carla-venv/bin/python vanet/tests/test_scene_lifecycle.py` 验证两个标记同时创建和清理。测试结果使用独立临时目录。
