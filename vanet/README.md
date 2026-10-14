# Town04 VANET 联仿

本目录在现有 CARLA/SUMO 共享桥上接入 OMNeT++ 6.1 和 Veins 5.3.1。车辆运动仍由 CARLA/SUMO 决定，Veins 为已入网车辆创建 OBU，模拟 802.11p 无线通信；本项目不使用 INET。共享桥是唯一推进 CARLA 时钟和调用 TraCI 的进程。

以下命令均从**工程根目录**执行。共享配置入口是 [town04.json](town04.json)，RSU 位置列表是 [scene/rsu/config/town04.json](../scene/rsu/config/town04.json)；运行入口是 `carla/Lincoln/run_lincoln.sh --vanet-slot auto`、`omnet/run_omnet.sh` 和 `scene/rsu/run_rsu.sh`。

## 快速启动

1. 按工程根目录 README 启动 CARLA，加载 Town04。联仿期间不要切换地图。首次使用或修改 OMNeT++ 模型后编译：

   ```bash
   bash omnet/build.sh
   ```

2. 在两个终端各运行一次车辆命令。每次只生成一辆 Lincoln；启动器依次选择路口 924 附近的空闲车位 A、B，并自动启动或复用共享 SUMO 桥。

   ```bash
   bash carla/Lincoln/run_lincoln.sh --vanet-slot auto
   ```

   若只想启动共享桥、不生成 Lincoln：

   ```bash
   ./cobridge/run_cobridge.sh
   ```

3. 在另一终端启动 OMNeT++：

   ```bash
   bash omnet/run_omnet.sh
   ```

   在 Qtenv 点击 Run（F5）或 Fast（F6），展开 `Town04Vanet` 查看车辆 OBU。两辆默认车位相距约 48 米，超过默认 40 米 V2V 半径；驶近后才能直连。没有 RSU 时也能观察车车通信。

4. 需要 RSU 时，在独立终端运行：

   ```bash
   bash scene/rsu/run_rsu.sh
   ```

   RSU 会同时出现在 CARLA、SUMO 和运行中的 OMNeT++。在该终端按 Ctrl+C 只删除 RSU；车辆和 OMNeT++ 继续运行。也可以先启动 RSU，再打开 OMNeT++。

退出时先在车辆、NPC、RSU 各终端按 Ctrl+C，再关闭 OMNeT++。需要停止由启动器管理的共享桥时运行：

```bash
bash cobridge/run_cobridge.sh --stop
```

这不会关闭 CARLA。修改了桥的 Python 代码时，已运行的桥不会自动加载新代码：先退出相关客户端并停止桥，再按上述步骤启动。只改 RSU 列表时重启 RSU 命令；只改无线半径时重启 OMNeT++。

Qtenv 暂停会让同步联仿暂停推进；Stop 后再 Run/Fast 会从同一仿真边界继续。关闭 OMNeT++ 后桥会断开网络连接，CARLA/SUMO 继续推进。启动新车前保持 Qtenv 正在运行，以便共享桥就绪检查看到帧推进。

## 车辆与 NPC

Lincoln 入口无参数时随机选择空闲出生点；`--pose` 使用 **CARLA 世界坐标**，依次为 x、y、z（米）和 yaw（度）：

```bash
bash carla/Lincoln/run_lincoln.sh
bash carla/Lincoln/run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
bash carla/Lincoln/run_lincoln.sh --pose 235.597504 -172.815598 0.695892 180.326126
```

固定点被占用会报错，不会改到随机位置。`carla/Lincoln/run_lincoln.sh --vanet-slot auto` 使用 `town04.json` 中的 A/B 实验车位，调用同一个 Lincoln 车辆入口；车位选择有进程锁和占用检查。Lincoln 在 SUMO 与 OMNeT++ 中使用 `lincoln0`、`lincoln1` 等名称。

NPC **默认只参与 SUMO/CARLA 交通，不创建 OBU**。例如生成 200 辆普通 NPC：

```bash
./sumo/traffic/run_npc.sh \
  -n 200 --sumo-gui --step-length 0.05 --tls-manager carla
```

只让某一批 NPC 参与通信，在该批命令加 `--vanet`：

```bash
./sumo/traffic/run_npc.sh \
  -n 20 --sumo-gui --step-length 0.05 --tls-manager carla --vanet
```

不同终端可同时运行普通批次和联网批次；Ctrl+C 只移除该终端创建的批次。联网 NPC 的名称为 `npc_<批次ID>_<序号>`，完成 SUMO/CARLA 映射后才在 OMNeT++ 创建 OBU；等待插入路网的车辆不会提前入网。Lincoln 启动器的 `--sumo-npcs N` 选项创建的常驻背景批次也默认不联网。首次升级到按批次选择是否联网时，必须重启共享桥。

## 配置、路口与坐标

[town04.json](town04.json) 定义实验车位、通信半径及网络端口，通过 `rsu_config` 引用 [RSU 位置列表](../scene/rsu/config/town04.json)。当前选用 Town04 路口 **924**：车位 A、B 位于相互垂直的入口；`rsu0` 位于路侧草地、红砖墙前。

| 对象 | CARLA x / y（米） | 其他参数 |
|---|---:|---|
| Lincoln A（车道修正后） | 199.390 / -204.544 | yaw 91.314° |
| Lincoln B（车道修正后） | 235.598 / -172.816 | yaw 180.326° |
| `rsu0` | 193.000 / -179.000 | 基座 z 0.39 米，顶部 z 60 米 |

车辆车位在启动时由当前 CARLA 驾驶车道 waypoint 修正高度和方向；横向偏差超过 4 米、方向差超过 35 度或位置落在路口内部时会停止。RSU 的 `x/y/z` 是 CARLA 世界绝对坐标，不是 SUMO GUI 状态栏坐标。

SUMO GUI 底部同时显示 `SUMO x/y | CARLA x/y` 和实时 `Sim/real` 倍率。右键菜单分别提供 `Copy SUMO coordinates`、`Copy CARLA coordinates`，复制鼠标落点的 x/y；z 和 yaw 需另行确定。坐标显示读取当前 SUMO 路网的 `netOffset`：

```text
CARLA.x = SUMO.x - offset.x
CARLA.y = offset.y - SUMO.y
```

鼠标落点与车辆中心、车头参考点可能不同。工程专用 GUI 位于 `sumo/gui/`，首次带 GUI 启动共享桥时自动构建；也可预先构建：

```bash
python3 sumo/gui/build.py
```

这不会修改系统 SUMO。`Sim/real` 显示近约 2 秒的仿真时间增量与现实时间增量之比；`1.00x` 为同速，暂停时为 `0.00x`。GUI 将周期性重绘等待由约 100 毫秒缩短到 10 毫秒。修改 GUI 后须关闭旧 SUMO 窗口及共享桥，再启动新窗口。

OMNeT++ 的道路背景是**静态**的：`omnet/build.sh` 调用 `omnet/tools/export_roads.py`，从 `cobridge/carla-source-official/Co-Simulation/Sumo/examples/net/Town04.net.xml` 导出所有车道及内部连接到 `omnet/roads.txt`。Qtenv 不直接绘制 CARLA 地图，也不从本次临时生成的 SUMO 路网实时读图。道路、车辆和 RSU 在 Qtenv 中共用同一坐标转换：

```text
桥发送：SUMO.x = CARLA.x + town04.json 中的 offset_x
          SUMO.y = -CARLA.y + town04.json 中的 offset_y
Qtenv 绘制：x' = SUMO.x - 路网最小 x + 40
           y' = 路网最大 y - SUMO.y + 40
```

桥每帧从 CARLA 快照读取车辆位置，借 SUMO 映射确定节点名称，通过本机 TCP 交给 OMNeT++；Veins 的 `ExternalMobility` 据此移动 OBU。`town04.json` 的网络偏移是当前工程配置值，与 SUMO GUI 从路网读取的 `netOffset` 是两条配置路径；若更换或重新转换路网，应核对两者。Qtenv 中可用 Ctrl + 鼠标滚轮缩放，再用滚动条移动视野。信道模型不自动导入 CARLA 建筑物作为无线障碍。

## RSU：新增、修改与清理

RSU 的唯一位置列表是 [scene/rsu/config/town04.json](../scene/rsu/config/town04.json) 的 `rsus`。保留现有 `rsu0`，复制一个列表项并改唯一 `id` 与坐标即可增加 RSU。例如：

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

第二项坐标只是格式示例，需换成实际目标位置。`id` 必须唯一，以 `rsu` 开头，后续只用字母、数字或下划线；它同时是 OMNeT++ 节点名、SUMO POI 名和 CARLA 显示名。`z` 为通信节点与主体顶部绝对高度；`base_z` 是塔脚绝对高度，可省略并由邻近车道估算。`beacon_radius` 是顶部光环半径，省略时为 1.6 米。网络高度范围为 0–100 米。

共享配置中 `rsu_config` 默认值为 `../scene/rsu/config/town04.json`，相对共享配置文件所在目录解析；RSU 配置的 `map` 必须与共享配置一致。自定义 `--vanet-config` 支持引用另一个 RSU 文件，也支持内嵌 `rsus` 列表，两种定义不能同时出现。加载后产生完整配置快照，传给桥和 OMNeT++。

修改列表后，在 RSU 终端按 Ctrl+C，再运行：

```bash
bash scene/rsu/run_rsu.sh
```

车辆、共享桥和 OMNeT++ 不必重启。删除列表项后重启 RSU 命令会删除对应 RSU；空列表 `[]` 表示不生成 RSU。RSU 命令异常终止时桥也会清理其实体和图标。

CARLA 中的塔身使用带碰撞的静态圆柱网格，能挡住车辆及 LiDAR/Radar 射线；顶部彩色框、光环、天线及巨大洋红色名称是辅助绘图，不具碰撞。塔身与显示均由 RSU 命令运行时创建，不改 CARLA 系统地图。当前 `rsu0` 顶部光环半径为 5 米，装饰最高约 68 米，名称最高约 81 米；能否产生传感器回波仍取决于量程、视场及遮挡。

## 无线通信与可视化

默认平面覆盖范围如下；高度仍参与 Veins 路径损耗。

| 链路 | 默认最大平面距离 | 行为 |
|---|---:|---|
| 车辆 ↔ 车辆 | 40 米 | 范围内可直接收发，不依赖 RSU |
| 车辆 ↔ RSU | 220 米 | 按该 RSU 的半径双向判断 |
| RSU ↔ RSU | 220 米 | 能接收广播，不继续转发其他 RSU 的转发包 |

在 `town04.json` 的 `communication` 中统一调整：

```json
"communication": {
  "vehicle_range": 40.0,
  "rsu_range": 220.0,
  "range_overrides": {}
}
```

`range_overrides` 可以用完整节点名指定单独半径，例如 `{"lincoln0": 25.0, "rsu0": 260.0}`。车辆与车辆、RSU 与 RSU 取双方较小半径；车辆与 RSU 以 RSU 半径判断。边界计入覆盖范围，超出半径的信号在项目的 `RangePhy80211p` 中被阻断；范围内仍经过 Veins 802.11p MAC/PHY、路径损耗、噪声与碰撞。保存配置后重新打开 OMNeT++ 生效，无需重编译。`omnetpp.ini` 不再单独保存半径或 RSU 坐标。

每个 OBU 约每仿真秒发送一个广播 probe，另外通过 HELLO 广播与 MAC 单播 ACK 确认邻居。RSU 收到车辆原始 probe 后随机等待 0–20 毫秒再转发一次；不会转发来自其他 RSU 的转发包。两辆车即使相距超过 40 米，只要同处一个 RSU 的覆盖内仍可经它收到转发。目前是广播连通性实验，**没有**车辆多跳、跨 RSU 单播路由、IP 路由或接入切换。

Qtenv 中，蓝色虚线表示 V2V 覆盖，紫色虚线表示涉及 RSU 的覆盖。虚线只代表几何范围，不代表每包成功。小场景实际收包可显示 TX/RX 气泡及短暂的绿色直达、橙色 RSU 转发线。节点增删、移动、越界时，OBU 与覆盖线随下一帧快照更新。

OMNeT++ 运行后，CARLA 主窗口还会显示**完成双向 HELLO/ACK 握手**的通信连线：

| 状态 | CARLA 半透明线 |
|---|---|
| 握手成功，近期无业务收包 | 灰色 |
| 单向业务收包成功 | 发送端至中点蓝色，中点至接收端黄色 |
| 双向业务收包成功 | 整条绿色 |

业务收包颜色保留 350 毫秒仿真时间；HELLO/ACK 不触发业务颜色。握手约 3 仿真秒未更新、节点删除或超出范围时线会消失。RSU 转发按实际车辆—RSU 链路显示，不画越过直连范围的车车虚拟线。线由 CARLA 主窗口的 Qt/X11 透明覆盖层绘制，不生成传感器可见的世界几何；车辆 Camera、LiDAR、Radar 图像中不会出现这些线。RSU 实体塔身与此不同，传感器可以探测。

## 结果、性能与日志

输出位于 `omnet/results/`：逐节点 CSV 记录时间、收发方、原始来源、序号、延迟和 direct/relay；`.vec/.sca` 记录统计指标。HELLO/ACK 另记为握手事件，计算业务投递率时应排除；RSU 转发包保留原始序号和发送时间，分析时还应排除发送方收到自己的转发副本。默认 Live 配置关闭完整 `.elog` 事件记录。CSV 定期缓冲刷新，强制结束进程可能丢失尾部记录；重要结果在重跑前先复制保存。

大场景建议用 Qtenv Express（F7）推进，并关闭 Express 窗口的 `auto-update inspectors`；普通 Run 的逐包动画开销很大。超过 20 个节点后，临时逐包气泡和收包线默认停用，持续覆盖线、CARLA 通信线和实际无线通信仍保留。可修改提示阈值，或用 Cmdenv 对照性能：

```bash
bash omnet/run_omnet.sh --*.manager.packetAnimationLimit=50
OMNET_UI=Cmdenv bash omnet/run_omnet.sh --cmdenv-express-mode=true
```

阈值 `0` 关闭逐包提示，`-1` 不限节点数。需要完整底层事件回放时可运行以下配置，但它可能生成 GB 级日志：

```bash
bash omnet/run_omnet.sh -c PacketDebug
```

先正常关闭 OMNeT++，查看结果目录，再按需删除事件日志：

```bash
du -sh omnet/results
find omnet/results -maxdepth 1 -type f -name '*.elog' -ls
find omnet/results -maxdepth 1 -type f -name '*.elog' -delete
```

若不再需要逐包数据，也可单独删除 CSV；这会丢失计算投递率所需的原始记录，`.vec/.sca` 不受影响：

```bash
find omnet/results -maxdepth 1 -type f -name '*.csv' -delete
```

需要分析联仿单步耗时时，在**启动新共享桥前**设置性能文件路径：

```bash
VANET_PROFILE_PATH=/tmp/vanet-phases.jsonl bash carla/Lincoln/run_lincoln.sh
```

桥每 100 帧记录 SUMO 推进、两个方向车辆同步、CARLA `world.tick()` 与等待 OMNeT++ ACK 的平均值和 P95。对已运行的桥设置环境变量不会生效。`OMNET_ROOT`、`VEINS_ROOT` 可覆盖 OMNeT++ 和 Veins 安装路径；网络端口默认读取 `town04.json` 的 `network.port`，通常无需设置 `VANET_PORT`。CARLA 非默认地址可给网络启动器传 `--carla-host` 和 `--carla-port`。

现有本机实测用于定位瓶颈，不能作为新场景的帧率保证：Town04、0.05 秒步长、约 49 辆 NPC 且无 Lincoln/OMNeT++ 时，CARLA `world.tick()` 约 10 毫秒/步；再加入保留 31 个传感器的 Lincoln，约 33 毫秒/步。另一次请求 200 辆、实际入场约 186–193 辆时，未接网络约 48 毫秒/步；接入 Cmdenv 后约 65 毫秒/步，其中网络等待约 17 毫秒。独立的 200 OBU + 1 RSU 静止合成坐标测试，经事件日志关闭、越界帧提前过滤等优化，由约 73.6 秒缩短到约 4.2 秒推进 3 仿真秒；它不包含 CARLA 渲染或真实车辆运动。上述网络测试来自 NPC 默认入网的旧阶段；当前 NPC 需显式 `--vanet` 才产生同类网络负载。

## 验证与实现位置

以下是不接管真实 CARLA/SUMO 的回归命令；其中 Veins 测试使用隔离 TCP 端口及合成快照。需要排查某项功能时按需运行：

```bash
bash omnet/run_omnet.sh -u Cmdenv -c Smoke
python3 vanet/tests/test_transport.py
python3 vanet/tests/test_transport_wait.py
python3 vanet/tests/test_radio_range.py
python3 vanet/tests/test_npc_radio.py
python3 vanet/tests/test_dynamic_rsu.py
python3 vanet/tests/test_multi_rsu.py
python3 vanet/tests/test_communication.py
python3 vanet/tests/benchmark_radio.py
~/carla-venv/bin/python vanet/tests/test_vehicle_sync.py
~/carla-venv/bin/python vanet/tests/test_scene_lifecycle.py
~/carla-venv/bin/python vanet/tests/test_traffic_light_sync.py
```

`Smoke` 在 5 秒删除第二辆模拟车、6 秒重新加入。`test_radio_range.py` 验证 40/220 米边界、V2V、RSU 转发和移动；`test_npc_radio.py` 覆盖混合车辆及大量 OBU 增删；`test_dynamic_rsu.py` 与 `test_multi_rsu.py` 验证 RSU 动态生命周期；`test_vehicle_sync.py` 验证 NPC 默认不联网、按批次入网与退出清理。旧联调记录位于 `vanet/validation/live-smoke.json`，产生于硬半径规则加入之前，不能直接当作当前 40 米 V2V 的测试结果。

真实传感器验证要求共享桥继续推进、RSU 命令和 OMNeT++ 已关闭；它会短暂创建测试塔身及传感器并清理：

```bash
~/carla-venv/bin/python vanet/tests/test_rsu_sensors.py
```

代码按功能分布：`runtime/` 管理桥端 TCP 通信、连线状态和性能统计；
`common/config.py` 处理配置，工程路径在 `cobridge/common/paths.py`；根目录 `omnet/` 包含 NED/C++ 无线模型、构建和启动入口、会话管理、道路导出及结果。
RSU 入口、会话、场景管理、实体与位置定义集中在 `scene/rsu/`；CARLA 实现在其 `carla/` 子目录，SUMO 图标在 `sumo/rsu.png`，位置列表在 `config/town04.json`。覆盖层在 `carla/vanet_visual/`，定制 GUI 在 `sumo/gui/`。
Lincoln 只使用 `carla/Lincoln/run_lincoln.sh`，固定车位用 `--vanet-slot`，不另设车辆启动器。
共享桥的管理、主服务、同步、IPC、路径和性能统计在 `cobridge/common/`；SUMO/TraCI 与 NPC 客户端在 `cobridge/sumo/`，桥端 OMNeT++ TCP 传输在 `cobridge/omnet/transport.py`，CARLA 通信显示进程管理在 `cobridge/carla/communication_view.py`。统一由 `cobridge/run_cobridge.sh` 管理，官方源码未修改。旧 `cobridge/bridge/` 和 `vanet/runtime/` 已移除。
OMNeT++ 不另起 SUMO，不另连 TraCI。
