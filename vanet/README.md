# Town04 Lincoln VANET 连通性实验

本目录使用本机 OMNeT++ 6.1 + Veins 5.3.1，不依赖 INET。OBU 是 Veins 802.11p 通信节点，不是 CARLA sensor。原有 SUMO 共享桥仍是唯一仿真推进者和 TraCI 调用者。

## 当前状态

已通过两辆真实 Lincoln + 一个 RSU 的 CARLA / SUMO / Veins 联调，每辆保留原有 31 个传感器。SUMO 车辆和 OMNeT++ OBU 统一命名为 `lincoln0`、`lincoln1`。两车在不同入口，实测间距约 48.1 米；5 秒仿真内，两车双向直达各成功接收 5 包，RSU 分别收到两车各 5 包，两车也均收到 RSU 广播及转发。结果保存在 `validation/live-smoke.json`。Qtenv 已检查实际收包气泡和通信连线。

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

候选为工程示例 Town04 路网的十字路口 **924**。四条入口为 -17.0.00、10.0.00、18.0.00、-9.0.00，入口长度约 36–69 米，适合小范围相遇测试。车位沿两个垂直入口向上游取 25 米。

| 对象 | CARLA x / y（米） | yaw（度） |
|---|---|---|
| Lincoln A（车道修正后） | 199.390 / -204.544 | 91.314 |
| Lincoln B（车道修正后） | 235.598 / -172.816 | 180.326 |
| RSU（对角路侧草地、红砖墙前） | 193.000 / -179.000 | 固定 |

配置种子坐标集中在 town04.json。车辆启动时使用当前 CARLA 驾驶车道 waypoint 修正高度和方向；检查横向偏差不超过 4 米、方向差不超过 35 度且不在路口内部，否则停止。RSU 在 OMNeT++ 中是固定无线节点；CARLA 同位置显示橙色塔架、青色顶部框和圆环、黄色天线及 各自的 RSU 名称（如 `rsu0`） 文字。该标记是世界调试绘图，无实体碰撞，不作为传感器可检测物体。当前基座已核对位于原路口对角的草地、红砖墙前；四个城区道路采样点到塔顶的视线检测无障碍，约 211 米外实景可见顶部标识。

RSU 的唯一位置定义是 `town04.json` 的 `rsus` 列表，单位为 CARLA 世界坐标米。`bash vanet/run_omnet.sh` 每次启动读取 JSON，自动换算并传入 OMNeT++，将同一配置快照交给共享桥：CARLA 显示塔架，SUMO 显示橙色 RSU 图标。单独启动车辆不会显示 RSU；关闭 OMNeT++ 窗口或在启动终端按 Ctrl+C 后，SUMO 图标移除，CARLA 绘图在恢复推进后的两个步长内消失（默认约 0.1 仿真秒）。暂停 Qtenv 不会移除标记。修改 JSON 后重开 OMNeT++ 即可，运行中的会话不热更新。

显示由现有桥线程统一处理，不再使用独立常驻标记进程；会话进程异常退出也会触发清理。首次升级此功能时，旧共享桥需要重启一次：正常关闭车辆后，在工程根目录执行 `bash run_lincoln.sh --stop-sumo`，再重新运行 VANET 车辆启动器。无需重启 CARLA 或修改地图。

桥传输的 SUMO 坐标使用 x=CARLA.x+503.02、y=-CARLA.y+423.76。OMNeT++ 内统一转换为 x'=x-xMin+40、y'=yMax-y+40，修正 Qtenv 画布 Y 轴向下与 SUMO Y 轴向上的差异；道路、车辆、RSU 和通信连线使用同一坐标系，距离保持不变。

roads.txt 包含 `carla-source/Co-Simulation/Sumo/examples/net/Town04.net.xml` 的全部 632 条车道及路口内部连接。`build.sh` 自动调用 `export_roads.py` 生成该文件，首行记录源路网边界。它是全图静态道路背景，车辆位置实时更新；Qtenv 使用 Ctrl + 鼠标滚轮缩放，通过滚动条移动视野以寻找路口。第一版信道仅包含路径损耗和噪声，不自动导入 CARLA 建筑物。

RSU 当前基座绝对高度为 0.39 米，主体顶部/通信节点绝对高度为 60 米，装饰天线和光环最高约 68 米。顶部光环半径为 5 米，用于远处识别；每个列表项的 `base_z`、`beacon_radius` 控制这些显示尺寸。建筑和树木仍可能遮挡个别视角。

## 添加多个 RSU

已扩展完成并编译通过，现有高塔保留为 **`rsu0`**，位置和高度不变。

以后在 [town04.json](town04.json) 的 **`rsus` 数组**中复制一项：

- 将 `id` 改为唯一名称，如 `rsu1`。
- 修改 `x、y、z` 及塔脚高度 `base_z`。
- 保存后重新打开 OMNeT++，OMNeT++、CARLA、SUMO 三个界面会统一创建；关闭 OMNeT++ 后全部清理。

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
- 保存后关闭并重新执行 `bash vanet/run_omnet.sh`，不必重新编译。删除列表项后重开即可移除对应 RSU；空列表 `[]` 表示不创建 RSU。

多个 RSU 分别使用真实 Veins 无线收发，均可转发收到的车辆原始广播；转发前随机等待 0–20 毫秒，减少同时收到同一包后同步发送造成的碰撞。不会继续转发其他 RSU 的转发包，避免循环。当前功能不包含跨 RSU 单播路由或接入切换。

首次升级列表功能，需要正常退出 OMNeT++ 和车辆，再在工程根目录执行 `bash run_lincoln.sh --stop-sumo`，重新启动车辆加载新桥；后续单纯修改列表无需重启车辆/桥。

## 启动

1. 按工程原说明启动 CARLA Docker，加载 Town04。运行中不切换地图。
2. 如果已有旧共享桥，先正常退出车辆客户端，再执行 bash run_lincoln.sh --stop-sumo。实验入口不会自动停止旧桥；不带 VANET_PORT 启动的旧桥无法提供网络接口。
3. 编译一次：

   bash vanet/build.sh

4. 两个终端分别启动以下命令，两辆车依次选择固定空闲车位 A、B：

   bash vanet/run_lincoln.sh

   每次命令只生成一辆车，需执行两次。VANET 入口计算车位坐标后，调用根 run_lincoln.sh --pose X Y Z YAW；不再向 ego_vehicle.py 传 --vanet-slot。

   首次启动的共享桥继承 VANET_PORT=9998，提供仅本机可访问的接口。自动车位使用进程锁和存活进程预留，并检查附近已有车辆，防止两个启动器同时选中同一车位。车位都占用时提示失败，不随机跳到其他道路。原 bash run_lincoln.sh 的随机出生行为保留。
5. 两辆车就绪后启动网络界面：

   bash vanet/run_omnet.sh

   在 Qtenv 点 Run（F5），展开 Town04Vanet，找到 rsu0、lincoln0、lincoln1。先观察静止通信，再用现有 Lincoln 面板低速驾驶。新车加入时也会自动创建 OBU。首次接入之后，每个 CARLA 步长内的网络事件处理完成才 ACK 放行下一步。

Qtenv 暂停会暂停共享仿真推进，这是同步联仿的预期行为。退出网络界面后，Python 桥检测断开，车辆仿真恢复独立推进。CARLA/桥停止时网络接口会报错，需重新启动网络实验。启动新车辆前请保持 Qtenv 正在运行，原共享桥就绪检查需要看到帧推进。

普通 Run 会播放详细动画，实际运行速度可能很慢；Fast 模式适合较快推进。默认 Live 实验在 120 仿真秒结束并释放桥连接，可用 `--sim-time-limit=300s` 覆盖时长。车辆编号按共享桥本次运行的生成顺序递增；销毁后新建车辆会取得新的编号。

OMNET_ROOT、VEINS_ROOT 可覆盖安装路径。VANET 端口默认自动读取 town04.json 的 network.port。`omnetpp.ini` 不再保存 RSU 坐标，修改位置只改 JSON。CARLA 非默认地址可给网络启动器传 `--carla-host=<地址> --carla-port=<端口>`。

## 看什么

每个节点约每秒发送一个广播 probe；RSU 收到车辆原始 probe 后只转发一次。这里是应用层广播连通性测试，尚未实现 MAC 单播、IP 路由、跨 RSU 或接入切换。

- 发送/接收节点显示 TX/RX 气泡。
- **实际收包成功**时画绿色直达连线；经 RSU 转发的接收画橙色连线，0.25 仿真秒后消失。未收到的包不会画成功连线。
- CSV 在 omnet/results/，包含发送方、接收方、原始来源、序号、仿真时间、端到端延迟及 direct/relay。
- .vec/.sca 记录收发计数、转发计数与延迟；.elog 支持事件回放。

转发报文保留原始序号和发送时间。发送方可能收到自己的 RSU 转发副本，分析时按 origin 排除；direct/relay 要分别计算，不能把所有 RX 相加当成投递率。CSV 每个模块实例独立文件，同一运行配置重跑会覆盖同名输出，重要结果先复制保存。

## 验证

不依赖 CARLA 的无线与增删测试：

   bash vanet/run_omnet.sh -u Cmdenv -c Smoke

真实 TCP 桥协议与合成车辆快照测试：

   python3 vanet/test_transport.py

Smoke 的第二辆车在 5 秒删除、6 秒重新生成。合成快照测试连接同一个 Python VanetBridge 类，与 OMNeT++ 执行逐帧 ACK，并检查 V2V、V2I、RSU 转发收包；它不会连接 CARLA 或启动 SUMO。

`~/carla-venv/bin/python vanet/test_scene_lifecycle.py` 使用独立的无界面 SUMO 和模拟 CARLA 绘图，验证 JSON 坐标传递、标记创建以及正常关闭、终止和崩溃时的清理。Smoke 配置与合成快照测试不显示真实场景标记；其他自建离线测试可显式传 `--no-scene-markers`。

## 原工程的接入改动

- run_lincoln.sh：无参数仍随机出生；显式校验并传递 --pose X Y Z YAW。
- vanet/launch.py：将实验车位转换为 --pose 后调用根启动器，SUMO 就绪检查只由根启动器执行一次；释放查询客户端后用关闭继承文件描述符的子进程启动车辆，父进程保持车位预留。
- lincoln/ego_vehicle.py：使用原有 --pose 接口，无 VANET 专用修改。
- carla-source/Co-Simulation/Sumo/run_synchronization.py：VANET_PORT 设置时启用本目录 TCP 端点，每步同步完车辆之后发送快照，退出时关闭端点。
- carla-source/Co-Simulation/Sumo/sumo_integration/sumo_simulation.py：支持按前缀独立编号，Lincoln 映射使用 lincoln0、lincoln1；网络节点沿用该名称。
- carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py：桥线程检查 OMNeT++ 会话，统一添加和移除 CARLA / SUMO RSU 标记。

其他新实现均在本目录；不启动第二套 SUMO、不建立第二个 TraCI 客户端，也不由 OBU 调用 world.tick()。

多 RSU 回归：`python3 vanet/test_multi_rsu.py` 验证两节点无线收发、空列表及无效配置；`~/carla-venv/bin/python vanet/test_scene_lifecycle.py` 验证两个标记同时创建和清理。测试结果使用独立临时目录。
