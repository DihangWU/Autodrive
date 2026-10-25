# Autodrive

CARLA、SUMO 和 CoBridge 分别运行。CARLA 负责三维场景、物理和传感器；SUMO 负责路网与交通仿真；CoBridge 负责时间推进、已登记 Lincoln 的 CARLA → SUMO 同步，以及 SUMO → CARLA 的 NPC 与红绿灯同步。RSU 保留现有独立进程，生命周期由它自己管理。详细基础架构见 [architecture.md](architecture.md)。

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

# 按需添加一批 SUMO 随机车流，并同步到 CARLA
add_npc -n 20
```

`run_cobridge`、`run_sumo`、`run_rsu`、`add_lincoln`、`add_npc` 保持前台运行，在各自终端按 Ctrl+C 退出。Lincoln 不会自动启动、关闭或切换 CARLA、SUMO、CoBridge，也不调用 `world.tick()`。SUMO 与 CoBridge 默认步长为 0.05 秒。

## SUMO 随机 NPC 车流

先启动当前地图对应的 CARLA、SUMO 和 CoBridge，再在独立终端执行：

```bash
source /home/maple/Desktop/Autodrive/setup.bash
add_npc                         # 默认申请 10 辆
add_npc -n 200 --seed 21         # 申请一批 200 辆随机车流
add_npc -n 20 --filter 'vehicle.audi.*'
add_npc --help

# 不加载 setup.bash 时的等价入口
bash modules/npcvehicules/npcvehicules.sh -n 20

# 有限时长运行，结束后清理本批车辆
add_npc -n 10 --duration 30
```

每次执行创建一个独立批次；Ctrl+C 请求删除本批两端车辆，随后释放批次连接。客户端异常退出时，桥也会检测连接断开并清理。退出 NPC 不关闭 CARLA、SUMO 或 CoBridge，不删除其他批次、Lincoln 或 RSU。正常退出桥时会清理其管理的 NPC；重启后需要重新执行 `add_npc`。桥或仿真器被强制杀死时不能保证立即完成清理。

NPC 批次业务由 `modules/npcvehicules/traffic/` 实现，调用桥端官方算法副本，并使用当前 SUMO 的通用 TraCI RPC 连接；入口通过现有 CoBridge 控制 socket 发起 `npc.start/status/stop` 请求。无需为 NPC 增加专用 SUMO 管理端、另一个 TraCI 客户端或额外时钟。`--bridge-socket` 可选择另一个已运行的桥；目标 CARLA 和 SUMO 由该桥的启动参数决定，NPC 入口不再单独选择一套地址。

```text
modules/npcvehicules/
├── npcvehicules.py          # 参数、批次控制、状态显示与退出
├── npcvehicules.sh          # 独立启动入口，无 ROS 环境依赖
└── traffic/
    ├── vehicle_types.py    # 读取官方车型 XML，按需加入 SUMO
    └── spawn_traffic.py    # 批次记录、分帧调度、归属和退出清理；调用官方衍生算法

foundation/cobridge/npcvehicules/
├── npcvehicules.py          # 批次归属、帧边界调用及清理协调
├── official_tools.py        # 加载官方 BridgeHelper、车型数据结构及 SUMO 工具
├── run_synchronization.py   # 官方同名脚本的 NPC 同步核心副本，移除独立时钟
├── spawn_npc_sumo.py        # 官方同名脚本的随机生成、道路末端续路副本
├── vehicle_sync.py          # 官方流程的 SUMO/CARLA 适配层、批量提交和贴地几何
└── config.json              # 每帧加入数量、总申请数量上限、车灯同步开关
```

`foundation/carla/carla-official/Co-Simulation/Sumo/` 原目录保持不改。将其 `run_synchronization.py`、`spawn_npc_sumo.py` 复制到 `foundation/cobridge/npcvehicules/` 后，保留并适配其中的 NPC 同步与随机车流算法，版权和 MIT 许可说明保留在副本头部。继续复用原目录中的 `examples/carlavtypes.rou.xml`、`BridgeHelper.get_carla_blueprint()`、`get_carla_transform()` 和 `get_carla_lights_state()`。只选择官方 XML 与当前 CARLA 都支持的四轮轿车，车型筛选使用通配符。随机种子控制车型、配色、出生道路和随机路线选择，不保证所有交通互动完全确定。

车辆命名为 `npc_<批次>_<序号>`，与 Lincoln 的登记和 CARLA → SUMO 投影分开。SUMO 决定 NPC 的跟车、换道、速度和信号灯通行；CARLA 代理关闭动力学，由桥提交姿态并保留实体显示，未配置额外传感器或联网功能。它们不是由 CARLA Traffic Manager 驾驶的车辆，也不是通过油门控制在 CARLA 中独立行驶的车辆。

车流生成和续接采用官方算法：随机选择车型及允许通行的出生道路，先建立单道路路线；行驶到当前路线最后一条道路时，从该道路允许的出口中随机选择下一条道路，将路线更新为“当前道路＋下一道路”。道路尽头没有出口时由 SUMO 正常驶离，并删除 CARLA 代理，不自动补车。旧的 32 道路预生成、提前续接算法及 `traffic/routes.py` 已删除，批次代码只调用官方衍生函数，不再保留第二套路线实现。

### 官方副本如何跟随 CoBridge 时钟

```text
CoBridge 帧边界
  → NPC 批次管理：分帧调用 spawn_npc_sumo.py 的生成函数
  → 读取 SUMO 上一已完成步骤的订阅及生成、驶离、传送事件
  → run_synchronization.py / SimulationSynchronization.synchronize_vehicles()
      → 官方顺序：订阅与生成 → 删除驶离车辆 → 更新姿态和车灯
      → vehicle_sync.py 的适配层合并 CARLA 命令并一次提交
  → 调用 spawn_npc_sumo.py 的道路末端续路函数
  → 返回 CoBridge，执行原有两端时钟推进
```

两个副本都没有独立 `main` 循环，不创建仿真器、不再次导出地图、不调用 `world.tick()` / `simulationStep()`，也不修改同步模式、创建 TM 或关闭仿真器。原脚本中的这些职责由现有 CoBridge 和 SUMO 管理进程继续持有。副本仅保留 SUMO → CARLA 的 NPC 阶段；原脚本的全场 CARLA → SUMO 扫描和信号灯接管不再执行，避免重复生成 Lincoln 或与现有控灯模块争抢控制。

适配层将当前本地 JSON RPC、帧订阅、批量提交和车型几何接到官方接口上；不逐车每帧查询路线、车型、灯光或位置。新增车辆首次订阅时可单独读取初始订阅结果，后续帧集中读取；实际路线和路线索引来自同一订阅帧，静态车型使用本批记录。SUMO 在路口可能为新路线补入前一条道路，因此不能用自己缓存的两条道路替代官方 `getRoute()` 的实际返回值。原官方入口仍可作为参考，但不应与 `run_cobridge` 同时启动另一套联仿时钟。

SUMO 自身可能因拥堵、碰撞或路线问题触发传送。新版分别处理生成、驶离及传送事件；传送中的同一 SUMO ID 不再因为暂时退出活动 ID 列表而销毁后重建 CARLA 代理。NPC 状态新增 `teleporting`、`teleports`，终端显示累计 SUMO 传送次数。传送结束后仍跟随 SUMO 的新位置，因此采用官方流程不等于禁止 SUMO 传送；这类源仿真位置跳变需与桥端更新异常区分。

公共 SUMO 启动入口 `foundation/sumo/init/sumo_init.py` 现在与官方 `SumoSimulation` 一样传入 `--lateral-resolution 0.25`，开启连续横向子车道运动，GUI 和 headless 模式均生效。参数集中在文件头部 `SUMO_LATERAL_RESOLUTION`；它是横向决策分辨率，不表示车辆每次横移 0.25 米，也不改变 0.05 秒时间步长。车辆换道会经过中间横向位置，降低默认离散换道造成的整车道横向跳步；它不禁止拥堵等原因触发的 SUMO 传送。细化横向决策会增加 SUMO 计算开销，需要单独衡量性能。

这个设置在 SUMO 进程启动时生效，已经运行的 SUMO 必须重启；建议先正常退出 NPC 和 CoBridge，再重启 SUMO、CoBridge，最后重新添加 NPC。CARLA、Lincoln 和传感器配置可保留。没有一起修改其他官方启动选项，例如路口碰撞检查。

独立 SUMO 换道对照：同一条 Town04 道路、5 m/s 速度、0.05 秒步长，分别采集 100 个步骤。未启用时最大单步二维位移约 3.50 米，出现一次超过 1 米的整车道跳步；启用 0.25 后成功换到目标车道，最大单步二维位移约 0.31 米，没有超过 1 米的步骤。此处位移包含纵向前进，不是横向分辨率的定义。已检查实际 GUI 进程携带新参数，Lincoln 保留原 actor，桥接入正常且 SUMO 步长仍为 0.05 秒；100 辆 NPC 连续运行 20 秒并正常清理通过。

本次官方衍生替换验证：从原官方源码提取同步与续路逻辑进行调用轨迹/结果对照，连同重试、传送保持身份、实际路口路线、贴地回退，共 11 项专项通过；15 项批次生命周期和 10 项贴地几何检查通过，原有 15 项 Lincoln 回归通过。真实 Town04 中两辆 NPC 可行驶，两个批次的归属清理通过；200 辆全部生成并清理，实际路线订阅后的 JSON 约 35 KB；100 辆连续运行 45 秒未再出现路线替换错误。官方原目录没有改动，测试不代表已经禁止 SUMO 传送或消除了所有可见跳变。

### 创建速度、同步顺序与状态

`foundation/cobridge/npcvehicules/config.json` 默认每帧最多向 SUMO 加入 2 辆、所有批次合计最多申请 200 辆，车灯同步开启。上限为当前本地 JSON RPC 回复容量预留空间：每帧集中读取一次全部 NPC 的少量动态订阅变量，避免逐车读取位置；提高上限前需要检查订阅回复大小及运行开销。无 NPC 批次时不额外查询车辆状态。

出生道路先检查是否允许该车型通行、是否有出口，以及道路长度是否至少为车长加 5 米；避免把车辆放进短于车身的 OpenDRIVE 路段后一直等待。SUMO 仍自行判断出生位置是否有空位，因此“申请/已提交”数量可能大于“SUMO 上路/CARLA 实体”数量；等待插入不会在 CARLA 创建空壳。状态还会显示等待插入和已驶离数量。路线、车型定义会保留在当前 SUMO 会话中，退出批次删除车辆实体，不重载 SUMO 地图。

现有桥仍是唯一时钟：在 CARLA 每次 tick 前，将 SUMO 上一已完成步骤的 NPC 姿态同步过去，然后执行原有 CARLA → SUMO 时序。动态姿态和车灯使用 CARLA 批量命令，所有 `apply_batch_sync` 都传入 `False`，不额外 tick。因此存在步骤边界上的一帧差异，不承诺两个 GUI 原子刷新。CARLA 或 SUMO 断开时停止该批继续创建，待可连接时清理；不会把旧 CARLA 世界的 actor ID 写到新世界，也不会自动把本批重新生成到另一地图。

日志位于 `.runtime/npcvehicules/npcvehicules.log` 和 `.runtime/cobridge/bridge.log`，SUMO 底层日志沿用 `.runtime/sumo/`。更新桥代码后需正常退出并重新运行 `run_cobridge`；`add_npc` 不会替你停止已有桥。

本轮 Town04 实测：两辆 NPC 都在 SUMO 和 CARLA 中生成并移动，约 1.2 秒采样区间内各移动约 5 米，非原子查询的位置对照误差最大约 0.22 米；两个批次同时运行时，断开第一批连接只删除第一批，第二批保留，其他连接不能删除不属于它的批次。修正过短出生道路后，200 辆在约 20 秒内全部上路并生成 CARLA 代理，动态订阅 JSON 约 30 KB；退出后两端 NPC 数量均为 0，删除前取消订阅后该批未再刷出旧车辆 ID 查询错误。另验证了创建过程中强制结束 NPC 客户端、有限时长退出，以及带有两辆 NPC 时正常退出桥：本批均清理，CARLA 和 SUMO 保留运行并释放时钟。清理仅取消实际建立过的订阅，删除失败后的重试不会重复取消同一订阅。桥先退出时，NPC 客户端会报告连接断开并退出。此结果验证加入和清理，不代表携带完整传感器时的 200 车性能保证，也未完成长时间拥堵或多地图压力测试。18 项临时 NPC 生命周期/路径专项检查及原有 15 项 Lincoln 回归通过。

### NPC 贴地高度与道路姿态

SUMO 转换路网中的高度和坡度不一定与 CARLA 实际车道完全重合，模型原点也不一定正好在轮胎底部。NPC 关闭动力学后，不会靠重力自行落地，直接套用 SUMO 高度会在部分坡道悬空或陷入路面。

`foundation/cobridge/npcvehicules/vehicle_sync.py` 保留官方转换得到的 XY 和 SUMO 航向，再用缓存的 CARLA 地图查询对应行车道，以道路局部平面修正 Z、俯仰和侧倾。换道时保留原航向，车身法线与路面法线对齐；车辆横向偏离车道中心时也按该道路平面计算高度。每种车型只读取一次实际 CARLA 模型包围盒的底部偏移，默认让模型底面距道路平面约 2 cm，不直接将车身高度的一半加到车辆原点。

立交桥选层时查询保留原始三维位置，不将高度设为 0；水平距离超过 6 米、原始高度与候选道路相差超过 2 米，或道路姿态异常时，不强行贴到该候选道路，而是使用官方转换得到的 SUMO 姿态并在 NPC 状态中报告。旧的“匹配失败后冻结，恢复后追上新位置”分支已删除。回退时不保证贴地，需要进一步核对路网；层级保护也不是完整的 SUMO/CARLA 车道编号映射。三个容差/间隙常量集中在该文件头部。

贴地计算不启用 NPC 动力学、不添加传感器、不改变 SUMO 跟车换道或仿真步长，也不修改官方目录。地图只缓存一次，道路查询在客户端本地完成，模型几何按车型缓存；每帧不逐车向服务器发送射线或物理参数查询。射线仅用于此次独立验证。

Town04 实测：200 辆 NPC 共 600 组姿态、其中 48 组带道路坡度，包围盒底面距道路平面约为 0.020 米，车身法线与道路法线夹角最大约 0.02 度。另抽查 16 处坡道的 CARLA 实际路面射线命中，模型底面与路面垂直间隙约 0～2.5 cm。10 项临时平面/车型偏移/层级保护检查及原有 18 项 NPC 生命周期检查通过。当前仍使用无动力学代理，包围盒贴地不是轮胎与悬挂动力学仿真。修改后需正常退出 NPC 与 CoBridge，再启动桥和 NPC；CARLA、SUMO 和 Lincoln 可保留。

## SUMO 自动控制两端红绿灯

正常运行 `run_cobridge` 即启用，无需额外手动切换。CARLA 和 SUMO 完整接入且地图指纹一致后，桥会读取本次 SUMO 路网 `.net.xml` 中的 `tlLogic/param linkSignalID:<连接索引>`，通过 OpenDRIVE 地标对应 CARLA 信号灯实体。实现位于 `foundation/cobridge/traffic_lights/sumo_traffic_lights.py`，复用官方 `SumoTLLogic` 和 `BridgeHelper.get_carla_traffic_light_state()`，不修改官方源码，也不另开 SUMO 连接或推进线程。

CARLA 0.9.16 的冻结接口作用于全场红绿灯，因此接管前必须覆盖全部 CARLA 信号灯实体，并检查 SUMO 控制器的映射；缺失的 actor ID、地标或控制器会明确报告，保持 CARLA 原控制，每秒重试。匹配成功后打印接管数量，冻结 CARLA 自主切灯；SUMO 继续执行现有程序，桥订阅实际灯色，每帧一次获取所有控制器的订阅结果，并只向发生变化的 CARLA 灯写入状态；退出时还原接管前的 SUMO 订阅变量。后续通过 SUMO 手动切相位、切程序或设置灯色也会被读取。不同程序对同一连接的地标映射必须一致，运行时新增或改写路网映射需要重启桥。

保留原有推进顺序：在 CARLA 每次 tick 和 TM 控制计算之前，应用 SUMO 上一已完成步骤的灯色，再推进 CARLA 和 SUMO。因此这里是按 0.05 秒步长耦合，不是两端渲染画面的原子刷新；SUMO 本轮发生的相位变化会用于下一次 CARLA 帧。SUMO 自主车流依据 SUMO 信号灯通行，CARLA TM 依据同步后的 CARLA 灯行驶；手动驾驶及自定义算法仍需自行遵守信号。

同一个 CARLA 灯若对应不同灯色的 SUMO 连接，会沿用官方保守取红灯规则，并报告冲突 actor ID；无法把多个车道方向的灯色无损显示为一盏实体灯。灯色枚举转换沿用官方规则，特殊的闪烁/右转信号可能转换为 Unknown，不能等同于普通绿灯。

SUMO 失联、桥主动脱离或 Ctrl+C 正常退出时，恢复保存的 CARLA 灯色和接管前的全场冻结状态，原来的红黄绿时长未修改；CARLA 世界已切换时不会向旧世界写入。恢复的是原灯色和冻结状态，不回放接管期间的 CARLA 相位，也不精确重置内部计时器。信号灯 RPC 失败时会尝试恢复并报告；SIGKILL、CARLA 崩溃等情况下无法保证执行清理。桥状态查询的 `traffic_lights` 字段可查看接管状态、覆盖数量和冲突信息。

Town04 实测匹配 43/43 个 CARLA 红绿灯、12 个 SUMO 控制器，手动改变 SUMO 相位后 CARLA 灯色随之改变。灯色切换边界曾观察到一帧快照差异，随后连续 29 帧、1247 次比较一致；RPC 写入和快照更新不是原子操作。正常退出后，CARLA 恢复原先异步运行和未冻结状态；SUMO 脱离后恢复、重新接入后再接管均通过。订阅读取版本在空载 Town04 的桥步进耗时中位数约 20.6 ms，取代逐个管理请求约 77.6 ms 的实现；有车辆和传感器时需另行测量。离线回归覆盖缺失映射、程序映射冲突、同灯多连接冲突、读取/写入失败恢复、原冻结状态保留及地图切换。启动中的旧桥不会加载此功能，需正常退出后重新运行 `run_cobridge`。

## SUMO 管理请求的就绪等待

`foundation/sumo/init/sumo_init.py` 的管理循环使用 socket 可读事件等待：有新连接、请求数据或客户端断连时立即继续处理，空闲时最多等待原来的 5 ms。若接收缓冲区已有完整请求，直接继续处理；收到退出信号后不再进入等待。启动阶段检查 SUMO 端口是否就绪的等待方式保留。

原循环无论是否有请求都会固定等待 5 ms，CoBridge 每帧连续调用多个 TraCI 接口时，这些等待会逐次累积。此次只调整管理端的等待方式，不新增进程、线程或目录，不改变 JSON/TraCI 接口、时钟归属、0.05 秒步长、传感器配置或 NPC 同步频率。空闲等待仍有超时，因此可以继续检查 SUMO 进程退出、自主推进和终止信号。

在独立临时 socket 和真实 headless SUMO 中，各执行 100 次简单查询的对照测试：原等待方式的中位耗时约 5.20 ms，更新后约 0.049 ms；验证了外部时钟接管期间不会自主步进、每次显式步进增加 0.05 秒、其他连接不能抢先推进、不同步长请求被拒绝，以及持有者断连后恢复自主推进。另有 5 项临时 socket 专项检查通过，覆盖新连接、已有连接数据、缓冲区中第二条请求、客户端断连和退出状态。测试使用独立 SUMO，没有操作当前联仿场景。

修改前，200 辆 NPC 加 1 辆 Lincoln、31 个传感器的约 8 秒采样为 10.1 帧/秒、约 0.51 倍实时速度。重启应用后，同样数量配置的约 12 秒采样为 16.8 帧/秒、约 0.84 倍实时速度，帧间隔中位数约 59 ms、最长约 76 ms。真实联仿中的简单时间查询中位耗时约从 5.23 ms 降至 0.12 ms，车辆 ID 查询约从 5.39 ms 降至 0.35 ms，全部 NPC 动态订阅查询约从 5.41 ms 降至 0.76 ms。两端均为 200 辆 NPC，Lincoln 的 CARLA → SUMO 投影和全部 43 个红绿灯同步保持正常，步长仍为 0.05 秒、原生传感器仍为 31 个。NPC 重建后的交通位置不同，因此这些是同数量条件下的实测对照，不是同一场景状态的严格回放，也不保证固定提升幅度。

已运行的 SUMO 管理进程不会热加载 Python 改动。应用到当前联仿时，先正常退出 NPC、CoBridge，再退出 SUMO；保留 CARLA 和 Lincoln。随后依次在独立终端重新运行原来的 `run_sumo`、`run_cobridge`、`add_npc -n 200`。SUMO 会话会重建，NPC 从新批次重新生成；Lincoln 保留在 CARLA 中，待桥恢复后重新登记并投影到 SUMO。无需重新启动 CARLA 或更改地图。

## SUMO GUI：低延迟、双坐标复制、仿真倍率、Lincoln 跟踪与导航

五个功能同时集成到同一个项目 SUMO GUI，仍只执行 `run_sumo`，不需要单独启动面板或功能脚本。

```text
foundation/sumo/init/
├── sumo_init.py                  # SUMO 进程、TraCI 管理和启动
├── sumo_init.sh
└── gui/
    ├── build_sumo_gui.py         # 统一构建、缓存、编译与链接
    ├── low_latency_sumo_gui.py   # 缩短 GUI 额外重绘等待
    ├── coordinate_sumo_gui.py    # 双坐标状态栏和两种坐标复制
    ├── simu_speed_sumo_gui.py    # 仿真时间相对真实时间的倍率
    ├── lincoln_track_sumo_gui.py # 按车辆增删动态生成 Lincoln 跟踪按钮
    └── lincoln_navigation_sumo_gui.py # 右键导航、地图选点与异步桥请求
```

调用链：`run_sumo` → `sumo_init.sh` → `sumo_init.py` → `gui/build_sumo_gui.py` 的 `ensure_sumo_gui()` → `.runtime/sumo/gui/sumo-gui`。各功能文件仅通过 `source_patches()` 提供源码修改，不编译、不启动进程，也不推进时钟；构建器合并同一 C++ 文件的全部修改后只编译一次，避免覆盖。例如双坐标和倍率对 `GUIApplicationWindow.cpp` 的修改会一起生效。旧的 `init/build_low_latency_sumo_gui.py` 已移除，不保留旧入口或另一套构建流程。

### 界面功能

- **低延迟**：保留既有 SUMO 1.22.0 修复，将 `GUIRunThread::tryStep()` 中 Linux 每秒一次的 `sleep(100)` 改为 `sleep(10)`，减少 GUI 重绘等待阻塞 TraCI。它不改变 0.05 秒仿真步长。
- **双坐标**：鼠标位于地图时，底部原 XY 位置显示 `SUMO x:… y:… | CARLA x:… y:…`。偏移实时取自当前加载路网的 `netOffset`，转换为 `CARLA x = SUMO x − offset_x`、`CARLA y = offset_y − SUMO y`；不修改路网坐标或翻转视图。
- **右键复制**：地图右键菜单提供 `Copy SUMO coordinates` 和 `Copy CARLA coordinates`，复制六位小数的 `x, y`。使用右键打开菜单时保存的地图位置，鼠标之后移到菜单上不会改变复制坐标；原有地理坐标菜单保留。
- **仿真倍率**：底部固定显示 `Sim/real: 0.85x` 等数值，表示仿真秒数÷真实秒数；使用单调时钟统计最近约 2 秒，每约 0.5 秒更新。样本不足时显示 `--`，约 1.2 秒无新步骤后显示 `0.00x`；重新加载、开始和停止时重置统计。倍率是 GUI 观测到的平均值，不是某一模块的独立 CPU 耗时。
- **Lincoln 跟踪**：按钮与底部 SUMO/CARLA 双坐标显示位于同一状态栏，为每辆已同步到 SUMO 的 Lincoln 显示一个按钮（如 `lincoln0`、`lincoln1`），不再占用顶部独立工具栏。点击后，该车立即位于当前活动视图中央，并随车辆持续移动；保留当前缩放比例。正在跟踪的按钮保持按下状态，文字切换为 `Stop tracking`，再次点击即可取消并恢复车辆名称；悬停提示仍显示对应车辆名称。点击另一辆车直接切换跟踪目标，前一按钮恢复。车辆删除后，对应按钮在约 0.5 秒内移除；没有车辆时不显示车辆按钮。识别依据为现有代理的 `autodrive.lincoln.owner=lincoln:…` 归属参数，NPC 即使使用 Lincoln 车型也不会产生按钮。多个 SUMO 视图中，按钮只操作当前活动视图，并定时同步该视图的原生跟踪状态。

跟踪功能只在 GUI 内每半秒检查车辆列表，增删按钮时更新布局；持续跟踪使用 SUMO 原有机制，不创建额外 TraCI 连接、不查询 CARLA，也不改变 CoBridge 的时钟。功能代码集中在 `foundation/sumo/init/gui/lincoln_track_sumo_gui.py`，由现有构建入口加载，无须单独运行。已运行的 GUI 需要正常重启 SUMO 才能加载。

Lincoln 退出／重启导致按钮增删时，会立即更新状态栏及按钮容器布局并请求重绘；即使新旧按钮数量相同，也不等待窗口缩放。最后一辆 Lincoln 退出后隐藏按钮容器，重新加入后恢复显示，避免残留空白占位。此修改只涉及 GUI 显示，不改变车辆生命周期、通信或联仿时钟。已编译并在独立界面验证连续等数量替换、全部删除、重新加入及点击跟踪。用户原始短暂空白未在独立窗口稳定复现；当前正在运行的旧 GUI 已自行恢复，仍需下次正常重启 SUMO 加载修复后观察。

### 在 SUMO 选择 Lincoln 导航目的地

左键 Lincoln 按钮的 tracking 行为保持原样：点击居中并持续跟踪，再点取消。**右键同一个按钮 → `Navigate here` → 在当前 SUMO 地图上左键点击目的地**；Esc 取消选点。选点的点击及释放单独处理，不会触发原地图的拖动、选择或改变 tracking 状态。导航菜单及其提示统一使用英文，使用 SUMO 原有字体，不额外加载中文字体。

菜单只是进入选点模式，“此处”最终指地图点击的位置。选点和导航不自动打开或关闭 tracking；原先正在跟踪的车辆仍继续跟踪。状态栏和车辆按钮悬停提示显示 `planning`、`navigating`、`arrived`、`cancelled`、`unreachable` 或 `error`。多车分别保存任务与状态，按车辆身份转发，车辆退出后不能把旧任务应用到新车辆。GUI 在后台限时发送请求，状态轮询约每秒一次，多车轮询交替进行；不会在窗口主线程等待桥回复。

新增功能的职责如下，其他入口仅负责接线：

```text
foundation/sumo/init/gui/lincoln_navigation_sumo_gui.py
    右键菜单、地图选点、Esc 取消、异步请求与英文状态提示
foundation/cobridge/lincoln/navigation.py
    校验地图与车辆身份、匹配可行驶车道、SUMO→CARLA 坐标转换与任务转发
modules/lincoln/driving/navigation.py
    后台规划 CARLA 合法有向车道路线、驾驶线程执行、接近减速与到达停车
```

通信链路：**SUMO GUI → CoBridge 本地控制 socket → 既有 Lincoln 登记连接 → 本车驾驶控制器 → CARLA**。路线和传感器时钟保持现状，新增规划工作线程不调用 `world.tick()`。桥不拥有车辆，也不直接向它发送油门；CARLA 默认控制器通过 Traffic Manager 的 `set_path` 执行路线，交通灯和避障仍由 TM 负责。到达附近时导航通过现有控制出口减速，到达同一目标车道、距离约 4 米且车速低于 0.3 m/s 后关闭 TM 并保持刹车。目标后的路线点只为 TM 前瞻提供缓冲，不会让车辆继续驶过目标。

点击位置必须距可行驶车道 12 米以内，桥从**当前实际 SUMO 路网**匹配车道中心并保留高度，转换使用实际 `netOffset`。上下层道路二维重叠且高度存在歧义时拒绝猜测，提示在未重叠路段重新选点。Lincoln 再用 CARLA 道路拓扑检查可达性，只允许道路前进及合法同向换道，不会打通隔离带或修改地图；无法找到路线时提示 `unreachable` 并停车等待。

每辆车同一时间只有一个导航任务；新目的地替换旧任务。规划结果带任务 ID，手动接管后即使旧规划稍后完成，也不会重新接管。面板／键盘驾驶或明确关闭自动驾驶会取消导航；保持刹车状态下手柄输入可以接管，导航运行时手柄轴不会打断自动驾驶。新导航规划期间先停车等待，规划完成后自动驾驶。自定义 `--driver` 可实现 `set_route(locations)` 和 `clear_route()`，未实现时明确报错，不回退成随机驾驶。

正常启动命令不变：先运行地图匹配的 SUMO 与 CoBridge，再 `add_lincoln`。这次升级涉及三个长驻进程，已有的 SUMO GUI、CoBridge、Lincoln 都需要正常退出后重新启动才能加载；不需要重启或切换 CARLA 地图。GUI 默认连接本工程 `.runtime/cobridge/control.sock`；使用自定义桥 socket 时，在启动 SUMO 的终端设置：

```bash
export AUTODRIVE_BRIDGE_SOCKET=/absolute/path/to/control.sock
run_sumo
```

导航读取的路网路径必须与桥接入的 SUMO 相同；更新后若仍是旧版 Lincoln，导航会提示重启，不会把命令发给其他车辆。

导航验证：29 项 Lincoln 检查通过，其中新增检查覆盖坐标与高度、远离道路／重叠层拒绝、地图及车辆身份隔离、过期状态拒绝、有向路线不可逆行、手动接管及迟到规划取消、到达停车和刹车保持。独立嵌套 X11 中验证了英文菜单、选点请求、Esc 取消和 tracking 目标不变；通过实际桥请求处理器与实际 Lincoln 登记线程完成 GUI→桥→后台规划→驾驶接口→到达状态回传。CARLA Town04 多组只读路线规划约 6～74 ms；另使用临时无传感器 Lincoln、复用当前 CoBridge 时钟完成约 45 米导航，车辆实际前进、减速并到达停车，随后清理该车和登记。全城任意交通条件下的长距离通行仍取决于地图连接、信号灯和 TM 行驶行为。

### 构建环境和缓存

需要 Linux、本机 SUMO 完整源码、已完成的 GUI CMake 编译产物，以及原编译器和 `ar`。当前环境示例：

```bash
export SUMO_HOME="$HOME/sumo-1.22.0"
# 可选：默认即为 $SUMO_HOME/build/cmake-build
export SUMO_BUILD_DIR="$SUMO_HOME/build/cmake-build"
run_sumo
```

只在项目内复制并编译 5 个源文件：`GUIRunThread.cpp`、`GUIApplicationWindow.cpp`、`GUISUMOAbstractView.cpp`、`GUIGlObject.cpp`、`GUIGLObjectPopupMenu.cpp`。分别使用对应 CMake 目标的 `flags.make`，在 GUI、窗口和图元三个静态库的副本中替换对象，再复用原 `CMakeFiles/sumo-gui.dir/link.txt` 链接项目程序。系统 SUMO 的源码、静态库和 `bin/sumo-gui` 不改写。

构建器和全部五个功能文件的内容、实际修改文本、原源码、编译/链接参数、依赖库/对象信息都参加缓存校验，任一功能变化都会触发重新构建。每个修改锚点必须唯一，缺失或版本不匹配时明确报错，不静默忽略功能或退回系统 GUI。构建锁避免并发写入，编译与链接完成后原子替换项目程序，随后更新签名；编译或链接失败保留原程序和签名。只有发行版可执行文件、没有源码或 CMake 产物时不能进行这种构建。无变化时直接复用缓存。

```text
.runtime/sumo/gui/
├── sumo-gui          # 同时包含五个功能的项目可执行文件
├── build/            # 5 个修改后源码副本、对象、3 个库副本
├── build.log         # 编译、链接及错误输出
├── build.lock
└── signature
```

无窗口模式仍使用原生 `sumo`，跳过全部 GUI 构建：

```bash
run_sumo --headless
```

需要提前构建时，在工程根目录执行：

```bash
~/carla-venv/bin/python foundation/sumo/init/gui/build_sumo_gui.py
```

已运行的旧 SUMO GUI 不会热加载新的可执行文件。新功能在下次 `run_sumo` 时生效；如果当前有联仿车辆，应先正常退出 NPC、CoBridge 和 SUMO，再启动 SUMO、CoBridge 并重新添加 NPC。首次构建时间不计入随后 SUMO 进程的启动等待。

### 验证记录

状态栏布局已重新编译并通过独立界面验证：Lincoln 按钮与双坐标同栏，顶部不再占用一行；使用保持按下外观的切换按钮。连续点击同一按钮，查询到原生跟踪目标由 `lincoln0` 变为空；点击另一按钮后目标切为 `lincoln1`，原按钮恢复名称。

Lincoln 跟踪已与原有功能一起编译通过，并在独立嵌套 X11 窗口中验证：两辆带归属标记的 Lincoln 分别生成按钮，普通 NPC 不生成；点击后通过 TraCI 查询确认原生跟踪目标正确，车辆移动时视图中心随之移动，切换和取消正常。删除正在跟踪的车辆后，原生跟踪解除、对应按钮消失；删除后同名重建也可重新跟踪。6 项构建检查再次通过，覆盖新增功能合并与缓存签名。测试未重启现有联仿会话。

三个功能已一起编译进同一个程序，并在独立隐藏的嵌套 X11 测试界面中检查，不移动用户鼠标或读取、改写用户剪贴板。状态栏双坐标和右键两个复制项同时可见；同一右键位置分别复制出 `452.460000, 487.814929` 与 `-50.560000, -64.054929`，与本次 Town04 的实际路网偏移量一致。按不同真实间隔驱动私有 TraCI，界面分别显示 `0.50x`、`1.00x`、`2.00x`，暂停后显示 `0.00x`。6 项临时构建检查覆盖功能合并、原源码锚点、真实缓存命中、功能文件参与签名、构建失败保留旧程序以及未知版本拒绝替换。

低延迟功能的既有独立对照中，80 次预热后步进调用，原版最长等待约 100.3 ms、4 次超过 50 ms；项目修复版最长约 10.3 ms、没有超过 50 ms 的调用。GUI 倍率仍受整个联仿链和显示事件处理影响，不保证任意场景下无卡顿。

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

手动按住油门按钮或驾驶模式下按 W 时，前进油门为 1.0，倒车保持 0.45；可在 `modules/lincoln/driving/manual_driving.py` 文件头调整两档油门。这里设置的是油门比例，实际加速度由车辆物理模型决定，定速与自动驾驶控制保持各自原有逻辑。

### Xbox 手柄驾驶

手柄逻辑集中在 `modules/lincoln/driving/xbox_controller.py`，使用 Linux 自带 `/dev/input/js*` joystick 接口，不需要 pygame/evdev、不新增线程、不推进时钟。默认启动车辆即寻找一只空闲 Xbox 手柄；未连接手柄不会影响原有面板、键盘和自动驾驶。

| 手柄输入 | 驾驶指令 |
|---|---|
| RT | 油门 0～1，按压量线性映射 |
| LT | 刹车 0～1，按压量线性映射 |
| 左摇杆左右 | 跟随本车手动驾驶时转向 -1～1；中心 8% 为死区，其余行程线性映射 |
| 左肩键 LB（左扳机上方） | 切换前进／倒挡；仅跟随本车手动模式，车速不超过 0.5 m/s 时允许 |
| A | 开启／关闭自动驾驶；扳机、左摇杆及 LB 不会打断已开启的自动驾驶 |
| 右摇杆按下（R3） | 切换自由跟随／跟随本车；进入自由跟随时自动开启驾驶，返回跟随本车时保留当前驾驶状态 |
| 自由跟随中的左摇杆 | 相对当前观察朝向水平前后左右移动；不随俯仰改变高度，幅度线性控制速度 |
| 自由跟随中的右摇杆 | 左右连续转头、上下俯仰；yaw 可连续转满 360°，pitch 限制在 ±89° |
| 自由跟随中的 RT / LT | 相机上升／下降，按压量线性控制升降速度；同时按下时升降量抵消，不作用于车辆 |

车辆手动轴输入只在本车拥有相机控制权且处于“跟随本车”的手动驾驶模式下生效。自由跟随时，左摇杆、右摇杆与扳机改为控制相机；车辆仍由自动驾驶或面板控制。普通“释放视角”不启用手柄相机控制。相机被另一辆车接管时，本车的手柄模式按键及相机、驾驶输入均暂停。

首次连接、重新连接或从自动驾驶／自由视角返回手动驾驶后，需先松开扳机并回中左摇杆，避免恢复时突然执行旧输入；**正常面板与手柄切换不要求回中**。按键按下边沿只触发一次，初始化或重连时已按住的按钮不会触发模式切换。LB 换挡与面板共用同一函数；UI 档位、自动驾驶和自由跟随按钮会同步更新。

通过右摇杆或面板进入自由跟随时默认开启自动驾驶；再次按下右摇杆只恢复跟随视角，不自动关闭驾驶。可以用 A 或面板明确关闭自动驾驶。自由跟随中相机仍随车的位置移动，朝向由手柄／面板独立调整，左摇杆沿当前 yaw 水平前后左右移动，RT 上升、LT 下降；相机俯仰不影响左摇杆的水平移动方向。左右转头上限 180°/秒、俯仰上限 45°/秒，均随摇杆幅度线性变化；移动速度与右侧面板滑轨同步，均由文件头常量提供初始值。面板视角操作优先于手柄视角操作，松开后恢复手柄当前输入。

面板与键盘按住操作时优先，松开后立即恢复手柄当前输入；手柄有效输入会退出原有定速控制，手柄回中、松开扳机后恢复收油滑行。手柄输入与面板共用 `drive()` 和驾驶锁，只有一个 CARLA 控制输出路径，不会由两个线程各自发送冲突命令。A 开启的自动驾驶不会被手柄的扳机、左摇杆或 LB 打断；**面板及键盘手动操作优先级最高，仍可立即关闭自动驾驶并接管**。断连后清除输入，并自动寻找空闲手柄。

面板的 Xbox 状态只保留设备与当前模式／就绪信息，车辆及相机操作统一由对应按钮的按下提示表示，不再额外显示第二行轴值。手柄实际控制车辆时，油门、刹车、左右转向按钮同步显示蓝色按下提示，不再显示第二行输入比例文字，以免与底部按钮行重叠；自由跟随时，右侧相机前进／后退、左移／右移、上升／下降、左右转头及抬头／低头按钮显示对应按下提示。使用面板时恢复其原有按压提示。

多车默认各占用一只空闲手柄，使用 `.runtime/lincoln/xbox/` 下基于设备号的进程锁防止一只手柄同时控制两辆车，锁在退出或异常退出后由系统释放。设备锁文件本身不需要删除，也不能靠删除锁文件抢占设备。配对不因暂时切换视角而释放。自动分配取决于启动顺序；需要固定配对时，给每辆车指定不同设备：

```bash
add_lincoln --xbox-device /dev/input/js1
add_lincoln --xbox-device /dev/input/js2
```

`js1`、`js2` 为示例，请以本机实际设备为准；手柄编号可能随重连变化。有稳定的 `/dev/input/by-id/...-joystick` 链接时也可以传入该路径，不能使用 `...-event-joystick` 的 event 接口。指定设备已被占用或未连接时，本车等待该设备，不自动改配别的手柄。CARLA 同一世界只有一个 spectator，现有“跟随本车”资格仍取决于该车是否持有相机控制权；多只手柄的配对互不冲突，不表示多辆车能同时占有同一个 spectator。

设备需要被运行 Lincoln 的 Linux 主机识别并允许当前用户读取；远程桌面电脑上的手柄必须先透传到该主机，不能只在远程客户端连接。

验证记录：已读取本机 Xbox Wireless Controller 的蓝牙轴映射，确认其使用 GAS/BRAKE 作为右／左扳机；两个实例及跨进程占用检查均未发生重复配对，释放后可重新分配。9 项独立检查覆盖线性映射、USB/蓝牙映射、首次回中、模式禁用与恢复、断连、面板优先与即时恢复、定速接管以及倒车面板油门；原有 15 项 Lincoln 回归检查通过。独立隐藏 Tk 窗口中验证了手柄油门、刹车、转向对应按钮的蓝色按下提示和状态数值。此次未启动车辆做实际手柄行驶验证。后续按键与相机扩展另通过 11 项输入事件检查，覆盖 A/LB/R3 按下边沿、按住不连发、初始事件忽略、自动驾驶不被手柄轴打断、面板明确接管、自由跟随模式往返、相机输入与驾驶隔离、相机前后移动、相对观察方向横移、独立升降、双扳机抵消及转头限制。已只读核对实机 A/LB/R3 的 Linux 按键映射，未向当前车辆发送测试控制。

Lincoln 只打开一个综合面板，左侧为车辆驾驶区，速度及指令状态位于驾驶区底部，与右侧操作提示的最后一行底线对齐；两区随窗口高度伸展并保留相同的底部留白。右侧键盘提示保持单行显示。无需另行启动 `foundation/carla/mouse_move/`；后者的原代码保持不变，Lincoln 直接复用其控件布局、水平移动计算、视角预设和调速功能，不启动该工具自己的窗口或相机进程。

| 模式 | 键盘与面板行为 |
|---|---|
| 跟随本车 | WASD 驾驶，Q 切换前进/倒车；保留油门、刹车、转向、目标速度、右键定速及自动驾驶按钮。右侧自由视角区整体隐藏，窗口随之收窄。 |
| 点击“释放视角”后 | 右侧自由视角区展开；面板内 WASD 控制相机水平前后左右移动，Q 下降、E 上升，Shift 临时加速；不把这些按键发送为驾驶指令。车辆驾驶按钮及自动驾驶功能仍可使用，释放视角本身不关闭自动驾驶或定速。 |
| 点击“自由跟随”后 | 右侧自由视角区展开，WASD/QE、转向、俯仰、预设和调速均可使用。相机随本车的世界坐标位移一起移动（含高度变化），不会因车辆转向自动旋转；手动移动相机后保留新的观察偏移，朝向始终由用户控制。按钮保持按下，再点一次取消自由跟随，回到原有释放视角模式。 |
| 点击“跟随本车”后 | 取消自由视角的临时操作及自动前进，恢复跟随和车辆键盘驾驶。 |

“释放视角”原有行为保持不变；“自由跟随 / 释放视角（或跟随本车）/ 贴靠 CARLA”三个按钮在左侧底部横向排列，等宽等高；自由视角区展开时，与右侧“速度− / 速度+ / 停止”一行对齐，按钮尺寸取自右侧按钮。自由跟随基于现有相机更新线程和 CARLA 快照，在相机锁内叠加本车每帧的位置变化，与面板的手动调整共用 `free_pose`；不新增线程、时钟或依赖，也不改独立自由视角工具。相机控制权被另一辆车接管时会停止本车的自由跟随。重新启动 Lincoln 后可使用新按钮，CARLA、SUMO 和 CoBridge 无须为此重启。

底部按钮布局已在独立隐藏界面中检查：右侧展开时两组三键逐列宽度一致、顶部及高度一致；右侧收起时左侧三键仍可见，并为其预留独立高度，不遮挡驾驶控件或速度文字。

自由视角区提供左转、前进、右转、左移、后退、右移、升降、抬头低头、恢复水平、45° 看地面、速度滑块、加减速及停止。“贴靠 CARLA”位于左侧底部三键行的最右侧，在跟随、自由视角和自由跟随模式下均可使用。展开自由视角时，左侧三键与右侧速度减、速度加、停止三个按钮尺寸相同并处于同一水平行；右侧三个按钮等宽铺满滑轨下方。左侧速度及指令文字保持在驾驶区底部居中，不与贴靠按钮挤在同一行。前进按钮右键开启/取消持续前进；持续前进中转向、调速、切换视角预设和失焦不取消自动前进，后退、停止或切回车辆跟随会取消。任意俯仰角度下前后左右均水平移动，45° 俯视是一次性预设。

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

# 关闭全部传感器和 RViz，保留驾驶面板、手动/自动驾驶及视角控制
add_lincoln --no-sensors

# 只生成 LiDAR，不打开面板和 RViz
add_lincoln --sensors lidar --headless --no-rviz

# 纯车辆连通性测试：显式关闭传感器、面板和 RViz
add_lincoln --sensors none --headless --no-rviz

# 运行 10 秒后自动清理（运行时长从初始化完成后计）
add_lincoln --headless --no-rviz --duration 10

# 使用自己的 RViz 布局
add_lincoln --rviz-config /完整路径/my_layout.rviz
```

`--no-sensors` 复用现有的 `--sensors none --no-rviz` 执行路径，不创建任何 CARLA 传感器，也不打开 RViz；默认仍保留驾驶面板、手动驾驶、定速、自动驾驶、跟随/释放视角和桥登记同步。该开关优先于同时指定的 `--sensors all/lidar`，不隐式开启 `--headless`，也不跳过车辆控制需要的 ROS 初始化。不带这个参数时仍默认创建 31 个传感器并打开 RViz；原有 `--sensors none` 单独使用的行为保持不变。 默认、无传感器、组合自动驾驶、与 `--sensors lidar` 同时指定及原有单独 `--sensors none` 五种参数组合已校验。Town04 实测新开关输出 `Sensors=0`，没有本车附属传感器或 RViz 进程，车辆仍成功登记并同步到 SUMO，定时退出清理通过。

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

## RViz 大点云与拼图传输缓冲

当画面偶尔停一下再跳动，而 CARLA 原生帧、车模及小雷达仍连续时，需要区分仿真停顿与显示端漏收大消息。一次 15 秒只读采样中，CARLA 原生帧最大间隔约 79 ms，显示 TF/车模约 100 ms 连续；LiDAR 显示订阅却出现约 200～228 ms 的接收间隔。诊断订阅者漏收不直接证明 RViz 每次都漏收，但说明同帧发布之后的传输环节仍可能破坏显示连续性。

本机使用 Fast DDS 2.14.7，原共享内存段为默认约 512 KiB，单帧 LiDAR 已接近或超过该容量，拼图约 6 MB。[官方文档](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html) 明确指出，段大小接近或小于数据大小时存在较高丢失风险。独立 ROS 域内模拟两路约 600 KB 点云加约 6 MB 拼图，每路发送 80 帧：默认配置收到点云 79/80、拼图 67/80；扩大缓冲后，三路均为 80/80。这不是对所有场景绝不丢包的保证。

`modules/lincoln/config/fastdds.xml` 配置每个参与者 64 MiB 的共享内存段、2048 项端口队列，并保留 UDPv4 发现及跨机通信，UDP 收发缓冲各请求 4 MiB。`add_lincoln.sh` 在 ROS 初始化前设置 `FASTDDS_DEFAULT_PROFILES_FILE` 和兼容的 `FASTRTPS_DEFAULT_PROFILES_FILE`，由 RViz 子进程自动继承。它只改变传输缓冲，不改变传感器/显示频率、Best Effort 策略、CARLA 或 CoBridge，也不修改系统 sysctl。共享内存传输并不等于 ROS 消息零拷贝，多车和多个 RViz 会增加对应内存开销。

已有用户 DDS 配置路径时不覆盖；显式选择非 Fast DDS 的 RMW 时不加载此配置。`--help` 不加载 ROS 或 DDS 配置。通过原 `add_lincoln`/`.sh` 命令正常退出并重新启动车辆即可应用，RViz 随车辆重新启动；CARLA、SUMO 和 CoBridge 可保持运行。直接执行 Python 入口会绕过 shell 的环境准备，需自行加载 ROS 环境并设置上述配置路径。默认加载、自定义配置保留、其他 RMW 跳过及帮助分支均已检查。

当前显示仍按约 10 Hz 同帧更新，扩大缓冲用于减少额外漏帧，不会把 10 Hz 数据变成 60 Hz 动画。网络丢包、RViz 渲染压力和原生仿真长帧需要分别排查。

重启后一次约 15 秒实测中，LiDAR、显示 TF、车模和抽查的小雷达各收到 146 帧，采集时间间隔连续为 100 ms，LiDAR 到达的最长间隔约 124 ms；新共享内存段已生效。但用户观察到的偶发停顿没有明显改善，因此缓冲调整只作为传输改进，不能视为卡顿修复。随后约 90 秒采样中，RViz 可见画面采样区域的最长变化间隔约 151 ms，CARLA 原生帧最长约 78 ms，尚未复现明显长停顿。诊断 ROS 订阅者曾出现一次约 390 ms 的回调间隔，同期画面仍更新，不能把诊断进程的间隔直接归因于 RViz。保持 10 Hz 显示；进一步定位需要捕捉实际停顿事件并对照消息和画面时间。 关闭相机拼图后用户仍观察到跳闪，因此不能把拼图显示认定为根因；暂不降低相机分辨率或修改显示频率。

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
│   ├── manual_driving.py           # 面板、手柄与定速的统一控制输出
│   ├── xbox_controller.py          # 线性手柄输入、设备独占配对与热插拔
│   └── navigation.py               # 导航规划、任务取消与到达停车
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
