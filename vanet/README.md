# Town04 紧急通行实验使用说明

本目录在原有 CARLA、SUMO、CoBridge、Lincoln、NPC 和 RSU 基础上增加紧急通行实验。方案约定见项目根目录 `description.md`；本文说明**当前代码提供的命令、操作流程和验证边界**，不代表所有对照实验已经验收。

目前使用真实 OMNeT++ / Veins 802.11p 无线收发，不使用 INET。CoBridge 统一协调仿真时间，车辆位置交给无线传播模型；RSU 业务只使用实际收到的消息，不从全局车辆列表读取交通状态。NPC 不转发紧急消息，只发本车信标与反馈。项目只使用一辆 Lincoln、一个 RSU；不增加车载雷达、摄像头、碰撞传感器或灯光传感器，交通状态与碰撞检查复用现有数据和几何信息。

## 1. 固定场景

| 项目 | 当前配置 |
| --- | --- |
| 地图 | Town04 |
| Lincoln 起点 | CARLA 出生点 184，`(-515.249939, 240.956757, 0.281942)`，yaw `89.866272°` |
| Lincoln 目的地 | CARLA 出生点 177，`(285.980682, -118.373642, 0.581942)` |
| 普通导航参考长度 | 约 1458.5 米 |
| 参考道路 | CARLA 道路 45 → 46 → 47 → 48 → 7 → 8 → 9 → 18 → 19 |
| RSU 无线节点 | `rsu0`，CARLA 世界坐标 `(193, -179, 6)` |
| 固定种子 | `20261104` |
| 固定灯态起点 | 每个控制器的程序第 0 相位、完整 42 秒；不继承启动阶段消耗的相位时间 |
| 联网动态 NPC | 原有前车及剧情车 4 辆，加高速交错背景车 40 辆，共 44 辆，Audi A2 |
| 静止堵车障碍 | 普通路段 280 辆、连接段 57 辆，共 337 辆灰色 Audi A2，不联网、不参与避让 |
| 仿真步长 | 默认 0.05 秒 |
| 信标、紧急更新周期 | 默认各 1 个仿真秒 |
| 协同消息有效期 | 默认 10 个仿真秒，有效更新续期 |
| 离开通信范围后的避让保持 | 默认 30 个仿真秒，从失去有效通道时计时 |
| Lincoln 路权 | 所有实验组均采用最高路权，不按道路限速、信号灯或标线许可限制行驶 |
| Lincoln 固定速度上限 | 无（`maximum_speed_kmh: null`）；保留弯道、障碍物、冲突和终点停车约束 |

RSU 的 6 米是网络节点高度，现有 CARLA 显示塔的高度和图形范围不代表无线天线高度或可靠覆盖范围。

`defaults.json` 中 `lincoln_highest_right_of_way: true` 作用于全部 VANET 实验组，包括第 1 组。Lincoln 统一使用最高路权轨迹控制，不再由普通组 Traffic Manager 的道路限速逻辑限制车速。规划器忽略换道标线许可，允许跨标线及借用对向车道逆向行驶；逆向路点在轨迹细分、控制和 RViz 显示中保持真实行驶方向。相邻车道仍必须是实际可行驶路面，并通过高度、距离和实体隔离检查，不穿越隔离墙或跳到立交桥其他层。变道前保留动态冲突预测。

取消道路交通规则限制不等于取消物理控制：弯道稳定、车辆与行人碰撞防护、借道预测冲突和目的地停车仍可降低目标速度。无固定上限表示不再人为封顶为 120 km/h，不代表车辆会全程维持某个速度。`vanet_status` 显示 `highest_right_of_way: true`、`maximum_speed_kmh: null`，运行控制器标识为 `highest_priority`；Lincoln 面板也显示“最高路权自动驾驶”。

Lincoln 不因 NPC 进入 50 米通信范围、收到 `feedback` 或 RSU `clearance`、等待 NPC 让开而施加无线限速。反馈继续接收、计算并记录，`CLEARANCE_ADVICE diagnostic_only=True` 表示仅供诊断；即使其显示 `active: true`，也不会把建议速度传给驾驶控制器。实际障碍物检查、借道冲突制动和终点停车保留。因此解除的是消息触发的减速，不是关闭物理碰撞防护。

Lincoln 的车辆防撞判断改为使用沿实际轨迹的相对闭合速度，不再把正在前进的 NPC 当静态障碍来计算停车限速。远处同向前车不会仅因进入旧的长距离避障窗口而压低目标速度；静止/对向车辆会按实际闭合风险更早触发保护。SUMO 代理的零物理速度由相邻 CARLA 帧位置差补偿，避免误判为停车。共享的障碍物快照由借道规划器逐帧提供，不新增传感器。

`lincoln_collision_guard` 配置制动减速度估计 4 m/s²、响应时间 0.35 秒、间距余量 3 米；触发后退出边界再加 3 米滞回。需要制动时立即收油并执行刹车下限；风险解除连续确认 0.25 个仿真秒，刹车每秒最多释放 2.0、油门每秒最多增加 1.5，避免反复刹车/加速。行人、实体设施、弯道和终点保护保留。每帧日志的 `collision_guard` 和 `longitudinal_control` 记录物理间距、闭合速度、制动边界、实际油门/刹车与恢复状态。离线闭环已检查“80 米前车、两车初速约 55.8/50.4 km/h，进入 50 米前继续加速”及“30 米前车延迟 3 秒让开时制动后恢复”；这些是简化动力学回归，不代替完整 CARLA 联仿验收。

实验组仍决定紧急身份和通信协同是否启用：第 1 组不自动启用紧急身份或通知 NPC 避让，第 2、3 组保留原有身份规则，第 3 组仍按原通信范围协同。NPC 的正常交通规则、RSU、网络和 CoBridge 时钟不因该本车策略改变。第 1 组现在不再代表“遵守交通规则”的基线，旧结果应与此配置分开比较。配置需重启联仿后生效。

当前 V2V 有效距离为 **50 米**，RSU 有效半径为 **170 米**，均按水平距离、双向生效，边界包含在内。RSU 圆距最近高速道路外缘约 3.08 米；超出范围不交付消息。NPC 已实际收到且尚未过期的避让任务，在离开有效通信通道后继续执行最多 **30 个仿真秒**；到期仍无新消息时开始平稳恢复。该时限从离开时计算，覆盖原有 10 秒租期，不随轮询或无消息的进出圈延长。范围内仍可能因真实无线干扰、丢包而接收失败。距离参数统一在 `experiments/experiment_group_configuration/defaults.json` 的 `communication_ranges`，保持时长在同文件的 `out_of_range_yield_seconds`。

协同组采用直连优先：50 米内 NPC 信标和反馈直接发给 Lincoln，RSU 不为这辆车重复通知或汇总；超过 50 米，且双方均在 RSU 圆内时才中转。Lincoln 支持直接使用 NPC 反馈，也能同时使用其他远处车辆的 RSU 汇总。通道切换等待新消息时，NPC 可在上述 30 秒内保持原避让动作；旧消息不作为新通道的接收或反馈依据。收到新的有效任务消息后使用新消息；收到有效任务取消时立即结束保持。状态变化立即反馈，周期反馈按固定种子错开发送相位。`vanet_status` 的 NPC 状态中，`out_of_range_yielding`、`out_of_range_yield_until` 显示保持状态及截止仿真时刻。

Qtenv 的 Lincoln 小车为深蓝色 `(30,90,230)`、NPC 为灰色 `(165,170,175)`；通信线仍为亮绿色 Lincoln ↔ RSU、蓝色 Lincoln ↔ NPC、暗绿色 RSU ↔ NPC，不标消息文字。SUMO 原有状态配色保持不变。

`run_vanet` 默认启动 **Qtenv 图形界面**，地图方向与 SUMO 一致；绿色圆为 RSU 范围，蓝色圆为 Lincoln 的 50 米范围，NPC 不画通信圈，彩色箭头表示实际完整接收的无线消息。SUMO 同时显示绿色 RSU 覆盖圆与跟随 Lincoln 的蓝色 50 米圈。车辆圈在每个交通步结束后根据实际位置和朝向更新，圆心在车身中心；复位、车辆移除与退出时清理。网络自动等待 CoBridge 时间指令，不需要手动点 Qtenv Run，也不会因此开始紧急实验。Qtenv Stop 会阻止网络推进，联仿期间请保持网络运行。

原有两辆初始同向前车已删除。3 辆定时出发车辆的配置保留在 `experiments/scenario_and_parameter_configuration/town04_emergency.json`；新增车辆逐辆放在 `npc/scenario_vehicle_configuration/`，由场景的 `npc_configuration_files` 清单加载。通用参数在 `experiments/experiment_group_configuration/defaults.json`；无线参数在 `omnet/simulation_configuration/radio_parameters.json`。每轮重置恢复固定 NPC 和保存的信号灯状态，不使用原有随机续路机制。

第一辆新增车为 `vanet_npc_001`，配置文件 `npc/scenario_vehicle_configuration/npc_001_lincoln_front_80m.json`。它在初始车道 `-41_1`，车身中心沿车道位于 Lincoln 初始车身中心前方 80 米；SUMO 出生位置为车头里程 `373.47237146932656` 米，已补偿半车长及车道声明长度与几何长度之差。由于道路弯曲，两车中心直线距离约 79.44 米。准备时该车保持静止；正式开始后正常行驶并参与现有通信与避让，其余 3 辆仍在第 35、38、52 个仿真秒出发。重启 `run_vanet` 并恢复场景后生效，不会向当前运行中的实验强行插车。

场景同时加载 `npc/stationary_traffic_obstacles/town04_left_lane_queues.json`，由同目录的 `highway_lane_queue.py` 展开为静止堵车车队。SUMO 的 0 号车道是路肩，四条同向行车道为 1～4 号；车队仅占最左侧的 3、4 号，覆盖起点前方的 `-41` 后半段、`-46`、`-47`、`-48`，在右转进入城镇的 `-7` 之前结束。普通路段两条车道各 140 辆，共 280 辆；三个中间连接处再放 57 辆，合计 337 辆。内部车道按实际连接映射选择 `:134_10_2/3`、`:1061_2_3/4`、`:764_3_2/3`，没有把普通道路的 3、4 号直接套用到内部车道。其他两条通行车道、匝道和对向道路不添加堵车车。车身为灰色 `(165,170,175)`，按车长加 2.8 米间隙排列；起点对齐按两条车道各自的几何长度转换为 SUMO 车头里程。

堵车车在 Reset 时随原有车辆创建，准备阶段初始共 378 辆（337 辆堵车障碍、原有 80 米前车和 40 辆高速联网背景车）。Start 仅释放动态车，堵车车始终限速为零、禁止自动变道，并设置原地道路停车，避免长时间等待后被 SUMO 自动移走。内部连接段通过一次 `moveTo` 放置，再设置内部车道原地停车，不逐帧搬动车辆。CARLA 保留灰色车辆外观和碰撞实体，关闭物理模拟，不接入 Traffic Manager。完成第一次贴地同步后不再重复发送静止姿态；贴地失败会重试。复位和退出清理所属实体与停车任务。

SUMO 位姿仅在创建时读取一次，随后退订动态字段，保留本地几何缓存和生命周期登记；不会因退订被当作新车反复生成。Lincoln 的 `carla_obstacle_cache.py` 缓存静止包围盒，车辆生命周期或世界改变时重建；`spatial_index.py` 用空间网格筛选轨迹附近的碰撞候选。碰撞保护的制动距离、车身余量和高度检查保留，动态 NPC 与行人仍逐帧预测；借道候选轨迹也单独查询静态网格。`OBSTACLE_CACHE_REBUILT` 日志记录静态和动态数量。不为静止车逐帧计算运动速度或做全车队与全轨迹点的交叉检查。

这批车不进入 VANET 的车辆采样、无线节点、消息、状态配色和动态 NPC 行程统计，因此 Qtenv 不为其创建无线小车或通信圈；SUMO 的原生碰撞报告仍包含这些实体。状态中 `stationary_obstacle_count` 为 337，`network_npc_count` 为 44 辆；CoBridge 日志记录每辆车的 ID、道路、车道、里程、静止及联网标记。将场景的 `stationary_highway_queues` 改为 `false` 可关闭这批堵车车。配置在下一次启动联仿并重置场景后生效。图形渲染、碰撞实体和 SUMO 停车对象管理仍有成本，不能据此承诺整机零开销。

新增高速背景车配置集中在 `npc/highway_background_traffic/town04_staggered_traffic.json`：左数第 3 车道对应 SUMO 2 号，第 4 车道对应 SUMO 1 号，各放 20 辆，共 40 辆。队尾从原来距 Lincoln 约 393 米的底部直线路段前移到左侧弯道红框处：第 4 车道首辆车身中心沿 Lincoln 初始车道位于前方 180 米，比原有 80 米前车再向前 100 米。参考车身中心为 SUMO `(65.296053,27.361684)`。两列沿各自实际车道几何每约 34.103 米一辆，第 3 车道领先第 4 车道约 17.051 米；保持整队最前与最后车身中心间距为 665 米。间距包含车身长度，单车道车身间净空约 30.38 米。全部初始位置仍位于高速与连接段，6 辆在连接段通过一次 `moveTo` 放置，其余正常出生；车头跨道路分界时按下一连接段校准，避免被 SUMO 截断到路尾。准备时静止并固定车道，Start 后恢复普通交通控制，不重复移动初始位置。实际追上车队的时刻仍取决于 Lincoln 与 NPC 的加速、跟车和路况，不通过移动车辆或改写 Lincoln 速度制造追车效果。

这 40 辆的车辆上限是 80 km/h，`speedMode=31`、`laneChangeMode=1621`，按 SUMO 正常跟车、信号灯、路口和变道安全规则行驶。`lcKeepRight=0`、`lcSpeedGain=0` 减少两列之间非必要的抢道/靠右变道，保留按路线提前选道及安全协作。将本侧 `-41/-46/-47/-48` 以及直行驶离段 `-49/-50` 的行车道和对应高速连接车道临时设为 100 km/h；不修改路肩、对向或城镇道路。`lane_speed_limits.py` 在创建场景时保存原值，批次清理/复位时恢复，下一轮重新应用，不改写生成的路网文件。

进城分流口为 `-48`：第 3 车道的 20 辆从创建时就规划为 `-48 → -49 → -50` 直行驶离。第 4 车道的 20 辆由 `junction_route_split.py` 在进入路口前的 `-47/-48` 时按实际前后到达顺序决定路线：按“3 辆直行、1 辆右转”循环 5 次，第 4、8、12、16、20 辆右转进城，其余 15 辆提前并入第 3 车道驶离。加上第 3 车道原有 20 辆，最终 35 辆直行、5 辆进城。`vehicles_per_lane` 定义每列数量，`departure_pattern` 定义分流周期。直行车辆由 SUMO 普通路线选择提前并入第 3 车道，右转车辆保持第 4 车道沿 `-7 → -8 → -9 → -18 → -19` 行驶；不从第 4 车道直接穿过没有直行连接的路口。同帧到达按实际道路进度排序，分流只设置一次路线，不强制变道或跳变位置。日志 `HIGHWAY_SPLIT` 记录顺序、车辆和路线；场景状态 `junction_departures` 可查看决定列表，复位后从第 1 辆直行重新计数。

联网与避让单独配置：`network_enabled: true`、`yielding_enabled: false`。新车正常加入网络并发送信标；实际收到 emergency/notice 后反馈 `received`，原因为 `yielding_disabled_for_scenario`，不发送“已让开”的虚假反馈，也不执行任何紧急变道、停车、降速或恢复驾驶命令。普通交通控制器仍可因跟车或路线需求变道、减速。该标记从场景定义传入协调器和 NPC 应用，SUMO 参数 `vanet.scenario.yielding_enabled` 与 NPC 状态也显示它。原有 4 辆的避让策略保持启用。今后可将该批配置的 `vehicle_defaults.yielding_enabled` 改为 `true`，重启后启用避让。

进城右转入口允许 NPC 利用弯道空间跟进红灯队列：`right_turn_queueing.py` 仅对实际路线为 `-48 → -7` 的动态场景 NPC，在 `-48` 和内部弯道 `:890_2/:890_13` 暂时放宽路口保空要求（独立车型的 `junctionModel.jmIgnoreKeepClearTime=0`），车头驶入 `-7` 后恢复原值。适用于原有 NPC 和新增车流，不限于 5 辆分流进城车；直行车、静态障碍物和 Lincoln 不应用。仍由 SUMO 判断跟车空间和冲突让行，前方红灯照常停车；不强制速度、不搬动车辆、不修改灯态。配置位于 `town04_emergency.json` 的 `right_turn_queueing`，日志 `RIGHT_TURN_QUEUE` 记录启用与恢复，仅状态变化时发送设置命令。

独立真实 SUMO 检查：`VANET_RUN_LIVE_SUMO=1 _vanet_python -m unittest vanet.tests.live_cosimulation_checks.test_highway_background_traffic vanet.tests.live_cosimulation_checks.test_right_turn_queueing -v`。检查 40 辆完整出生、180 米队尾位置、665 米整体长度、车道和几何间距、网络采样、正常行驶、80 km/h 车辆上限（道路仍为 100 km/h）、安全换道空间、实际经过的分流道路（35 直行/5 右转）、零碰撞和限速清理恢复。右转专项复现入口等待，验证后车驶入弯道跟队、红灯不越线、绿灯继续行驶及设置恢复。离线检查验证到达顺序排序、周期分流路线、复位计数以及紧急消息不触发避让；完整 CARLA/无线联仿和 Lincoln 实际追车过程仍需下一次启动验收。

静态队列回归可用 `VANET_RUN_LIVE_SUMO=1 _vanet_python -m unittest vanet.tests.live_cosimulation_checks.test_stationary_highway_queues -v` 独立运行：检查连接车道映射、337 辆原地停车、动态订阅清空、310 个仿真秒零位移/碰撞/传送，以及重置恢复。离线空间索引回归同时对照原始完整算法，覆盖弯道、多层道路、制动滞回和动态车辆，避免通过漏检换取提速。这些检查不代替完整 CARLA 联仿性能验收。

当前配置已根据首次现场记录修正前级红灯阻塞：交叉口车改为从 `-17_2`、位置 35 米、正式开始后 52 秒出车，路线为 `-17 → -18 → -19`；两辆对向车改从 `10_3` 出车，保留位置 8/24 米和 35/38 秒出车时间，路线为 `10 → 9 → 8 → 7`。旧配置从 `-16` 和 `11` 出车，会在前一级路口长期等红灯，未能到达 RSU 附近的冲突位置。

出生位置检查使用真实 SUMO 车道形状和车长测量中心间距，避免把 SUMO 声明里程直接当作几何距离。完整 CARLA/SUMO 联仿仍需运行验证。**实际收到信标或 clearance，不能单独证明 notice → NPC 避让 → feedback → RSU 汇总闭环已经发生。**

## 2. 首次启动与一键入口

在项目根目录执行：

```bash
source setup.bash
run_vanet
```

关闭本工程的联仿进程和对应终端、车辆窗口、RViz、Qtenv，保留 CARLA、当前执行命令的终端和无关终端：

```bash
source setup.bash
stop_vanet
```

可先用 `stop_vanet --dry-run` 查看将关闭的进程，不发送退出信号。关闭顺序为启动器、Lincoln / NPC / RSU / 观察视角、CoBridge、OMNeT++、SUMO；先等待程序正常清理，超时才逐步强制退出。详细记录保存在 `vanet/results/shutdown_*/`。手动启动的项目终端若还有其他任务则保留；不依赖过期的 PID 文件，也不会批量关闭终端服务器。

`run_vanet` 调用 `vanet/start_vanet_experiment.sh`，依赖步骤依次检查并启动：

1. 检查 CARLA；已有可连接实例则复用，没有实例则启动。
2. 检查 Town04；必要时切换地图，并检查或生成相应 SUMO 路网。
3. 构建带实验操作入口的 SUMO GUI，以及 VANET 无线模块。
4. 启动 SUMO，再启动启用 VANET 的 CoBridge。
5. 启动 OMNeT++ / Veins 并等待网络接口就绪。
6. 启动 RSU。
7. 在固定起点加入一辆无传感器 Lincoln。
8. 初始化固定 NPC 和场景，等待 `ready`。

默认各进程保留独立终端，终端内容同时写入启动日志。没有图形终端时可使用诊断方式：

```bash
run_vanet --no-terminals
```

`--no-terminals` 仅改变进程输出方式，并不会把 SUMO GUI 或 CARLA 改成无界面仿真。启动脚本目前会拒绝直接接管已运行的 SUMO，也会拒绝重复添加已登记的 Lincoln；先退出对应旧进程，再使用一键入口。可复用已经打开且可连接的 CARLA。

VANET 时钟按完整一帧计算现实时间预算：控制确认、网络等待、交通推进和显示更新均计入，超过 50 毫秒时不再额外限速；仿真步长仍为 0.05 秒。Lincoln 新帧控制完成后立即唤醒状态交换，空闲时保留心跳。Qtenv 收到网络推进指令时优先执行，等待外部时钟期间整图刷新约 20 次/现实秒，网络计算不依赖绘图刷新。

CoBridge 每完成 20 帧输出一次 `frame_timing`，包含 `mean_ms` 中的整帧现实耗时 `frame_wall`、协调处理 `coordinator`、交通推进 `traffic_step`、步后处理 `after_step`、屏障轮询等待 `barrier_poll_wait`、限速等待 `pacing_wait`，以及整帧耗时的 `frame_p95_ms`。这些耗时使用现实时间，不计入仿真实验通行时间；完整日志仍保留。修改后需重启联仿进程生效。

一键启动只把实验准备到可操作状态，**不会替操作者开启紧急状态或正式开始计时**。准备、开始和取消仍由操作者在 SUMO 或下面的等价命令中明确执行。

## 3. 三组实验的操作步骤

| 组名 | CLI 参数 | 行为 |
| --- | --- | --- |
| 1 Normal | `normal` | 非紧急身份，Lincoln 最高路权驾驶，无协同避让 |
| 2 Emergency | `emergency` | 紧急身份，Lincoln 最高路权驾驶，无通信驱动的 NPC 避让 |
| 3 V2V + RSU | `cooperative` | 紧急身份和最高路权驾驶，加真实 V2V / V2I / RSU 协同 |

每组使用同一固定场景和种子。更改场景配置后，应重新运行需要比较的各组；不能把不同出车配置混作同一套对照结果。

正常 GUI 流程：选择组别 → **Reset scene** → 等待就绪 → **Prepare / Enable** → 等待车辆、路线和网络准备完毕 → **Start experiment**。准备阶段不发送任务协同消息、不提前避让。正式开始后才计算本轮时间。

SUMO GUI 按钮不可点击或需要明确查看返回信息时，可由操作者使用 `vanet_control`。下面以第 3 组为例，每一步分别执行并检查状态，不能用固定等待时间代替就绪确认：

```bash
vanet_control reset --group cooperative
vanet_control status
# 等待 phase 为 ready

vanet_control prepare --group cooperative
vanet_control status
# 等待 phase 为 prepared

vanet_control start
vanet_control status
```

第 1 组把两处 `cooperative` 改为 `normal`，第 2 组改为 `emergency`。这些命令是人工操作入口；`prepare` 在紧急组会明确启用紧急身份，`start` 才开始运行实验。使用命令代替按钮不意味着后台自动启用紧急状态。

需要取消紧急状态时，由操作者执行：

```bash
vanet_control cancel
vanet_control status
```

到达目的地、结束当前路线或人工接管都不会自动取消紧急身份。开始下一轮前，若仍有紧急身份，必须先取消并等待恢复完成，再执行下一轮 `reset`。普通组没有紧急身份时无需执行 `cancel`。取消紧急状态不是完整的进程关闭命令。

`vanet_control status` 或简写 `vanet_status` 可查看阶段、实验组、Lincoln 状态、结果目录、网络连接、RSU 计数及 NPC 状态。出现 `paused` 时检查 `detail` 和日志；不要把暂停轮次当成完成的有效实验。

## 4. 可单独使用的命令

所有函数均通过项目根目录 `setup.bash` 加载。在新的终端先执行 `source setup.bash`。

| 命令 | 用途 |
| --- | --- |
| `run_vanet` | 按依赖顺序启动完整实验会话 |
| `build_vanet_network` | 构建新增无线模块 |
| `run_omnet` | 启动真实 OMNeT++ / Veins 网络进程 |
| `add_vanet_npc` | 向已启用 VANET 的 CoBridge 请求固定场景初始化；不是随机加车 |
| `vanet_status` | 查询当前实验状态 |
| `vanet_control status` | 查询当前实验状态 |
| `vanet_control reset --group normal` | 重置固定场景并选择组别 |
| `vanet_control prepare --group normal` | 准备该组路线和控制状态 |
| `vanet_control start` | 在准备完成后正式开始 |
| `vanet_control cancel` | 人工取消当前 Lincoln 紧急身份 |

现有命令仍包括 `run_carla`、`choose_town04`、`map_carla2sumo`、`run_sumo`、`run_cobridge`、`run_rsu`、`add_lincoln`、`add_npc`。完整实验推荐使用 `run_vanet`，避免漏掉 `run_cobridge --vanet`、无传感器 Lincoln 或固定场景配置。单独运行 `add_npc` 会引入原有随机交通，不属于此处固定对照场景。

网络单项入口支持：

```bash
run_omnet --build-only
run_omnet --seed 20261104
# 可额外指定 --socket 路径、--results 目录，用于独立诊断。
```

不要在现有会话使用的 socket 上重复启动网络进程。当前没有统一的 `stop_vanet` 命令；结束会话时需正常结束本会话的各模块进程，以便刷新文件，避免直接杀掉进程丢失结尾统计。

## 5. 新增目录与集成位置

| 目录或文件 | 当前作用 |
| --- | --- |
| `project_paths.py` | 只集中存放新增功能的路径变量；不存放业务参数 |
| `start_vanet_experiment.sh` | 一键启动的 Shell 入口 |
| `experiments/experiment_session_management/` | 启动依赖、命令入口、会话阶段、每轮协调及终端管理 |
| `experiments/scenario_and_parameter_configuration/` | 固定 Town04 场景、NPC 创建/重置、信号灯恢复 |
| `experiments/experiment_group_configuration/` | 三组共用的默认实验参数 |
| `experiments/metrics_and_event_recording/` | 结构化事件、通信/交通/避让指标与 CSV 输出 |
| `cobridge/` | 移动信息快照、网络节点同步、无线进程连接与时步消息交换 |
| `lincoln/emergency_state_management/` | 紧急身份、导航任务、准备/开始/取消和控制权交接 |
| `lincoln/emergency_driving_control/` | 路径跟踪、速度约束及实际车辆控制 |
| `lincoln/emergency_route_planning/` | 对向借道轨迹评估、几何冲突与返回原车道 |
| `lincoln/v2x_data_exchange/` | Lincoln 与 CoBridge 的状态/指令交换和驾驶日志 |
| `npc/` | 实收消息校验、冲突判断、SUMO 避让动作、本车反馈与恢复 |
| `rsu/` | 实收信标缓存、相关车辆选择、定向通知、反馈汇总及终端状态 |
| `omnet/` | Veins 网络定义、外部移动节点、802.11p 应用、分片重组、构建与独立检查 |
| `sumo/gui_build_and_integration/` | SUMO 实验操作入口的构建接入 |
| `sumo/vehicle_state_display/` | Lincoln/NPC 颜色与参数显示，状态变化时更新，结束后恢复 |
| `protocol/` | 控制接口与消息格式校验 |
| `tests/offline_function_checks/` | 不依赖真实无线或 CARLA 的功能检查 |
| `tests/live_cosimulation_checks/` | 显式开启的真实 SUMO 固定场景/API 检查 |
| `runtime_state/` | socket、锁、当前会话索引等运行状态，不是实验结论 |
| `results/` | 启动会话、每轮实验、网络检查的结果和日志 |

原有模块在现有目录保留，只通过必要入口接入 `vanet`。新增路径从 `vanet.project_paths` 导入；外部安装目录可通过 `AUTODRIVE_PYTHON`、`OMNETPP_ROOT`、`VEINS_ROOT`、`SUMO_HOME` 环境变量指定。原有模块的路径方式没有整体重写。

## 6. 状态和日志怎么查看

SUMO 中的 NPC 颜色表示：收到消息为蓝色，正在避让为橙色，已让出通道为绿色，无法避让为紫红色，恢复中为紫色；正常时恢复原色。Lincoln 紧急状态显示红色，取消后恢复原色。颜色只是业务状态显示，不能单靠颜色判断无线收发是否成功。

RSU 终端定期读取协调器状态，在变化时打印实际接收数、notice、feedback、clearance、回补计数和 NPC 状态。SUMO、CoBridge 和原有 RSU 输出保留；CoBridge 事件和 OMNeT++ 无线记录用于核对消息是否真正到达接收端。

启动时即创建 `results/launch_<时间>_<标识>/`，包含启动事件和各模块终端日志。每轮重置创建独立 `results/run_<时间>_<标识>/`；后续任务也可能使用 `continuation_` 前缀。当前具体目录以 `vanet_status` 的 `results` 字段为准，当前启动会话索引保存在 `runtime_state/session_state/active_session.json`。

主要结果文件：

- `config.json`：本轮参数快照。
- `events.jsonl`：每行一个结构化事件，包含仿真时间、状态变化、收发消息和指标等。
- `metrics.csv`：本轮汇总指标；未到达或暂停不能按成功到达解读。
- `omnet/boundaries.jsonl`：CoBridge 收到的网络步结果。
- `omnet/` 中的无线事件、时步和清单：真实网络侧证据。
- 启动会话的 `logs/`：SUMO、CoBridge、Lincoln、RSU、OMNeT++ 等进程终端输出。

OMNeT++ 原生 scalar/vector 输出属于网络进程生命周期；多轮共享同一进程时，单轮清单引用这些共享文件。正常退出才会完整刷新其尾部，不能把仍运行进程的输出当作最终统计文件。

当前交通统计区分 NPC 出现、真正到达、暂时消失与 teleport；只有 SUMO 到达事件算完成。未完成车辆不伪装成到达，平均速度按实际可观测时间计算。避让响应以实际收包触发的状态变化为依据。几何重叠指标与 SUMO 碰撞事件分别记录，不代表完整物理碰撞传感器结果。

## SUMO GUI 等待 TraCI 时的重绘

项目 GUI 的 `traci_render_sumo_gui.py` 在阻塞接收 TraCI 数据期间临时释放网络绘图锁，收到数据后先恢复锁，再解析和执行命令；异常退出也恢复锁。Lincoln 的 CARLA 位置投影暂时落在 SUMO 车道外、或启用车辆路线等附加图形时，视图不再因等待下一个外部命令而阻塞绘制。CoBridge 仍是唯一时钟，步长仍为 0.05 秒，没有增加虚构的车辆位置或额外交通步。

构建只修改源码副本并链接到 `.runtime/sumo/gui/sumo-gui`，不改本机 SUMO 安装。源码、功能模块及共享头文件都参与构建缓存；正常重复运行不重新编译。新 GUI 在下一次启动 SUMO 时生效，已经打开的窗口继续使用原来的程序。

下面的回归创建独立 GUI，不连接当前实验或 CARLA。它用同一个项目二进制切换旧锁行为作对照，强制构造车道外投影，检查等待时重绘、仿真时间不变以及恢复推进。需要桌面显示环境，先完成项目 GUI 构建：

```bash
source setup.bash
_vanet_python -m foundation.sumo.init.gui.build_sumo_gui
VANET_RUN_LIVE_SUMO_GUI=1 _vanet_python -m unittest \
  vanet.tests.live_cosimulation_checks.test_sumo_gui_render_wait -v
```

`AUTODRIVE_SUMO_GUI_PAINT_TRACE=1` 可按完成重绘输出单调时间戳到 SUMO 终端，默认关闭，避免逐帧日志开销。`AUTODRIVE_SUMO_GUI_DISABLE_READ_UNLOCK=1` 仅供上述回归恢复旧锁行为，正常实验不要设置。这项修复解决等待收包造成的重绘阻塞；交通状态仍按外部时步更新，不代表已实现 60 FPS 位置插值或消除所有整机性能瓶颈。

## 7. 验证层次与当前限制

开发环境中先加载 ROS Jazzy 和系统 Python 包路径，再使用项目 Python 入口运行离线检查（部分原有模块依赖 ROS 与系统 `yaml`）：

```bash
source /opt/ros/jazzy/setup.bash
source setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
_vanet_python -m unittest discover -s vanet/tests/offline_function_checks -v
```

真实 SUMO 检查需要显式开启，会创建自己的临时 headless SUMO 进程、空闲端口与临时输出，检查固定 NPC、重置、信号灯和真实 TraCI 避让 API；不连接 CARLA：

```bash
VANET_RUN_LIVE_SUMO=1 _vanet_python -m unittest discover \
  -s vanet/tests/live_cosimulation_checks -p test_fixed_sumo_scenario.py -v
```

真实无线独立检查入口：

```bash
_vanet_python -m vanet.omnet.build_and_launch.wireless_smoke
_vanet_python -m vanet.omnet.build_and_launch.application_smoke
```

各层含义必须分开：

| 检查层次 | 使用真实组件 | 不能据此证明 |
| --- | --- | --- |
| 离线功能检查 | Python 业务逻辑 | 无线投递、真实车辆驾驶、全链路实验通过 |
| headless SUMO 检查 | SUMO、TraCI、固定场景及避让 API | CARLA 控制和无线联仿通过 |
| `wireless_smoke` | OMNeT++ / Veins 无线、单播/广播、分片与重置 | NPC 在真实联仿中完成避让 |
| `application_smoke` | 真实无线及 NPC/RSU 业务 | SUMO 动作在真实车辆上完成；此检查的 SUMO 动作接口为 mock |
| 三组完整现场实验 | CARLA、SUMO、CoBridge、Lincoln、NPC、RSU、OMNeT++ 全部参与 | 需要逐轮证据，不能以某项 smoke 代替 |

本次已通过 89 项新增离线检查和 86 项原有检查；这不代替下面的现场验收。

本次第 3 组真实现场运行记录为 `results/run_20261104_022049_569955_6f7994/live_verification.json`：Lincoln 在 76.2 个仿真秒后到达，几何重叠事件和 SUMO 碰撞事件均为 0，仍保留紧急身份；到达前有 489 次应用发送和 289 次完整应用接收，其中 Lincoln 实收 9 条 RSU clearance。记录验证了真实 CARLA / SUMO / CoBridge / OMNeT++ 时间协调、V2V 驱动的 NPC 避让，以及 Lincoln → RSU 紧急消息、RSU → Lincoln 汇总消息的真实接收。

该轮没有验证 RSU 定向 notice → NPC → feedback 完整闭环，不能把 clearance 接收当成这一闭环已通过；对向借道的完整现场执行和三组对照也尚未验收。运行中的进程早于随后补充的颜色显示、信标发送相位、每轮网络归档、固定场景修正和通信反馈限速，因此这些更新仍需下次启动验证。通信反馈限速仅使用实际收到且仍有效的前方冲突信息，不能替代物理障碍检查。实际结果以对应轮次日志为准；没有为验证擅自取消当前紧急身份。

当前无线模型使用 Veins 路径损耗与 802.11p 接收模型，没有导入 CARLA 建筑网格生成无线遮挡；配置中的最大干扰距离不是可靠接收半径。更详细的网络实现和输出说明见 `omnet/README.md`。
