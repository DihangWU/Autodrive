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
| 固定 NPC | 两辆同向车、一辆交叉口车辆、两辆对向车，Audi A2 |
| 仿真步长 | 默认 0.05 秒 |
| 信标、紧急更新周期 | 默认各 1 个仿真秒 |
| 协同消息有效期 | 默认 10 个仿真秒，有效更新续期 |
| 紧急驾驶速度上限 | 120 km/h；控制器根据曲率、障碍物和冲突减速，并非全程恒速 |

RSU 的 6 米是网络节点高度，现有 CARLA 显示塔的高度和图形范围不代表无线天线高度或可靠覆盖范围。

固定 NPC 的路线、初始车道、位置、速度与出车时间在 `experiments/scenario_and_parameter_configuration/town04_emergency.json`；通用参数在 `experiments/experiment_group_configuration/defaults.json`；无线参数在 `omnet/simulation_configuration/radio_parameters.json`。每轮重置恢复固定 NPC 和保存的信号灯状态，不使用原有随机续路机制。

当前配置已根据首次现场记录修正前级红灯阻塞：交叉口车改为从 `-17_2`、位置 35 米、正式开始后 52 秒出车，路线为 `-17 → -18 → -19`；两辆对向车改从 `10_3` 出车，保留位置 8/24 米和 35/38 秒出车时间，路线为 `10 → 9 → 8 → 7`。旧配置从 `-16` 和 `11` 出车，会在前一级路口长期等红灯，未能到达 RSU 附近的冲突位置。

这些调整已通过独立 headless SUMO 的 80 秒检查：5 辆固定车辆均实际出现，3 辆延迟出车的车辆都前进超过 10 米，随后重置成功。调整仅对下一次加载场景生效，没有改变当前运行现场。修订后的参数仍需完整联仿复验，不能仅凭固定出车时间保证冲突和无线投递。**实际收到信标或 clearance，不能单独证明 notice → NPC 避让 → feedback → RSU 汇总闭环已经发生。**

## 2. 首次启动与一键入口

在项目根目录执行：

```bash
source setup.bash
run_vanet
```

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

一键启动只把实验准备到可操作状态，**不会替操作者开启紧急状态或正式开始计时**。准备、开始和取消仍由操作者在 SUMO 或下面的等价命令中明确执行。

## 3. 三组实验的操作步骤

| 组名 | CLI 参数 | 行为 |
| --- | --- | --- |
| 1 Normal | `normal` | 普通驾驶，无协同避让 |
| 2 Emergency | `emergency` | 启用紧急驾驶，无通信驱动的 NPC 避让 |
| 3 V2V + RSU | `cooperative` | 同样的紧急驾驶，加真实 V2V / V2I / RSU 协同 |

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
