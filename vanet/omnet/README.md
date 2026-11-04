# 外部时钟驱动的 Veins 802.11p 网络

本模块已实现实际的 Veins `Nic80211p`、`Mac1609_4` 和 `PhyLayer80211p` 网络。没有 INET，也没有 TraCIScenarioManager；交通时钟始终属于 CoBridge。

## 运行入口

从项目根目录执行：

```bash
python3 -m vanet.omnet.build_and_launch.build_network
python3 -m vanet.omnet.build_and_launch.run_network
```

独立启动支持 `--socket PATH`、`--results DIR`、`--seed INTEGER`；`--build-only` 只编译。项目统一入口通过 `setup.bash` 的 `run_omnet` 调用。默认依赖和源码路径都从 `vanet/project_paths.py` 获取。

默认 `--ui Qtenv`，一键入口也明确启用 Qtenv；无图形诊断可指定 `--ui Cmdenv`。编译使用当前安装的 Qt5Widgets。Qtenv 自动进入网络运行，外部时钟调度器在等待 CoBridge 时持续处理 GUI 事件，不使用现实时间推进网络。实验开始与取消仍由 SUMO 控制；联仿期间停止 Qtenv 会阻止网络返回边界，触发桥端等待或超时。

## 距离约束与通信可视化

统一读取 `vanet/experiments/experiment_group_configuration/defaults.json` 的 `communication_ranges`：V2V 为 50 米，包含 RSU 的链路为 170 米，按水平距离计算且包含边界。实际帧经过 Veins 无线模型后，应用在分片重组前检查距离；移动出圈时丢弃该来源的未完成重组，旧传输不能靠重新进圈恢复。距离内仍受 PHY/MAC 干扰和丢包影响。

边界结果增加 `communication_ranges` 和 `communication_links`，后者只是可通信的几何邻居，用于撤销出圈来源；不能当作已收到消息。实际 `deliveries` 增加 `communication_link` 距离、类型与上限。NPC 按实际接收来源分别保存有效授权，出圈立即撤销该来源，重新进入必须实际收到新消息；RSU 同样仅使用范围内实际收到的任务及信标。

生成的 `qtenv_scene.json` 保存 Town04 路网、坐标转换、半径及高速边缘间隔，`communication_ranges.json` 保存参数快照。最近高速道路外缘约 173.08 米，当前半径 170 米；生成场景时拒绝覆盖高速的半径。SUMO 的 Y 轴反转一次得到 Qtenv 画布，车辆、道路、范围与箭头共用米制坐标，不会出现上下镜像。

绿色圆表示 RSU 范围，蓝色圆表示 Lincoln 的 50 米范围；NPC 不显示通信圈。红色紧急消息、金色定向通知、蓝色其他消息箭头仅在实际完整接收后出现，保留 0.5 个仿真秒；Lincoln 发射时通信圈短暂加亮。界面统计是累计发送及完整接收数量；原始事件继续详细写入 JSONL。SUMO 中也显示相同中心、170 米半径的绿色轮廓圆，以及随 Lincoln 车身中心移动的蓝色 50 米圈；在每个交通步结束后更新 SUMO 图形，支持转向和场景复位，无额外仿真时钟。

车辆使用带车窗、轮胎与车灯的俯视小车示意图，并随 CARLA 航向旋转；Lincoln 为红色、NPC 为蓝色。NPC 间不进行业务通信，普通交通判断复用现有 CARLA 全局状态；NPC 信标定向发送给 RSU，紧急消息与反馈仍经过真实无线网络。

运行进程在 Unix socket 接受一个 CoBridge 连接；初始化后停留在网络时间 0，只有收到 `advance` 才推进。每个步长终点安排低优先级边界事件，确保该时刻的无线事件处理完成后再返回结果。桥端非阻塞调用可继续响应 SUMO GUI，不依赖墙上时间推动网络。

## Python 接口

```python
from vanet.cobridge.omnet_connection_management import OmnetConnection

network = OmnetConnection(timeout=10)
network.start()
# 在主事件循环调用 poll()，检查 network.status。
network.poll()
if network.ready:
    network.submit(step_id=1, until=0.05, nodes=nodes, transmissions=messages)
# 后续循环持续 poll()，take_result() 返回完成的边界结果或 None。
```

节点为 `{id, role, x, y, z, speed, heading}`，坐标使用 CARLA 世界坐标、米、角度。车辆的 z 是 actor 参考点，模型增加配置中的车辆天线高度；RSU 的 z 已经是实际配置的天线位置。内部统一平移坐标以满足 Veins 正坐标 playground，不改变相互距离。移动采用上一已提交交通位置的零阶保持，不外推尚未发生的交通运动。

发送请求为 `{sender, recipient, payload}`，recipient 为目标节点 ID 或 `broadcast`。定向通知使用实际 MAC 单播，广播通过实际 PHY/MAC 投递。不存在的目标节点记录 `message.target_absent`，不会制造接收事件。

无线应用将 JSON 载荷拆成最多 1200 字节的内容片段，每片实际 bitLength 包含内容和应用元数据。只有实际接收全部片段才返回完整应用消息；接收不全会记录过期。这样较长路线不会被当成固定长度小包传输。

结果包含 `step_id`、边界 `time`、`deliveries` 和累计统计。每条接收包含 sender、recipient、payload、sent_at、received_at；时间为 OMNeT++ 精确事件时间。Python 客户端校验步号、边界和接收因果，不接受未来结果。

## 复位与结束

空闲时调用 `network.reset(seed)`；继续 poll，等待 reset_pending 变为 False，再消费 `take_result()` 的 reset 确认。复位会注销并销毁全部旧无线节点，清空重组、消息和本轮计数，按 seed 初始化所有全局 RNG。网络时刻继续单调前进，协调器使用返回 time 作为本轮网络时间起点；新轮 step_id 可以重新开始。

每轮独立归档使用 `network.reset(seed, results_directory=round_omnet_directory)`。目录必须是 `vanet/results/` 的子目录，客户端及 C++ 服务端都校验真实路径和符号链接；每轮使用新的目录。reset 确认中的 `artifacts` 返回实际输出路径。新进程 hello 的 `capabilities.per_run_archives` 为 true；若连接旧版本进程，请在正常结束现有实验后重启网络，再启用归档。

正常关闭使用 `request_shutdown()`，等待 stopped 确认后 `close()`，让 OMNeT++ 完成 scalar/vector 输出。单纯关闭连接被视作联仿中断；同一网络进程不接受隐式重连继续实验。网络错误和超时通过 `status['error']` 报告，主协调器必须暂停交通推进。

## 结果与当前传播模型

每次运行保存 `omnetpp.ini`、`radio_parameters.json`、生成的传播模型 XML、`launch.json`、`wireless_events.jsonl` 以及 OMNeT++ scalar/vector 文件。JSONL 包含仿真时间和现实时间，记录节点增删、帧发送与接收、消息重组、MAC 控制通知、丢弃和复位。

指定每轮目录后，原始事件写入该轮 `wireless_events.jsonl`，每步累计统计及交付数量写入 `network_steps.jsonl`。`network_manifest.json` 保存本轮归档起点、最近边界及关闭后的终点；切换到下一轮或正常结束网络时关闭区间。未指定目录的旧式 `reset(seed)` 保留当前输出位置。原生 `network.sca` 和 `network.vec` 仍是网络进程共享文件，在进程正常结束时完成输出；manifest 明确标记 `shared_network_process` 并引用原文件，不把它们伪称为独立每轮结果。

初始参数为 20 mW、6 Mbps、5.89 GHz、路径损耗指数 2.4、噪声底 -98 dBm，均可通过无线参数配置修改并留存快照。当前使用 Veins SimplePathlossModel 和 Decider80211p；尚未把 CARLA 建筑物网格导入无线遮挡模型。最大干扰距离是连接管理器的上界，不表示界内必然收包。实际接收取决于 PHY/MAC、干扰、信噪比及队列。

## 可执行验证

```bash
python3 -m vanet.omnet.build_and_launch.wireless_smoke
python3 -m vanet.omnet.build_and_launch.application_smoke
python3 -m vanet.tests.live_cosimulation_checks.wireless_range_check
python3 -m vanet.tests.live_cosimulation_checks.wireless_range_check --ui Qtenv
```

`wireless_range_check` 使用真实 Veins 验证双向 50/50.1 米、170/170.1 米边界、广播按接收者筛选、分片出圈后重新进入及新传输恢复、水平距离与外部时钟空闲冻结。Qtenv 模式使用离屏 Qt 实际启动、实际收发并保存 `omnet/qtenv_view.png`；添加 `--visible` 可以显示该独立检查窗口。检查不连接现有 CARLA/SUMO，不等同于完整联仿验收。

`wireless_smoke` 使用真实 OMNeT++/Veins 检查近距离广播、远距离无接收、约 9.8 KB 消息分片重组、MAC 定向投递、复位后同种子同延迟、节点删除、每轮事件隔离和归档区间，以及 Python/C++ 两端对目录越界及符号链接越界的拒绝。

`application_smoke` 把真实无线接收送入项目实际 NPC/RSU 应用，验证信标 → RSU 选车 → 定向通知 → NPC 避让反馈 → Lincoln 汇总的闭环；SUMO 动作接口用 mock 验证，未运行或改变 CARLA/SUMO。两类验证都将结果保存到 `vanet/results/`。
