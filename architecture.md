# Autodrive 架构

## 独立进程与生命周期

CARLA、SUMO、CoBridge、RSU 分别占用一个终端，各自通过 Ctrl+C 退出。启动顺序可以不同，缺失的依赖只会使对应功能等待或暂时不接入。

| 命令 | 职责 | 没有 CoBridge 时 | Ctrl+C 的影响 |
| --- | --- | --- | --- |
| `run_carla` | 前台容器、CARLA 世界与物理仿真 | 原生异步推进 | 退出本次 CARLA 容器 |
| `run_sumo` | 前台 SUMO 运行层、唯一 TraCI 连接 | 固定步长自主推进 | 退出本次 SUMO 运行层和子进程 |
| `run_cobridge` | 独立时钟、发现并接管可用仿真器 | 自己可空载运行 | 释放接管；CARLA、SUMO、RSU 不退出 |
| `run_rsu` | 手动创建、显示和清理 RSU | 可直接使用 CARLA 与 SUMO | 只清理本次 RSU，不退出其他进程 |

CoBridge 不再创建 RSU、不持有 RSU 生命周期，也不拥有任何仿真器进程。RSU 进程不调用 `world.tick()`、不改变 CARLA 同步设置、不申请 SUMO 时钟控制权。

RSU 同时需要 CARLA 实体与 SUMO 图标，因此两者尚未就绪时等待；运行中依赖断开时尽量清理并等待恢复，恢复后重新创建。CoBridge 退出不构成 RSU 依赖断开，实体和显示继续存在。

## 目录与职责

```text
foundation/
├── console.py                      # 按组件名输出终端信息与日志，无业务模块注册表
├── module_status.py                # 通用模块状态上报和展示，不接管生命周期
├── ipc.py                          # 通用本地 JSON socket 传输
├── carla/
│   ├── init/carla_init.sh          # 前台附着 Docker；退出时停止本次容器
│   ├── init/load_town04.sh         # 加载地图
│   ├── map_convert/               # OpenDRIVE 导出与 SUMO 路网转换
│   └── carla-official/             # 官方工具
├── sumo/
│   ├── init/sumo_init.py           # 前台进程启动、就绪检查、Ctrl+C 清理
│   ├── runtime.py                 # 自主步进、时钟接管与释放策略
│   ├── events.py                  # 通用创建/删除成功事件、待确认缓冲
│   ├── client.py                  # 运行层客户端；关闭客户端不关闭 SUMO
│   └── traci/
│       ├── connection.py          # 加载官方库、建立实际 TraCI 连接
│       └── remote.py              # 通用公开方法代理与分发，无实体业务
└── cobridge/
    ├── cobridge.py                 # 空载时钟、周期发现、动态接入/释放
    ├── control.py                  # 时钟状态的本地控制传输
    ├── network.py                  # 实际地图文件、指纹与坐标偏移
    ├── adapters/
    │   ├── carla.py                # 接管 CARLA 同步步进，退出恢复原设置
    │   └── sumo.py                 # 接管 SUMO 步进，退出仅释放控制权
    ├── rsu/
    │   ├── rsu.py                 # 可复用的 RSU 实体/图标生命周期
    │   ├── carla_rsu.py           # CARLA 塔身实体
    │   ├── rsu_marker.py          # 标记显示
    │   ├── rsu_label.py           # 文字显示
    │   ├── config/town04.json     # RSU 专属参数
    │   └── assets/rsu.png         # SUMO 图标
    ├── lincoln/                   # 仅目录及空实现、空配置
    └── npcvehicules/              # 仅目录及空实现、空配置，沿用已有拼写
modules/
├── rsu/rsu.py                      # 独立 RSU 命令与重连循环
├── lincoln/lincoln.py               # 空文件
├── npcvehicules/npcvehicules.py      # 空文件
└── omnet/                          # 预留
```

实体桥接实现仍按对象分类放置；`cobridge/rsu/` 的目录归属不代表其运行生命周期属于 CoBridge。Lincoln、NPC、OMNeT++ 和 ROS 2 尚未实现，空配置不会加载。

## SUMO 连接与通用接口

SUMO 原生 TraCI 的关闭命令会结束仿真，不能让短生命周期的 CoBridge 持有并关闭它。为保持独立生命周期，`run_sumo` 的运行层持续持有唯一官方 TraCI Connection，并串行处理本地客户端请求。

`traci/` 中仅保留连接和通用公开方法代理。例如 `connection.poi.add(...)`、`connection.vehicle.getPosition(...)` 可直接使用，新增业务通常不改 TraCI 层。JSON 可序列化的返回值直接传输；不支持的复杂返回值会报告调用错误，不使运行层退出。私有属性访问和远程关闭仿真器等生命周期操作被拒绝。

时钟归属位于 `sumo/runtime.py`：

1. 无时钟持有者时，运行层按 SUMO 步长自主推进。
2. CoBridge 接入后申请唯一时钟控制权；此时运行层暂停自主步进，响应该持有者的 `simulationStep()`。
3. CoBridge 正常释放或其控制连接断开后，运行层恢复自主步进。终端在接管时明确输出“已由外部统一时钟接管，自主步进已暂停”，释放时输出“外部时钟已释放，恢复独立推进”；启动提示仅描述接入前状态。
4. RSU 等实体客户端可以调用公开域接口，但不能推进 SUMO、夺取他人的时钟控制权或关闭 SUMO。
5. 只有 `run_sumo` 自己退出时关闭官方 TraCI 连接，并结束自己创建的 SUMO 子进程。

通用 socket 传输在 `foundation/ipc.py`；时钟策略、进程管理、RSU 坐标转换等均不放入 `traci/`。

## SUMO 对象操作通知

创建与删除的终端通知使用同一条消息链路：

```text
SUMO 执行公开域方法成功
    → SUMO 生成待确认事件
    → CoBridge 通过 events.poll 接收并显示
    → CoBridge 通过 events.ack 回传确认
    → SUMO 收到确认，由自己的 Console 输出结果
```

例如创建和删除图标时，SUMO 终端分别显示 `[SUMO] poi rsu0：创建成功。`、`[SUMO] poi rsu0：删除成功。`。事件由 SUMO 运行层在实际 `add` / `remove` 调用成功后生成，失败调用和只读查询不生成成功事件。对象类别和标识来自通用方法及参数，公共处理不判断 RSU、Lincoln、NPC 等业务类型。关键字调用缺少位置对象标识时，通知只显示域与操作结果。

这条链路与模块进程自报状态分别表达不同事实：模块状态描述进程或会话，SUMO 操作事件描述 SUMO 已完成的对象操作。回传确认只用于通知收讫与本地显示；操作执行不等待 CoBridge，不改变实体或时钟归属。

CoBridge 未接入时事件留在 SUMO 的有界待确认队列中，接入后分批返回；默认容量 2048、每批 64。超限丢弃最旧事件并记录丢弃数量，确认时明确报告。只有当前时钟连接可以读取与确认操作事件；普通实体客户端不能确认。重复确认已移除的事件不会重复输出。事件留在内存，CoBridge 断开不清空，SUMO 进程重启后不保留。

`sumo/events.py` 负责事件缓存和确认，`sumo/runtime.py` 在通用方法调用成功后记录事件；CoBridge 的 SUMO 适配器接收事件并回传确认；SUMO 初始化入口提供自己的输出回调。`traci/` 的基础连接与通用调用无需增加业务分支。

## CoBridge 时间与动态接入

CoBridge 启动不加载 CARLA API、不读取地图、不连接 SUMO；先创建自己的时钟与状态入口。发现循环按配置间隔尝试接入，两端都缺失时进程仍运行。

CARLA 接入后设置同步模式与固定步长；SUMO 接入后申请对应步长的控制权。默认步长 0.05 秒，SUMO 与 CoBridge 的步长需相同。单个仿真器可先接入；另一端随后接入时检查地图指纹，地图不一致则拒绝该次接入并继续等待。

每轮尝试发现 → 推进当前已接入的 CARLA → 推进当前已接入的 SUMO → 增加本地帧号。只比较各自每步的时间增量，不强行将已经独立运行过的仿真器绝对时间归零。每次接入建立新的时间基准，不代表空载时段或接入前的历史已同步。默认按现实时间预算等待；`--fast` 取消等待。连接尝试和接口调用的耗时可能降低实际运行速率。

一端退出或世界切换后，CoBridge 释放该端并继续管理其他可用端，周期重试接入。单端失效时不声称整个实验仍是完整联仿，终端和日志会明确显示断开状态。两端步进不是原子事务，不回滚已经完成的帧。

CoBridge 正常退出：恢复同一 CARLA 世界的原设置、释放 SUMO 时钟控制权、关闭自己的状态入口。CARLA、SUMO、RSU 进程继续运行。文件锁防止本项目重复启动时钟。强杀 CoBridge 可以通过 SUMO socket 断开恢复自主步进，但 CARLA 无法保证自动恢复同步设置；SIGKILL 不等同于正常 Ctrl+C。

OMNeT++ 未来可保留事件调度，再与时间边界协调；50 ms 不作为通信事件的精度。当前没有实现通信联仿。

## RSU 进程

`run_rsu` 读取专属 JSON，等待地图一致的 CARLA 和 SUMO，自行调用 `RsuBridge.setup()` 创建实体与图标，以自己的显示周期绘制标记和文字。显示循环不依赖 CoBridge。位置使用 CARLA 坐标，转换为 SUMO：`x_sumo = x_carla + offset_x`、`y_sumo = -y_carla + offset_y`，偏移来自实际路网。

运行中的 Ctrl+C 会清理本次实体和图标、关闭自己的运行层客户端；关闭该客户端不影响 SUMO 或其他客户端。单实例锁防止两个 RSU 命令相互覆盖。异常退出时资源恢复记录保留在 `.runtime/rsu/session.json`，下次启动恢复；旧 `.runtime/cobridge/rsu-session.json` 会在持锁后迁入新位置。强杀 RSU 命令的资源在下次启动时恢复，不保证即时清理。

## 使用方式

每个终端先执行 `source setup.bash`，然后分别运行：

```bash
# 终端一
run_carla

# 终端二：可先于仿真器启动
run_cobridge

# 终端三：使用已有地图配置，可先于 CoBridge 启动
run_sumo

# 终端四：手动创建；依赖尚未就绪时等待
run_rsu
```

首次准备地图时，在 CARLA 启动后另开临时终端执行 `choose_town04`、`map_carla2sumo`，再启动 SUMO。独立 SUMO 也可以通过位置参数指定现成 `.sumocfg`，不需要正在运行的 CARLA。

每个终端 Ctrl+C 只退出对应组件。更新代码后应重新加载函数、退出旧版组件并按新入口重启；旧后台 CARLA 容器无法直接变成新命令的前台实例，可先运行 `sudo docker stop carla` 退出旧容器后再运行 `run_carla`。旧版 CoBridge/SUMO 的直接 TraCI 连接方式与新运行层不兼容，不能混用。

常用参数：

- CoBridge：`--host`、`--port` 指定 CARLA；`--sumo-socket` 指定 SUMO 运行层；`--step-length`、`--timeout`、`--rpc-timeout`、`--steps`、`--fast`。
- SUMO：配置文件位置参数、`--headless`、`--port` 指定底层 TraCI TCP 端口、`--step-length`、`--control-socket` 指定运行层 socket。始终前台，不再提供后台模式。
- RSU：`--host`、`--port`、`--sumo-socket`、`--config`、`--duration`、`--timeout`、`--rpc-timeout`、`--status-socket`（可选 CoBridge 状态通知地址）。`duration` 为命令总运行秒数，默认直到 Ctrl+C。

SUMO 运行层默认 `.runtime/sumo/control.sock`，CoBridge 状态入口默认 `.runtime/cobridge/control.sock`。原始 TraCI TCP 端口仅由 SUMO 运行层使用，不作为 CoBridge 或 RSU 的连接入口。默认地图选择 `.runtime/maps/*/*.sumocfg` 中最近修改的文件。

## 路径与参数约定

自有 Python 文件的路径与可调默认值集中于 import 后、函数前；使用 `Path(__file__).resolve().parents[n]` 定位根目录，再拼接项目路径。Shell 在开头使用 `BASH_SOURCE[0]` 定位根目录并声明参数，解释器可用 `AUTODRIVE_PYTHON` 覆盖。外部 Docker、X11、Linux `/proc` 路径也在顶部维护。协议字段、索引和数学运算常量不是可调运行参数。官方 CARLA 文件不改写。

日志：`.runtime/cobridge/bridge.log`、`.runtime/rsu/rsu.log`、`.runtime/sumo/runtime.log`、`launcher.log`、`sumo.log`、`sumo-error.log`。

## 终端输出边界

进程报告自己负责的生命周期、依赖和错误。CoBridge 输出时钟、仿真器接入状态，以及各模块主动上报的状态；SUMO 输出实际时钟归属与进程状态；RSU 输出自己的实体状态。公共代码不包含具体业务模块的固定启动说明或模块名单。

`foundation/console.py` 提供 `Console(component)`，统一 `[组件名] 消息` 格式，并将信息同时写入当前进程的日志。普通信息到 stdout，警告和错误到 stderr；异常堆栈保留在日志。名称由调用方传入，没有 RSU、Lincoln、NPC 等固定注册表。以后新模块初始化自己的 Console 即可输出自己的信息；如需在 CoBridge 中展示，则使用同一个通用状态上报接口，不需要修改公共工具、CoBridge 或 SUMO 的输出。

各 Python 运行入口的组件名集中在文件顶部的 `COMPONENT_NAME`。业务提示保留在对应模块，公共工具只处理格式、日志和输出流，不判断任何业务类型。

模块通过 `ModuleReporter(name, path)` 可选地上报 `waiting`、`running`、`recovering`、`error`、`stopped` 状态。后台线程处理连接、重试和定期补报，不使业务运行依赖 CoBridge。CoBridge 通过通用 `module.status` 请求显示名称和状态；名称完全来自上报者，没有 RSU 专属分支。重复状态不刷屏，状态连接断开只更新展示，不操作实体或时钟。

例如 RSU 创建后 CoBridge 显示 `[CoBridge] 模块 RSU：运行中（实体已创建）`；正常退出显示“已停止”。CoBridge 晚启动或重启后，仍在运行的模块会补报当前状态。`status` 回复中的 `modules` 列表可供检查，这只代表模块自报状态，不保证实体清理已由观察方确认。RSU 可用 `--status-socket` 指向非默认 CoBridge 控制入口；默认值与 CoBridge 状态 socket 一致。

## 验证

真实 CARLA/SUMO 验证覆盖：CoBridge 空载运行、RSU 先启动等待、SUMO 晚启动接入、CoBridge 退出后三者继续运行、实际 CARLA/SUMO 再次接入与释放、SUMO 独立 Ctrl+C、SUMO 重启后 RSU 恢复、RSU Ctrl+C 仅清理自身，以及仿真器恢复自主推进。

另有回归测试覆盖接管与释放、实体客户端不能推进时钟、通用 RPC 错误隔离、晚接入、清理异常保留等。CARLA 新启动脚本的前台附着和 Ctrl+C 清理通过假 Docker CLI 验证；没有为了测试而重启现有 GPU 容器。Lincoln、NPC、OMNeT++、ROS 2、长时间压力运行与 CARLA 实际重启恢复尚未验证。

已额外通过独立真实 SUMO 实例验证对象创建与删除，经 CoBridge 适配器接收、确认后在 SUMO 终端输出；重复轮询和确认不会重复输出。操作失败不误报、断开后补发、确认权限及缓冲分批由回归测试覆盖。
