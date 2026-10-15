# CARLA / SUMO 环境

本工程将 CARLA 启动、地图转换、SUMO 打开和 RSU 场景模块分别管理。`simuenv/` 集中存放仿真环境的启动、地图转换、TraCI 服务和自由视角工具；场景模块仍放在 `modules/`，运行产物仍放在根目录 `.runtime/`。地图转换读取 CARLA 当前地图；SUMO 加载已有配置，由独立 TraCI 服务接收模块请求；RSU 在 CARLA 和 SUMO 中使用同一份位置配置。

## 目录结构

```text
Autodrive/
├── setup.bash                      # 加载当前终端的快捷命令
├── simuenv/
│   ├── carla/
│   │   ├── init/
│   │   │   ├── carla_init.sh       # 后台启动 CARLA Docker 容器
│   │   │   ├── choose_town04.sh    # 将默认 CARLA 实例切换到 Town04
│   │   │   └── notes.md           # CARLA 和 Docker 操作笔记
│   │   ├── mouse_move/
│   │   │   ├── mouse_move.sh      # 独立自由相机悬浮面板入口
│   │   │   └── mouse_move.py      # 鼠标按钮、拖拽转头和调速
│   │   └── carla-official/         # 官方源码及 SUMO 联仿工具
│   ├── map_convert/
│   │   ├── carla2sumo.sh          # 地图转换 Shell 入口
│   │   └── carla2sumo.py          # 导出 .xodr、转换 .net.xml、生成 .sumocfg
│   └── sumo/
│       ├── init/
│       │   ├── sumo_init.sh       # SUMO 打开 Shell 入口
│       │   └── sumo_init.py       # 只选择 .sumocfg、调用管理模块启动
│       └── traci/
│           ├── service.py         # 后台持有 TraCI 连接，串行处理请求
│           └── client.py          # 其他模块调用的本地请求接口
├── modules/
│   └── rsu/
│       ├── rsu.sh                 # RSU 前台会话入口
│       ├── rsu.py                 # 配置校验、两端创建和退出清理
│       ├── carla_rsu.py           # 从旧版复用的有碰撞塔身
│       ├── rsu_marker.py          # 显眼的塔架和信标绘制
│       ├── rsu_label.py           # 巨型粉色文字绘制
│       ├── sumo_rsu.py            # 坐标转换和 SUMO POI 请求
│       ├── config/town04.json     # RSU 配置，保留旧版位置
│       └── assets/rsu.png         # SUMO 图标
└── .runtime/                       # 共用的运行产物目录，已加入 Git 忽略规则
    ├── maps/<地图名>/               # 地图文件和 SUMO 日志
    ├── sumo/                      # TraCI 管理端日志、本地 socket 和锁
    └── rsu/                       # RSU 会话状态、日志和锁
```

## 环境准备

- CARLA 启动需要 Docker、NVIDIA GPU 容器支持和可用的 X11 桌面显示。
- CARLA 镜像为 `carlasim/carla:0.9.16`，使用宿主机网络。
- 地图转换使用 `$HOME/carla-venv/bin/python`，需要 CARLA Python API 和官方脚本依赖的 `lxml`。
- SUMO 的 `netconvert` 和 `sumo-gui` 需要加入 `PATH`，官方转换脚本还需要 `SUMO_HOME/tools/sumolib`。
- SUMO 打开入口也使用上述 Python 路径，管理端通过 `SUMO_HOME/tools` 导入 TraCI；SUMO 本身不要求 CARLA 正在运行。RSU 命令则同时需要 CARLA 与本工程的 SUMO 管理端就绪。

按当前本机 SUMO 安装位置配置：

```bash
export SUMO_HOME="$HOME/sumo-1.22.0"
export PATH="$SUMO_HOME/bin:$PATH"
```

## 加载快捷命令

在每个需要使用快捷命令的终端执行：

```bash
source /home/maple/Desktop/Autodrive/setup.bash
```

`source` 将函数加载到当前 Shell；执行 `bash setup.bash` 不会把命令留在当前终端。加载后可以在其他工作目录使用：

| 快捷命令 | 对应脚本 | 参数 |
|---|---|---|
| `run_carla` | `simuenv/carla/init/carla_init.sh` | 可选的容器名称、RPC 端口，按位置填写 |
| `choose_town04` | `simuenv/carla/init/choose_town04.sh` | 当前不解析参数，固定连接 `localhost:2000` |
| `map_carla2sumo` | `simuenv/map_convert/carla2sumo.sh` | 默认调用可用；自定义参数见下方说明 |
| `run_sumo` | `simuenv/sumo/init/sumo_init.sh` | 可选的 `.sumocfg` 路径 |
| `run_rsu` | `modules/rsu/rsu.sh` | 可选的 `--host`、`--port`、`--config`、`--duration` |
| `mouse_move_carla` | `simuenv/carla/mouse_move/mouse_move.sh` | 可选的 `--host`、`--port`、`--speed`、`--turn-speed` |

快捷函数通过 `"$@"` 转发参数。下面涉及相对文件路径的示例，除非另有说明，均在工程根目录运行。

## 常用启动顺序

```bash
source /home/maple/Desktop/Autodrive/setup.bash
run_carla
```

等待 CARLA 完成启动后，如需 Town04 再执行；已经选好地图则跳过：

```bash
choose_town04
```

然后导出当前地图，再打开 SUMO：

```bash
map_carla2sumo
run_sumo
```

需要添加 RSU 时，重新加载包含 `run_rsu` 的 `setup.bash` 后执行：

```bash
run_rsu
```

RSU 命令保持在前台，按 `Ctrl+C` 删除本次 RSU，保留 CARLA 和 SUMO。各入口不自动串联：启动 CARLA 不会启动 SUMO，地图转换不会打开 SUMO，打开 SUMO 也不会再次导出地图。

## CARLA 启动与关闭

默认启动命名为 `carla`、RPC 端口为 `2000` 的容器：

```bash
run_carla
```

对应的完整脚本命令为：

```bash
bash simuenv/carla/init/carla_init.sh
```

可以按顺序传入容器名称和 RPC 端口：

```bash
run_carla carla_test 3000
```

省略时分别使用 `carla` 和 `2000`。这是位置参数，不是 `--name`、`--port` 形式。启动设置包括 GPU、4 GB 共享内存、无声音、1280×720 窗口以及 `--network host`。本机客户端通过 `127.0.0.1:<RPC端口>` 连接；原始地图资源位于 CARLA 容器中。

Docker 使用 `--rm -d`：后台启动，退出后自动删除容器。启动时打印的长字符串是容器 ID，不是 PID。脚本随后显示名称和连接地址，这些输出不代表 CARLA API 已经就绪。

查看容器和日志：

```bash
sudo docker ps
sudo docker logs --tail 100 carla
sudo docker logs -f carla
```

使用 `logs -f` 时，`Ctrl+C` 只退出日志查看。停止后台 CARLA：

```bash
sudo docker stop carla
```

自定义名称时将上面的 `carla` 替换成对应名称。因为启用了 `--rm`，退出后通常无需再执行 `docker rm`。

`choose_town04` 当前写死了 `localhost:2000`，不会跟随 `run_carla` 的端口参数。其他端口的 CARLA 可以通过 Python 显式选择地图，例如：

```bash
~/carla-venv/bin/python -c "import carla; c=carla.Client('127.0.0.1',3000); c.set_timeout(120); c.load_world('Town04')"
```

其他操作笔记见 [simuenv/carla/init/notes.md](simuenv/carla/init/notes.md)；其中部分笔记描述前台运行方式，当前启动入口以这里的 `--rm -d` 为准。

## 独立自由视角悬浮面板

加载或重新加载 `setup.bash` 后，可直接使用快捷命令：

```bash
source /home/maple/Desktop/Autodrive/setup.bash
mouse_move_carla
```

快捷命令同样转发参数，例如 `mouse_move_carla --port 3000 --speed 20`。

远程桌面无法直接控制 CARLA 窗口时，可以单独启动鼠标控制面板：

```bash
bash simuenv/carla/mouse_move/mouse_move.sh
```

指定另一份 CARLA 或初始速度：

```bash
bash simuenv/carla/mouse_move/mouse_move.sh --host 127.0.0.1 --port 3000 --speed 10 --turn-speed 45
```

查看参数：

```bash
bash simuenv/carla/mouse_move/mouse_move.sh --help
```

面板功能：

控制按钮布局如下；移动和转头按住操作，视角预设单击生效：

```text
左转       W 前进     右转
A 左移     S 后退     D 右移
Q 上升     抬头       恢复水平视角
E 下降     低头       45° 看地面
```

| 控件 | 操作 |
|---|---|
| W / S | 按住前进 / 后退；普通模式沿视线移动，45° 锁定时沿水平面移动 |
| A / D | 按住左移 / 右移 |
| Q / E | 按住上升 / 下降，沿世界竖直方向移动 |
| 抬头、低头、左转、右转 | 按住持续转动观察方向 |
| 恢复水平视角 | 保留位置和 yaw，将 pitch、roll 归零 |
| 45° 看地面 | 单击锁定 pitch=-45°、roll=0，再次单击释放；按钮显示锁定状态。锁定期间 WASD 沿水平面移动，Q/E 沿世界竖直方向升降；可左右转向，抬头、低头及拖拽的俯仰操作暂不生效 |
| 蓝色拖拽区 | 按住鼠标左键拖动，改变朝向；不需要 CARLA 窗口接收鼠标 |
| 滚轮、速度按钮、滑条 | 调整移动速度，范围 0.5～100 米/秒，默认 10 |
| 停止 | 清除当前移动与转头操作 |
| 贴靠 CARLA | 重新将面板放到 CARLA 窗口右下角 |

释放 45° 锁定后保持当前视角，恢复普通移动和转头操作；点击“恢复水平视角”也会解除锁定并恢复水平。停止、按钮松开、面板失焦或最小化只清除运动操作，不解除锁定；连接重建或切换地图时解除锁定。面板置顶，可拖动标题栏重新摆放；启动时通过可选的 `xwininfo` 查询唯一的 CARLA 窗口并定位到右下角。查询工具不可用、没有匹配窗口或有多个 CARLA 窗口时，默认放在桌面右下角，由用户拖动到合适位置。它是独立窗口，不嵌入或修改 CARLA 原生界面，也不会自动跟随 CARLA 窗口后续移动。

该功能只使用 Python 标准库 Tk 和 CARLA Python API，不调用 SUMO、RSU、ROS、TraCI 或其他场景模块，并通过 `setup.bash` 提供 `mouse_move_carla` 快捷函数。面板直接连接 `--host/--port`，通过 `world.get_spectator().set_transform(...)` 控制观察相机，不创建传感器、不操作车辆、不调用 `world.tick()`、不更改同步模式。CARLA 请求在独立子进程执行，避免 RPC 超时占用 Python 解释器而冻结 Tk 界面；失败时面板保持响应并自动重试；切换地图后重新取得 spectator。

关闭面板只停止面板自身，不停止 CARLA，观察相机保留最后位置。启动脚本使用 `nohup` 后台运行，立即释放终端，并输出面板 PID；关闭终端后面板仍可运行。标准输出和错误追加到 `.runtime/carla/mouse_move/panel.log`；`--help` 仍直接在终端显示。CARLA 的 spectator 是共享观察相机，其他客户端若同时持续控制它，仍可能互相覆盖，需要关闭另一端的跟随相机功能。

## CARLA 地图导出与 SUMO 转换

默认连接 `127.0.0.1:2000`，读取已经加载的地图，不执行地图切换：

```bash
map_carla2sumo
```

对应脚本为：

```bash
bash simuenv/map_convert/carla2sumo.sh
```

Python 入口定义了三个参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--host` | `127.0.0.1` | CARLA 服务地址 |
| `--port` | `2000` | CARLA RPC 端口 |
| `--timeout` | `20` | CARLA 请求超时，单位为秒 |

**当前 Shell 入口的 `--host 127.0.0.1` 行与后面的 `"$@"` 之间缺少参数分隔空格。** 无额外参数时可以运行；追加参数时，首个参数可能被拼接到地址末尾。该限制也影响通过 `map_carla2sumo` 转发的参数。本次仅更新文档，尚未修改脚本。

需要自定义参数或查看帮助时，暂时直接运行 Python：

```bash
~/carla-venv/bin/python simuenv/map_convert/carla2sumo.py --host 127.0.0.1 --port 3000 --timeout 60
~/carla-venv/bin/python simuenv/map_convert/carla2sumo.py --help
```

转换流程：

```text
CARLA 当前地图
    ↓ get_world().get_map() 获取地图对象
    ↓ save_to_disk(...) 导出道路描述
<地图名>.xodr
    ↓ mapconvert_carla2sumo() 调用官方 util/netconvert_carla.py
    ↓ 官方脚本调用 SUMO netconvert，使用官方车道类型配置
    ↓ 补充信号灯控制连接与默认程序
<地图名>.net.xml
    ↓ generate_sumocfg() 写入路网文件名和官方显示配置路径
<地图名>.sumocfg
```

官方工具位于 `simuenv/carla/carla-official/Co-Simulation/Sumo/`，源码保持原样。当前调用带有 `--guess-tls`，信号灯控制连接可能经过推断，需要核对，不能视为已经验证正确；生成的默认灯控程序也不代表 CARLA 当前的实时灯色。

地图名称从 `current_map.name` 最后一段取得，并非写死 Town04。地图与日志提示使用带工程目录名称的相对路径，例如 `Autodrive/.runtime/maps/Town04/Town04.xodr`。

每次转换直接写入同名文件，不保留历史版本。失败后目录内可能同时存在本次 `.xodr` 和之前的转换结果，应在重试成功后再打开。请一次执行一个转换任务；同名地图目前不按 CARLA 实例隔离。

## 打开已有 SUMO 地图

```bash
run_sumo
```

或者：

```bash
bash simuenv/sumo/init/sumo_init.sh
```

无参数时，从工程根目录的 `.runtime/maps/*/*.sumocfg` 中，按文件最后修改时间选取最新的一份。它不查询 CARLA 当前地图；手动保存配置也会改变选择结果。

明确指定配置：

```bash
run_sumo .runtime/maps/Town04/Town04.sumocfg
```

查看参数：

```bash
run_sumo --help
```

相对配置路径按当前终端目录解析；不指定配置时，脚本根据自身位置寻找工程的 `.runtime`，不受当前工作目录影响。

`init/sumo_init.py` 保持简短：保留配置选择与校验，由 `open_sumo()` 调用独立模块的 `start_sumo()`。所有后台启动、就绪等待、复用、TraCI 连接和 POI 管理都在 `traci/service.py`，没有把管理参数塞进初始化入口。

`traci/service.py` 后台运行，通过 `traci.start()` 启动 SUMO 并持有一条连接；同一配置重复运行 `run_sumo` 会复用管理端，不重复开窗。管理端使用本机 UNIX socket 接收 JSON 请求，RSU 通过 `simuenv.sumo.traci.client` 访问，不能把这个 socket 当成 CARLA 的 RPC 端口。

`init/` 只放初始化入口，`traci/` 放本工程的管理服务和请求接口，文件统一命名为 `service.py`、`client.py`。自己的模块使用完整包路径导入，例如 `from simuenv.sumo.traci.client import request`；官方 SUMO 库仍使用 `import traci`，从 `SUMO_HOME/tools` 加载，两者不是同一模块。运行文件继续保存在根目录 `.runtime/sumo/`。

终端在服务就绪后释放。管理日志写入 `.runtime/sumo/service.log`，SUMO 日志写入配置同目录的 `sumo.log`。配置引用的官方 `viewsettings.xml` 决定显示样式，移动工程或官方资源后，已有配置中的绝对样式路径可能需要重新生成。

查看管理状态和关闭受管理的 SUMO，使用服务模块自己的命令：

```bash
~/carla-venv/bin/python simuenv/sumo/traci/service.py --status
~/carla-venv/bin/python simuenv/sumo/traci/service.py --stop
```

关闭 SUMO 窗口同样会使管理端退出。切换 SUMO 地图时先停止管理端，再用 `run_sumo <新配置路径>` 启动。当前仅管理一个 SUMO 实例。此前用普通 `Popen()` 打开的窗口不受该服务管理，需要关闭旧窗口后重新运行 `run_sumo`。

当前管理端不调用 `simulationStep()`，因此不会推进 SUMO 仿真时间，也不会接管 CARLA 时钟；运行中可以添加、查询和删除 POI。后续车辆联仿需要单独接入统一时钟协调器。

该入口只加载文件，不导出或转换地图，也没有 `--host`、`--port`、`--timeout`、`--export-only` 参数。关闭 SUMO 窗口不影响 CARLA。

## RSU 实体与 SUMO 图标

运行前保证 CARLA 已加载所需地图、该地图已转换，并且 `run_sumo` 已启动对应配置。使用快捷命令或完整脚本：

```bash
run_rsu
# 或
bash modules/rsu/rsu.sh
```

默认读取 `modules/rsu/config/town04.json`，位置保留旧版 `f511f80` 的 `rsu0`：`x=193`、`y=-179`、塔脚 `base_z=0.39`、塔顶 `z=60`，信标半径 `beacon_radius=5`，单位为米。`z` 是塔顶的 CARLA 绝对高度，实际塔身高度为 `z - base_z`。

```json
{
  "map": "Town04",
  "coordinate_system": "carla",
  "rsus": [
    {
      "id": "rsu0",
      "x": 193.0,
      "y": -179.0,
      "z": 60.0,
      "base_z": 0.39,
      "beacon_radius": 5.0
    }
  ]
}
```

新增 RSU：在 `rsus` 数组中增加一项，使用唯一 ID（字母、数字或下划线），填写 `x/y/z/base_z`，然后退出并重新运行 RSU 命令。位置建议选道路外的平整区域，塔身高度支持大于 0 且不超过 200 米。使用其他地图时，另备配置并修改 `map` 和对应位置；不会把 Town04 的坐标自动搬到其他地图。

自定义 CARLA 地址、端口或配置：

```bash
run_rsu --host 127.0.0.1 --port 3000 --config /完整路径/my_rsus.json
```

运行五秒后自动清理，便于验证：

```bash
run_rsu --duration 5
```

实现分工：

- `carla_rsu.py` 复用旧版圆柱堆叠实体，使用 CARLA 内置 `/Engine/BasicShapes/Cylinder.Cylinder`，关闭重力与物理运动、启用碰撞。塔身可被相机和雷达感知，不需要独立 `.fbx` 文件。
- `rsu_marker.py`、`rsu_label.py` 使用醒目标记和约 8 米高的粉色大字。每个塔顶只显示一块文字面板，`rsu0` 显示为带空格的大写 `RSU 0`（`rsu1` 显示 `RSU 1`）。文字绕塔中心的竖直轴水平旋转，每 4 秒现实时间一圈，让各方向轮流看到正面；约 20 Hz 更新并使用短寿命绘制减少残影，塔身本身不转。这些是短时 debug 绘制，退出后在 CARLA 继续推进时自然消失；碰撞由实体塔身承担。
- `sumo_rsu.py` 请求管理端添加同名 POI，图标引用本模块 `assets/rsu.png`。POI 是显示标记，不阻挡 SUMO 车辆。
- `rsu.py` 管理整个会话，任一端创建失败会清理已创建对象；只允许一个 RSU 命令会话运行，但一份配置可以包含多个 RSU。

坐标直接使用 CARLA 世界坐标，SUMO 转换从当前实际路网的 `<location netOffset>` 读取偏移：

```text
SUMO x = CARLA x + offset_x
SUMO y = -CARLA y + offset_y
```

Town04 当前路网的偏移是 `(503.02, 423.76)`，因此旧版 RSU 对应 SUMO `(696.02, 602.76)`。偏移不写死在 RSU 配置中。启动前还会核对 CARLA、SUMO、RSU 配置的地图名称，以及导出 `.xodr` 与 CARLA 当前道路数据是否一致。

退出行为：

- `Ctrl+C` 或正常终止 RSU 命令：删除本次 CARLA 实体和 SUMO POI，不关闭两端仿真程序。
- 关闭 SUMO 或管理端：RSU 检测到断开后清理 CARLA 实体并结束。
- 强制杀死 RSU 进程：管理端自动回收该进程的 POI；CARLA 实体在下次启动 RSU 时根据会话前缀回收，不能把强制退出当成即时清理。
- 清理无法确认时保留 `.runtime/rsu/session.json`，日志记录原因，下次启动时重试。

该模块尚未接入 OMNeT++，不计算无线通信、握手或通信连线，也不会修改 CARLA 地图资源。

## 运行文件与排查

```text
.runtime/maps/Town04/
├── Town04.xodr                 # CARLA 导出的 OpenDRIVE 道路描述
├── Town04.net.xml              # 官方工具转换出的 SUMO 路网
├── Town04.sumocfg              # SUMO 启动配置，引用路网和显示配置
├── conversion.log              # 最近一次转换的普通输出和错误输出
└── sumo.log                    # SUMO GUI 输出，启动时追加
```

`.sumocfg` 是 XML 格式的配置文件，仅保存路网引用等设置，不包含 `.net.xml` 的道路数据。`.xodr` 和 `.net.xml` 描述道路，不包含完整建筑、植被和材质。

- CARLA 无法连接：检查容器状态、RPC 地址与端口，等待服务就绪，再按需增大超时。
- 启动后找不到容器：使用了 `--rm`，若程序很快退出，容器会被自动删除；排查时可临时去掉该选项以保留退出日志。
- 转换失败：查看 `conversion.log`。`joined opendrive roads` 表示道路对应关系出现合并，重复 landmark 警告表示多个信号标志匹配到同一连接；转换成功仍需核对灯控关系。
- SUMO 窗口未出现或加载失败：查看 `sumo.log`，检查图形会话、路网和显示配置路径。

也可以直接使用 SUMO 命令加载已有配置：

```bash
sumo-gui -c .runtime/maps/Town04/Town04.sumocfg
```

其他运行文件：

```text
.runtime/sumo/
├── service.log                 # 后台 TraCI 服务日志
├── control.sock                # 模块请求接口，服务退出时移除
└── service.lock                # 防止重复服务持有同一个 SUMO
.runtime/rsu/
├── rsu.log                     # RSU 创建与清理日志
├── session.json                # 活动会话及实体 ID，正常退出后删除
└── session.lock                # 防止重复运行 RSU 命令
```

本轮实测覆盖：旧版单塔位置、两个 RSU 的双端创建和删除、重复会话拒绝、毫米波雷达与激光雷达命中、Ctrl+C 清理、强制退出后的图标回收和重启实体恢复、中途创建失败的完整回滚，以及 SUMO 退出后的 CARLA 清理。测试中没有改变 CARLA 同步模式或固定步长。

当前已有 TraCI 运行时控制接口和 RSU 双端同步，但尚未实现车辆双向联仿或时钟同步。直接执行 `sumo-gui -c ...` 仍只能预览，不会建立本工程 RSU 所需的管理服务。
