# Autodrive

本工程正在逐步重构。目前已实现：启动 CARLA、切换地图、读取当前地图，通过 CARLA 官方工具转换为 SUMO 路网，并在 SUMO GUI 中显示。

所有示例命令默认在工程根目录执行。当前地图显示程序使用 `~/carla-venv/bin/python`，启动后在后台运行。

## 1. 快速启动

### 启动 CARLA

```bash
bash carla/init/start_carla.sh
```

脚本启动 `carlasim/carla:0.9.16` 容器，名称为 `carla`。容器在后台运行，打开 1280×720 的 CARLA 窗口，使用 NVIDIA GPU 和宿主机网络。

等待 CARLA 服务启动完成，再选择 Town04：

```bash
bash carla/init/choose_town04.sh
```

该脚本通过 Python API 连接 `localhost:2000`，调用 `load_world('Town04')`，成功后打印当前地图名称。它只切换地图，不启动 CARLA。

### 打开 SUMO 地图

```bash
bash cobridge/start_bridge.sh
```

脚本立即返回终端，打印后台进程 PID 和日志路径。程序随后读取 CARLA 当前地图、转换路网，并打开 SUMO GUI。关闭启动终端不会停止程序。

后台进程启动并不表示地图转换已经成功。查看实际结果：

```bash
tail -n 50 cobridge/runtime/map_bridge.log
```

关闭 SUMO 窗口即可结束本次地图显示程序，CARLA 继续运行。

## 2. 当前功能范围

| 功能 | 当前状态 |
|---|---|
| 启动 CARLA、切换到 Town04 | 已实现 |
| 从当前 CARLA 地图导出 OpenDRIVE | 已实现 |
| 使用官方工具生成 SUMO 路网 | 已实现 |
| 导入车道类型、权限、编号映射和连接关系 | 已实现 |
| 根据 CARLA 地标生成 SUMO 信号灯信息 | 已实现 |
| 加载官方 SUMO GUI 显示配置 | 已实现，白色背景 |
| CARLA 与 SUMO 车辆生成、运动和销毁同步 | 尚未接入当前入口 |
| 实时同步两边的信号灯状态 | 尚未接入 |
| Lincoln、传感器、ROS2、RViz | 尚未重建 |
| NPC 批次管理、RSU、OMNeT++/Veins 通信 | 尚未重建 |

当前 `start_bridge.sh` 启动的是地图导出和显示流程，还不是完整的实时同步桥。它不调用 `world.tick()`，不修改 CARLA 同步模式，不建立 TraCI 连接。

## 3. 地图如何从 CARLA 到 SUMO

```text
正在运行的 CARLA
    → Python API 获取当前地图
    → to_opendrive() 导出 OpenDRIVE
    → 官方 netconvert_carla.py
        → netconvert + 官方车道类型配置
        → 处理 CARLA 交通信号灯地标及控制连接
    → 生成 .net.xml 和 .sumocfg
    → SUMO GUI 加载路网与官方 viewsettings.xml
```

地图来源是正在运行的 CARLA，不固定为 Town04，也不直接使用官方目录里预置的 `examples/net/Town04.net.xml`。

每次启动都会重新导出和转换。程序打开 GUI 后，不会持续监听 CARLA 地图变化。切换地图时，先关闭旧 SUMO 窗口，在 CARLA 中选好地图，再重新运行 `bash cobridge/start_bridge.sh`。

### 路网包含的信息

- 车道类型和通行权限。
- OpenDRIVE 道路、车道的原始编号映射，保存在 `origId` 参数中。
- 道路与车道之间的连接关系。
- 交通信号灯控制器、受控连接和 CARLA 地标映射。

SUMO 自身的道路和车道 ID 不保证与 CARLA ID 完全相同，`origId` 用于记录原始映射。信息保存在路网里，不代表 GUI 默认会显示全部编号标签。

当前调用官方工具时启用了 `--guess-tls`，用于推断路口信号灯的控制连接。SUMO 中生成的是默认配时，尚未读取运行中的 CARLA 实时灯色。

### 界面与坐标

SUMO GUI 加载官方 `examples/viewsettings.xml`，使用白色背景及其中的道路、车道等显示设置。它展示二维路网，不复制 CARLA 的三维建筑、纹理或摄像头画面。

SUMO 坐标经过路网转换处理，可能与 CARLA 世界坐标不同。转换生成的 `.net.xml` 中包含 `<location>`、`netOffset` 等信息；当前界面没有额外显示 CARLA 坐标，也没有加入时间倍率或复制 CARLA 坐标功能。

## 4. 目录与文件职责

```text
Autodrive/
├── README.md
├── how_to_run.md                       # 简要启动命令
├── carla/
│   └── init/
│       ├── start_carla.sh              # 启动 CARLA Docker 容器
│       ├── choose_town04.sh            # 通过 API 加载 Town04
│       └── notes.md                    # CARLA 使用笔记
└── cobridge/
    ├── start_bridge.sh                 # 后台启动地图显示程序
    ├── .gitignore                      # 忽略生成文件及 Python 缓存
    ├── map/
    │   ├── map_bridge.py               # 地图导出、转换及 GUI 启动
    │   └── notes.md                    # 地图模块笔记
    ├── carla-source-official/          # 引入的 CARLA 官方源码
    │   ├── LICENSE
    │   └── Co-Simulation/Sumo/
    │       ├── util/
    │       │   ├── netconvert_carla.py
    │       │   └── data/opendrive_netconvert.typ.xml
    │       ├── examples/               # 示例路网、车辆类型、GUI 配置
    │       ├── data/                   # 车辆类型数据
    │       ├── sumo_integration/       # 官方 CARLA/SUMO 同步适配器
    │       ├── run_synchronization.py  # 官方同步入口，当前未调用
    │       └── spawn_npc_sumo.py       # 官方车流入口，当前未调用
    └── runtime/                       # 自动生成，不提交到 Git
        ├── map_bridge.log
        └── Town04/                    # 实际名称取决于当前地图
            ├── Town04.xodr
            ├── Town04.net.xml
            ├── Town04.sumocfg
            └── netconvert.log
```

`map_bridge.py` 保留在 `cobridge/map/`；运行文件统一写入 `cobridge/runtime/`。路径根据脚本位置计算，不依赖启动时的工作目录。

## 5. 环境与依赖

当前本机使用的环境：

| 项目 | 配置 |
|---|---|
| CARLA 服务 | Docker 镜像 `carlasim/carla:0.9.16` |
| CARLA Python API | `carla 0.9.16` |
| Python 解释器 | `~/carla-venv/bin/python` |
| SUMO | `1.22.0`，默认安装目录 `~/sumo-1.22.0` |
| XML 处理 | Python 标准库及虚拟环境中的 `lxml` |
| 显示环境 | 当前 CARLA 启动脚本使用 X11、`DISPLAY` 和 `xhost` |

地图转换需要 `netconvert`、SUMO 的 `sumolib` 和 CARLA Python API；打开窗口还需要 `sumo-gui`。当前流程没有建立 TraCI 连接。

程序寻找 SUMO 可执行文件的顺序：

1. 若设置了 `SUMO_HOME`，先查找其 `bin/`。
2. 未设置时，先查找 `~/sumo-1.22.0/bin/`。
3. 找不到时，再查找系统 `PATH`。

使用其他 SUMO 安装目录：

```bash
SUMO_HOME=/你的/SUMO/安装目录 bash cobridge/start_bridge.sh
```

程序会根据选中的 `netconvert` 安装位置，为官方转换子进程设置 `SUMO_HOME` 和 `PATH`。

## 6. 参数与直接运行

查看帮助，帮助内容在当前终端打印：

```bash
bash cobridge/start_bridge.sh --help
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--host` | `localhost` | CARLA 服务地址 |
| `--port` | `2000` | CARLA RPC 端口 |
| `--timeout` | `30` | CARLA API 超时时间，单位秒 |
| `--export-only` | 关闭 | 只生成路网和配置，不打开 SUMO GUI |

例如：

```bash
bash cobridge/start_bridge.sh --host localhost --port 2000 --timeout 60
```

只导出路网时，启动脚本仍会在后台执行，结果查看日志：

```bash
bash cobridge/start_bridge.sh --export-only
```

调试时可以直接在前台运行 Python，实时查看输出：

```bash
~/carla-venv/bin/python cobridge/map/map_bridge.py
```

只导出并等待完成：

```bash
~/carla-venv/bin/python cobridge/map/map_bridge.py --export-only
```

地图选择脚本的 CARLA API 超时为 120 秒，与地图显示程序的默认 30 秒分开设置。

## 7. 日志、退出与清理

| 文件 | 内容 |
|---|---|
| `cobridge/runtime/map_bridge.log` | 后台程序输出、CARLA 连接错误、GUI 输出 |
| `cobridge/runtime/<地图名>/netconvert.log` | 官方转换工具与 netconvert 的输出、警告和错误 |
| `<地图名>.xodr` | 从本次 CARLA 地图导出的 OpenDRIVE |
| `<地图名>.net.xml` | 本次转换生成的 SUMO 路网 |
| `<地图名>.sumocfg` | 路网和 GUI 显示配置的引用 |

同一地图再次启动时会覆盖其生成文件和转换日志；每次后台启动都会覆盖 `map_bridge.log`。

查看 Town04 的转换日志：

```bash
tail -n 50 cobridge/runtime/Town04/netconvert.log
```

关闭 SUMO 窗口会结束对应地图显示程序，不关闭 CARLA。后台模式下，启动终端的 Ctrl+C 不用于停止已经返回的程序。当前没有独立的桥停止命令，也没有防止重复启动的单实例检查；关闭旧窗口后再启动，避免重复窗口和共享输出文件。

停止 CARLA 容器：

```bash
sudo docker stop carla
```

当前容器使用 `--rm`，退出后会自动删除容器，Docker 镜像仍保留。后台启动模式下不需要保持原终端打开。

关闭地图显示程序后，可删除全部生成文件和日志，下次启动会重新生成：

```bash
rm -rf cobridge/runtime
```

此命令只清理运行产物，不删除地图程序或官方源码。

## 8. 常见问题

### CARLA 仍显示 Town10

当前启动脚本没有指定地图，启动后单独运行：

```bash
bash carla/init/choose_town04.sh
```

先等 CARLA 服务启动完成。切换成功后，脚本会打印实际地图名称。

### CARLA 连接超时

确认容器仍在运行，并查看日志：

```bash
sudo docker ps
sudo docker logs --tail 50 carla
```

确认 CARLA 已完成启动，且 `--host`、`--port` 与服务一致。地图显示程序使用 `--timeout` 调整 API 超时，但它不会自动等待一个尚未启动的服务。

### 容器名称 carla 已被占用

检查现有容器：

```bash
sudo docker ps -a
```

已运行的 CARLA 可以继续使用，无需重复启动；需要重新启动时，先停止现有容器。

### 启动后没有 SUMO 窗口

先查看 `map_bridge.log`，再查看对应地图的 `netconvert.log`。检查 CARLA 连接、SUMO 安装位置、官方文件是否完整，以及 `DISPLAY` 是否有效。

### 路网转换出现警告

警告不一定表示失败。程序会检查官方转换工具的退出码；转换失败会在主日志中提示转换日志的位置。成功转换仍可能包含几何或地标警告，具体道路问题应结合日志和地图检查。

### SUMO 与 CARLA 的图形或信号灯不同

SUMO 显示的是转换后的二维路网，CARLA 显示三维世界。当前没有车辆或灯色的实时同步；原始编号映射也不代表两边使用完全相同的内部 ID。

## 9. 官方工具来源

- [CARLA 官方仓库](https://github.com/carla-simulator/carla)
- [CARLA 0.9.16 SUMO 联仿目录](https://github.com/carla-simulator/carla/tree/0.9.16/Co-Simulation/Sumo)
- 本地许可：[cobridge/carla-source-official/LICENSE](cobridge/carla-source-official/LICENSE)

当前工程从本地官方目录调用转换脚本，保留官方车道类型配置和显示配置。官方同步适配器及车流脚本已引入，但尚未接入当前运行流程。保留官方许可文件及源码版权声明。
