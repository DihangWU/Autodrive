# CARLA / SUMO 环境

本工程目前将 CARLA 启动、地图转换和 SUMO 打开分为三个独立入口。地图转换读取 CARLA 当前地图，SUMO 只加载已经生成的配置文件。

## 目录结构

```text
Autodrive/
├── setup.bash                      # 加载当前终端的快捷命令
├── envinit/
│   ├── carla/
│   │   ├── init/
│   │   │   ├── carla_init.sh       # 后台启动 CARLA Docker 容器
│   │   │   ├── choose_town04.sh    # 将默认 CARLA 实例切换到 Town04
│   │   │   └── notes.md           # CARLA 和 Docker 操作笔记
│   │   └── carla-official/         # 官方源码及 SUMO 联仿工具
│   ├── map_convert/
│   │   ├── carla2sumo.sh          # 地图转换 Shell 入口
│   │   └── carla2sumo.py          # 导出 .xodr、转换 .net.xml、生成 .sumocfg
│   └── sumo/
│       ├── sumo_init.sh           # SUMO 打开 Shell 入口
│       └── sumo_init.py           # 加载已有 .sumocfg 并后台打开 GUI
└── .runtime/                       # 共用的运行产物目录，已加入 Git 忽略规则
    └── maps/<地图名>/               # 地图文件和日志
```

## 环境准备

- CARLA 启动需要 Docker、NVIDIA GPU 容器支持和可用的 X11 桌面显示。
- CARLA 镜像为 `carlasim/carla:0.9.16`，使用宿主机网络。
- 地图转换使用 `$HOME/carla-venv/bin/python`，需要 CARLA Python API 和官方脚本依赖的 `lxml`。
- SUMO 的 `netconvert` 和 `sumo-gui` 需要加入 `PATH`，官方转换脚本还需要 `SUMO_HOME/tools/sumolib`。
- SUMO 打开入口也使用上述 Python 路径，但不会导入 CARLA，也不要求 CARLA 正在运行。

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
| `run_carla` | `envinit/carla/init/carla_init.sh` | 可选的容器名称、RPC 端口，按位置填写 |
| `choose_town04` | `envinit/carla/init/choose_town04.sh` | 当前不解析参数，固定连接 `localhost:2000` |
| `map_carla2sumo` | `envinit/map_convert/carla2sumo.sh` | 默认调用可用；自定义参数见下方说明 |
| `run_sumo` | `envinit/sumo/sumo_init.sh` | 可选的 `.sumocfg` 路径 |

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

三个入口不自动串联：启动 CARLA 不会启动 SUMO，地图转换不会打开 SUMO，打开 SUMO 也不会再次导出地图。

## CARLA 启动与关闭

默认启动命名为 `carla`、RPC 端口为 `2000` 的容器：

```bash
run_carla
```

对应的完整脚本命令为：

```bash
bash envinit/carla/init/carla_init.sh
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

其他操作笔记见 [envinit/carla/init/notes.md](envinit/carla/init/notes.md)；其中部分笔记描述前台运行方式，当前启动入口以这里的 `--rm -d` 为准。

## CARLA 地图导出与 SUMO 转换

默认连接 `127.0.0.1:2000`，读取已经加载的地图，不执行地图切换：

```bash
map_carla2sumo
```

对应脚本为：

```bash
bash envinit/map_convert/carla2sumo.sh
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
~/carla-venv/bin/python envinit/map_convert/carla2sumo.py --host 127.0.0.1 --port 3000 --timeout 60
~/carla-venv/bin/python envinit/map_convert/carla2sumo.py --help
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

官方工具位于 `envinit/carla/carla-official/Co-Simulation/Sumo/`，源码保持原样。当前调用带有 `--guess-tls`，信号灯控制连接可能经过推断，需要核对，不能视为已经验证正确；生成的默认灯控程序也不代表 CARLA 当前的实时灯色。

地图名称从 `current_map.name` 最后一段取得，并非写死 Town04。地图与日志提示使用带工程目录名称的相对路径，例如 `Autodrive/.runtime/maps/Town04/Town04.xodr`。

每次转换直接写入同名文件，不保留历史版本。失败后目录内可能同时存在本次 `.xodr` 和之前的转换结果，应在重试成功后再打开。请一次执行一个转换任务；同名地图目前不按 CARLA 实例隔离。

## 打开已有 SUMO 地图

```bash
run_sumo
```

或者：

```bash
bash envinit/sumo/sumo_init.sh
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

相对配置路径按当前终端目录解析；不指定配置时，脚本根据自身位置寻找工程的 `.runtime`，不受当前工作目录影响。当前输出路径使用 `relative_to(PROJECT_DIR.parent)`，建议选择本工程内的配置；工程父目录之外的配置可能在打印路径时报错。

SUMO GUI 使用 `Popen()` 和独立会话后台运行，终端可继续使用，输出追加到配置所在目录的 `sumo.log`。配置引用的官方 `viewsettings.xml` 决定显示样式，移动工程或官方资源后，已有配置中的绝对样式路径可能需要重新生成。

该入口只加载文件，不导出或转换地图，也没有 `--host`、`--port`、`--timeout`、`--export-only` 参数。关闭 SUMO 窗口不影响 CARLA。

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

当前功能范围是地图导出、转换和预览，没有启动 TraCI 联仿、车辆双向同步或时钟同步，也不会修改 CARLA 同步模式。
