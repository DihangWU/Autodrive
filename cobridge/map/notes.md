* 地图来源是正在运行的 CARLA，不固定为 Town04；切换地图后重新运行即可。
* 生成的 OpenDRIVE、SUMO 路网、配置和转换日志保存在 `cobridge/runtime/<地图名>/`，不提交到 Git。日志在每次启动时覆盖。
* 关闭 SUMO 窗口即可退出，CARLA 继续运行。
    ```text
    cobridge/
    ├── start_bridge.sh       # 启动入口
    ├── map/
    │   └── map_bridge.py     # 读取 CARLA 地图、转换路网并打开 SUMO GUI
    └── runtime/              # 自动生成的路网、配置和日志，不提交到 Git
    ```
* `map_bridge.py` 放在 `cobridge/map/`，由 `cobridge/start_bridge.sh` 调用。
* 生成文件保存在 `cobridge/runtime/<地图名>/`。
* 查看地图显示程序的参数：
    ```shell
    bash cobridge/start_bridge.sh --help
    ```



## 补充说明：官方地图转换

`cobridge/map/map_bridge.py` 调用 `cobridge/carla-source-official/Co-Simulation/Sumo/util/netconvert_carla.py`，由官方脚本加载其 `data/opendrive_netconvert.typ.xml` 车道类型配置。

转换包含车道类型及通行权限、道路与车道的原始编号映射（`origId`）、车道连接关系，以及根据 CARLA OpenDRIVE 地标生成的信号灯信息；使用 `--guess-tls` 处理路口信号灯控制连接。编号映射保存在路网中，不代表 GUI 默认会显示所有编号标签。

SUMO GUI 加载官方 `examples/viewsettings.xml` 显示配置。地图来自当前 CARLA，生成文件和转换日志位于 `cobridge/runtime/<地图名>/`；后台启动日志位于 `cobridge/runtime/map_bridge.log`。

这一阶段仍只转换和显示地图。SUMO 信号灯数据包含转换工具生成的默认配时，暂未与运行中的 CARLA 信号灯状态同步。