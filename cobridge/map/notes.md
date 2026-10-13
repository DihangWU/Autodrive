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