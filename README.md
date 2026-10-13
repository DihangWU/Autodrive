* 以下命令默认在 Autodrive 根目录下执行
1. 打开 Carla
    ```shell
    ./carla/init/start_carla.sh
    ```
2. 选择地图 Town04
    ```shell
    ./carla/init/choose_town04.sh
    ```
3. 将 CARLA 当前地图转换为 SUMO 路网，并打开 SUMO GUI
    ```shell
    bash cobridge/start_bridge.sh
    ```
