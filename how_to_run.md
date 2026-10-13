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
    ./cobridge/start_bridge.sh
    ```
4. 在 Carla 中 添加 Lincoln 轿车
    * 随机位置添加
        ```shell
        bash carla/Lincoln/run_lincoln.sh
        ```
    * 指定位置添加，指定位置使用 `--pose X Y Z YAW`。
        ```shell
        ./carla/Lincoln/run_lincoln.sh --pose 199.390472 -204.544281 0.519592 91.314423
        ```
        ```shell
        ./carla/Lincoln/run_lincoln.sh --pose 235.597504 -172.815598 0.695892 180.326126
        ```
