* 此目录的 sh 文件不依赖路径，可以移动到任何位置执行
* 因为 town04 地图的路况全，但是新的 carla 0.10.0 不支持，ubuntu24.04 又无法直接安装 CARLA 0.9.16，所以采用 0.9.16 docker 
* docker 操作
    * 查看查看本地已下载的镜像image
        ```shell
        sudo docker images
        ```
    * 查看正在运行的 docker 容器(container，是镜像的实例化对象)
        ```shell
        sudo docker ps
        ```
        * 查看查看所有容器，包括用`sudo docker stop carla`停止的
            ```shell
            sudo docker ps -a
            ```
            * 容器是一个环境，容器内的程序停止了不表示容器停止了
    * 删除指定容器“carla“(删除镜像实例化的容器，不是本地镜像)
        ```shell
        sudo docker rm carla
        ```
* 打开 carla 界面后可以 wasdqe 控制方向，鼠标长按滑动切换视角，鼠标滚轮切换速度
* 终端 ctrl + c 退出容器
* 注意手动拖动 carla 窗口大小会闪退
* 列出可用自带地图(需要先启动 carla 才生效否则卡死)
    ```shell
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(20); print('\n'.join(c.get_available_maps()))"
    ```

* 选择地图(需要先启动 carla 才生效否则卡死)
    ```shell
    ~/carla-venv/bin/python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(120); c.load_world('Town04')"
    ```