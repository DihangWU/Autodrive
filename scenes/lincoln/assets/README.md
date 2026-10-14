# 林肯 MKZ 2020 网格来源

`lincoln.dae` 取自桌面 CarlaDemo 的 `carla/assets/lincoln_mkz2020/blue.dae`。
它由本机 CARLA 0.9.16 原始车辆骨骼网格及车门、玻璃、灯具部件转换，
使用米和 ROS 坐标（x 前、y 左、z 上），原点位于车身几何中心的地面投影。
仅材质简化、车漆为蓝色；车门闭合、车轮为中性姿态。

原资产归属 CARLA 项目贡献者；CARLA 资产许可为 CC-BY，代码为 MIT。
见 https://github.com/carla-simulator/carla/blob/0.9.16/README.md#licenses 。

传感器安装位置与参数提取自 CarlaDemo 的五个传感器模组。
本项目独立实现驾驶、ROS 发布和 RViz 配置，不依赖 Demo 的地图、ROS 包或构建产物。
