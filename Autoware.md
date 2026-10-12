```shell
ros2 launch autoware_launch planning_simulator.launch.xml   map_path:=$HOME/autoware_data/maps/sample-map-planning   vehicle_model:=sample_vehicle   sensor_model:=sample_sensor_kit
```



下面按照我们这次 **Autoware 1.9.0 原生安装的全过程**，整理你在 MapleStation-Pro 上新下载、安装、编译的软件、驱动相关组件、依赖库和开发工具。

表格采用四列：

**名称｜安装路径｜磁盘占用（GB）｜功能概述**

磁盘占用为本次扫描安装路径得到的实际分配空间（`du -s -B1`），按 **1 GB = 1,000,000,000 字节**换算并保留两位小数。目录包含其子目录，文件统计自身占用；符号链接路径按目标统计。重复/嵌套路径的数值有重叠，**不能直接相加，也不代表对应软件包独占大小**。小于 0.005 GB 的文件可能显示为 `0.00`，不表示不存在。路径含“等”时只统计明确列出的主要路径。文字描述的 CUDA 库目录按 `targets/x86_64-linux/lib/`、bin 按 `bin/`、文档按 `doc/`、Nsight 按 `/opt/nvidia/` 下对应工具目录解析；“acados 源码及编译目录”统计 `/opt/acados/`。无法定位的文字路径、当前终端会话以及不存在/读取不完整的路径按实情标注。


原表中几项模糊的 NVIDIA 路径已用 `dpkg -L` 确认：`cuda-nsight-13-0` 统计 `nsightee_plugins/` 与其管理脚本；`gds-tools-13-0` 统计 `/usr/local/cuda-13.0/gds/tools/`；三个 `cuda-toolkit-*-config-common` 统计其 `/usr/share/doc/<包名>/`，后两项另含 `/etc/ld.so.conf.d/987_cuda-13.conf`、`000_cuda.conf`。其余表格保留原安装路径文字。

有一个重要区别：**NVIDIA 驱动 595.91.07、Ubuntu 24.04 和 ROS 2 Jazzy 是本次安装 Autoware 之前就已经存在的**。我会把它们单独列为“原有环境”，避免误认为我们这次重新安装了驱动。

此外，APT 软件包经常同时把文件放在 `/usr/bin`、`/usr/lib`、`/usr/include` 等目录。因此第二列列出的是**主要安装路径**，而不一定是该软件包全部文件的位置。

## 一、Autoware 本体及源码

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| -------------------------- | ---------------------------------- | ---: | ----------------------------------------------------- |
| Autoware 1.9.0 主仓库 | `/home/maple/autoware/` | 7.27 | 自动驾驶软件栈的总工作区，管理源码和编译。 |
| Autoware Core | `~/autoware/src/core/` | 0.30 | 自动驾驶核心功能，包括规划、控制、定位及基础算法。 |
| Autoware Universe | `~/autoware/src/universe/` | 1.00 | 社区扩展模块，包括感知、定位、规划、控制及 GPU 算法。 |
| Autoware Launcher | `~/autoware/src/launcher/` | 0.05 | 管理 Autoware 节点的启动和系统集成。 |
| Autoware Sensor Component | `~/autoware/src/sensor_component/` | 0.13 | 传感器相关组件和接口。 |
| Autoware Ansible 配置 | `~/autoware/ansible/` | 0.00 | 官方开发环境安装和依赖版本管理配置。 |
| Autoware Core 编译文件 | `~/autoware/build/` | 5.12 | 保存 CMake 缓存、中间目标文件等编译产物。 |
| Autoware Core 安装文件 | `~/autoware/install/` | 0.25 | 保存编译完成的 ROS 2 软件包、库和环境配置。 |
| Autoware 编译日志 | `~/autoware/log/` | 0.42 | 保存 Colcon 编译日志和错误信息。 |
| Autoware Universe 编译记录 | `/tmp/autoware-universe-build.log` | 0.00 | 本次 Universe 编译过程中保存的终端输出。 |

目前源码工作区共识别出 **488 个 ROS 2 软件包**，其中 Core 的 **133 个包已经成功编译**。Universe 的 282 个包仍以最终构建结果为准。

## 二、GPU 驱动、CUDA 和 TensorRT

这是我们这次最重要的一组 GPU 开发环境。

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ------------------------------- | ---------------------------------------------------- | ---: | ------------------------------------------------------------ |
| NVIDIA Driver 595.91.07（原有） | `/usr/lib/x86_64-linux-gnu/` 等 | 10.39 | 控制 RTX 4070 Ti SUPER，提供 GPU 驱动能力；本次没有重新安装。 |
| NVIDIA CUDA Toolkit 13.0 | `/usr/local/cuda-13.0/` | 5.18 | NVIDIA GPU 程序的完整 CUDA 开发工具链。 |
| CUDA 编译器 nvcc 13.0.88 | `/usr/local/cuda-13.0/bin/nvcc` | 0.03 | 将 CUDA C++ 源码编译成 GPU 可执行代码。 |
| CUDA Runtime | `/usr/local/cuda-13.0/targets/x86_64-linux/lib/` | 4.54 | CUDA 程序运行时依赖。 |
| CUDA 开发头文件 | `/usr/local/cuda-13.0/targets/x86_64-linux/include/` | 0.04 | CUDA 程序编译需要的 API 头文件。 |
| cuBLAS | CUDA 13.0 库目录 | 4.54 | GPU 高性能矩阵计算。 |
| cuFFT | CUDA 13.0 库目录 | 4.54 | GPU 快速傅里叶变换。 |
| cuSPARSE | CUDA 13.0 库目录 | 4.54 | GPU 稀疏矩阵计算。 |
| cuSOLVER | CUDA 13.0 库目录 | 4.54 | GPU 线性代数求解。 |
| cuRAND | CUDA 13.0 库目录 | 4.54 | GPU 随机数生成。 |
| cuPTI | CUDA 13.0 目录 | 5.18 | CUDA 性能分析和性能数据采集。 |
| NVRTC | CUDA 13.0 库目录 | 4.54 | CUDA 运行时编译。 |
| NVIDIA Nsight Compute | NVIDIA 工具安装目录（APT 管理） | 1.31 | CUDA Kernel 性能分析。 |
| NVIDIA Nsight Systems | NVIDIA 工具安装目录（APT 管理） | 1.09 | CPU/GPU 程序执行与性能分析。 |
| TensorRT 10.13.3.9 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | NVIDIA 深度学习推理加速库。 |
| TensorRT 开发头文件 | `/usr/include/x86_64-linux-gnu/` | 0.04 | 编译 TensorRT C++ 节点和插件。 |
| TensorRT Plugin | `/usr/lib/x86_64-linux-gnu/` | 10.39 | 提供 TensorRT 扩展推理算子。 |
| TensorRT ONNX Parser | `/usr/lib/x86_64-linux-gnu/` | 10.39 | 解析 ONNX 神经网络模型。 |

其中，TensorRT 的 8 个 APT 软件包已经明确验证为 **`10.13.3.9-1+cuda13.0`**。

**注意：目前没有确认安装 cuDNN，因此不能将 cuDNN 列为已安装组件。**

## 三、cumm 与 spconv：GPU 稀疏卷积

这是我们为完整编译 BEVFusion 和 TensorRT Plugins，专门额外安装的组件。

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ------------------- | ------------------------------------------------------ | ---: | ------------------------------------------------- |
| spconv_cpp 源码仓库 | `/home/maple/spconv_cpp/` | 2.88 | Autoware Foundation 维护的 C++ 稀疏卷积源码项目。 |
| cumm 0.5.3 源码 | `~/spconv_cpp/cumm/` | 0.01 | CUDA 矩阵运算和底层计算支持。 |
| cumm 编译目录 | `~/spconv_cpp/cumm/build-amd64/` | 0.00 | cumm 的编译中间文件。 |
| cumm 0.5.3 安装包 | `~/spconv_cpp/cumm/_packages/cumm_0.5.3_amd64.deb` | 0.00 | 自行编译生成的 Debian 安装包。 |
| cumm 0.5.3 | `/usr/local/` | 15.41 | 为 spconv 提供底层 CUDA 计算组件。 |
| cumm CMake 配置 | `/usr/local/share/cmake/cumm/cummConfig.cmake` | 0.00 | 让 CMake 通过 `find_package(cumm)` 找到 cumm。 |
| spconv 2.3.8 源码 | `~/spconv_cpp/spconv/` | 2.86 | GPU 稀疏卷积算法源码。 |
| spconv 编译目录 | `~/spconv_cpp/spconv/build-amd64/` | 2.51 | 保存大量 CUDA 编译中间文件。 |
| spconv 2.3.8 安装包 | `~/spconv_cpp/spconv/_packages/spconv_2.3.8_amd64.deb` | 0.15 | 自行编译生成的 Debian 安装包，约 150 MB。 |
| spconv 2.3.8 | `/usr/local/` | 15.41 | GPU 稀疏卷积库，支持点云深度学习模型。 |
| libspconv.so | `/usr/local/lib/` 下的 spconv 库文件 | 0.58 | spconv 的 C++ 动态链接库。 |
| spconv CMake 配置 | `/usr/local/lib/cmake/spconv/` | 0.00 | 让 CMake 找到 spconv 和 `spconv::spconv` 目标。 |

我们已经实际验证：

```
cumm found: /usr/local/share/cmake/cumm
spconv found: /usr/local/lib/cmake/spconv
spconv::spconv target available
```

因此，**cumm 和 spconv 已安装，并且可以被 CMake 发现**。不过，BEVFusion 的完整运行功能仍需要后续实际测试。

## 四、acados：自动驾驶优化与控制

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ---------------------- | ------------------------------------------------- | ---: | ------------------------------------------------------------ |
| acados v0.5.3 源码 | `/opt/acados/` | 1.01 | 实时最优控制和非线性优化框架。 |
| acados 编译目录 | `/opt/acados/build/` | 0.02 | CMake 编译中间文件。 |
| libacados.so | `/opt/acados/lib/libacados.so` | 0.00 | acados 核心 C/C++ 动态库。 |
| BLASFEO | `/opt/acados/lib/` | 0.01 | 为优化问题提供高性能线性代数运算。 |
| HPIPM | `/opt/acados/lib/libhpipm.so` | 0.00 | 求解模型预测控制中的二次规划问题。 |
| qpOASES | acados 源码及编译目录 | 1.01 | 二次规划求解器，编译时启用了相关支持。 |
| Tera Renderer v0.2.0 | `/opt/acados/bin/t_renderer` | 0.01 | 根据模板生成 acados 求解器代码。 |
| acados Python 虚拟环境 | `/opt/acados/.venv/` | 0.71 | 隔离 acados 的 Python 依赖，避免影响 ROS 2。 |
| CasADi | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | 符号计算、自动微分和优化建模。 |
| SymPy | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | Python 符号数学计算。 |
| SciPy | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | 科学计算和数值算法。 |
| NumPy | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | 数组、矩阵和基础数值计算。 |
| Matplotlib | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | 绘制数值计算结果。 |
| Cython | `/opt/acados/.venv/lib/python3.12/site-packages/` | 0.71 | Python 与 C/C++ 扩展接口。 |
| acados_template 0.5.1 | `/opt/acados/interfaces/acados_template/` | 0.00 | acados 的 Python 求解器代码生成接口，采用 editable 模式安装。 |

其中 `acados_template` 的 Python 包版本显示为 **0.5.1**，而 acados 主仓库版本是 **v0.5.3**。这是你实际安装日志中的版本组合。

## 五、GeographicLib：地理坐标与高程模型

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| -------------------------- | ----------------------------------------------- | ---: | ---------------------------------- |
| geographiclib-tools | `/usr/bin/` 等 | 0.82 | GeographicLib 地理计算工具。 |
| GeographicLib 地理模型目录 | `/usr/share/GeographicLib/` | 0.47 | 保存地理模型数据。 |
| EGM2008-1 高程模型 | `/usr/share/GeographicLib/geoids/` | 0.47 | 提供大地水准面数据，用于高程转换。 |
| egm2008-1.pgm | `/usr/share/GeographicLib/geoids/egm2008-1.pgm` | 0.47 | EGM2008-1 的主要模型文件。 |

你已经得到：

```
geoid egm2008-1 installed.
```

说明地理模型安装成功。

## 六、通过 rosdep 新安装的系统依赖和 ROS 2 软件包

这是你执行以下命令时安装的主要依赖：

```
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y
```

此前的 APT 模拟结果为 **82 个新软件包、0 个升级、1 个移除**。其中移除的是原有 `systemd-timesyncd`，安装了 `chrony`。

下面按照功能归类，但保留实际软件包名称。

### 6.1 ROS 2 Jazzy 相关软件包

这些 ROS 2 软件包的主要安装位置通常是 `/opt/ros/jazzy/`。

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| --------------------------------- | ----------------- | ---: | ---------------------------------- |
| ros-jazzy-ament-clang-format | `/opt/ros/jazzy/` | 1.00 | ROS 2 代码格式检查。 |
| ros-jazzy-aruco | `/opt/ros/jazzy/` | 1.00 | ArUco 标记检测。 |
| ros-jazzy-behaviortree-cpp-v3 | `/opt/ros/jazzy/` | 1.00 | 行为树决策框架。 |
| ros-jazzy-can-msgs | `/opt/ros/jazzy/` | 1.00 | CAN 总线 ROS 消息定义。 |
| ros-jazzy-diagnostic-aggregator | `/opt/ros/jazzy/` | 1.00 | 汇总系统诊断状态。 |
| ros-jazzy-eigen-stl-containers | `/opt/ros/jazzy/` | 1.00 | 支持 Eigen 类型的 STL 容器。 |
| ros-jazzy-geodesy | `/opt/ros/jazzy/` | 1.00 | ROS 地理坐标转换。 |
| ros-jazzy-geographic-info | `/opt/ros/jazzy/` | 1.00 | 地理信息消息与工具。 |
| ros-jazzy-geometric-shapes | `/opt/ros/jazzy/` | 1.00 | 几何形状处理。 |
| ros-jazzy-grid-map-core | `/opt/ros/jazzy/` | 1.00 | 网格地图核心库。 |
| ros-jazzy-grid-map-costmap-2d | `/opt/ros/jazzy/` | 1.00 | Grid Map 与二维代价地图集成。 |
| ros-jazzy-grid-map-cv | `/opt/ros/jazzy/` | 1.00 | 网格地图 OpenCV 操作。 |
| ros-jazzy-grid-map-msgs | `/opt/ros/jazzy/` | 1.00 | 网格地图消息定义。 |
| ros-jazzy-grid-map-pcl | `/opt/ros/jazzy/` | 1.00 | 网格地图与点云处理。 |
| ros-jazzy-grid-map-ros | `/opt/ros/jazzy/` | 1.00 | Grid Map 的 ROS 接口。 |
| ros-jazzy-grid-map-rviz-plugin | `/opt/ros/jazzy/` | 1.00 | 在 RViz 中显示网格地图。 |
| ros-jazzy-lanelet2-core | `/opt/ros/jazzy/` | 1.00 | Lanelet2 高精地图核心库。 |
| ros-jazzy-lanelet2-io | `/opt/ros/jazzy/` | 1.00 | Lanelet2 地图读写。 |
| ros-jazzy-lanelet2-maps | `/opt/ros/jazzy/` | 1.00 | Lanelet2 地图相关组件。 |
| ros-jazzy-lanelet2-matching | `/opt/ros/jazzy/` | 1.00 | 轨迹与地图匹配。 |
| ros-jazzy-lanelet2-projection | `/opt/ros/jazzy/` | 1.00 | 地理坐标投影。 |
| ros-jazzy-lanelet2-python | `/opt/ros/jazzy/` | 1.00 | Lanelet2 Python 接口。 |
| ros-jazzy-lanelet2-routing | `/opt/ros/jazzy/` | 1.00 | Lanelet2 路由图与路径搜索。 |
| ros-jazzy-lanelet2-traffic-rules | `/opt/ros/jazzy/` | 1.00 | 地图交通规则。 |
| ros-jazzy-lanelet2-validation | `/opt/ros/jazzy/` | 1.00 | Lanelet2 地图验证。 |
| ros-jazzy-magic-enum | `/opt/ros/jazzy/` | 1.00 | C++ 枚举反射工具库。 |
| ros-jazzy-mrt-cmake-modules | `/opt/ros/jazzy/` | 1.00 | CMake 构建辅助模块。 |
| ros-jazzy-nmea-msgs | `/opt/ros/jazzy/` | 1.00 | NMEA/GNSS 消息类型。 |
| ros-jazzy-osqp-vendor | `/opt/ros/jazzy/` | 1.00 | OSQP 优化求解器集成。 |
| ros-jazzy-pcl-ros | `/opt/ros/jazzy/` | 1.00 | PCL 与 ROS 点云处理接口。 |
| ros-jazzy-point-cloud-msg-wrapper | `/opt/ros/jazzy/` | 1.00 | 点云消息封装。 |
| ros-jazzy-proxsuite | `/opt/ros/jazzy/` | 1.00 | 数值优化求解库。 |
| ros-jazzy-radar-msgs | `/opt/ros/jazzy/` | 1.00 | 毫米波雷达消息定义。 |
| ros-jazzy-random-numbers | `/opt/ros/jazzy/` | 1.00 | 随机数工具。 |
| ros-jazzy-rclpy-message-converter | `/opt/ros/jazzy/` | 1.00 | ROS Python 消息转换。 |
| ros-jazzy-ros-testing | `/opt/ros/jazzy/` | 1.00 | ROS 测试工具。 |
| ros-jazzy-ros2test | `/opt/ros/jazzy/` | 1.00 | ROS 2 测试命令。 |
| ros-jazzy-rqt-runtime-monitor | `/opt/ros/jazzy/` | 1.00 | RQT 运行状态监控。 |
| ros-jazzy-septentrio-gnss-driver | `/opt/ros/jazzy/` | 1.00 | Septentrio GNSS 驱动。 |
| ros-jazzy-sophus | `/opt/ros/jazzy/` | 1.00 | SE(3)、SO(3) 等位姿数学运算。 |
| ros-jazzy-tensorrt-cmake-module | `/opt/ros/jazzy/` | 1.00 | TensorRT 的 CMake 查找与集成模块。 |
| ros-jazzy-topic-tools | `/opt/ros/jazzy/` | 1.00 | ROS Topic 数据处理工具。 |
| ros-jazzy-topic-tools-interfaces | `/opt/ros/jazzy/` | 1.00 | Topic 工具接口定义。 |
| ros-jazzy-ublox-gps | `/opt/ros/jazzy/` | 1.00 | u-blox GNSS 驱动。 |
| ros-jazzy-ublox-msgs | `/opt/ros/jazzy/` | 1.00 | u-blox 消息定义。 |
| ros-jazzy-ublox-serialization | `/opt/ros/jazzy/` | 1.00 | u-blox 协议序列化。 |
| ros-jazzy-udp-msgs | `/opt/ros/jazzy/` | 1.00 | UDP 通信消息接口。 |
| ros-jazzy-velodyne-driver | `/opt/ros/jazzy/` | 1.00 | Velodyne 激光雷达驱动。 |
| ros-jazzy-velodyne-msgs | `/opt/ros/jazzy/` | 1.00 | Velodyne 点云消息。 |
| ros-jazzy-velodyne-pointcloud | `/opt/ros/jazzy/` | 1.00 | Velodyne 点云转换。 |

### 6.2 Ubuntu C/C++ 开发依赖

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ------------------ | ---------------------------- | ---: | ------------------------------ |
| clang-format | `/usr/bin/` | 0.82 | C/C++ 代码格式化工具。 |
| clang-format-18 | `/usr/bin/` | 0.82 | Clang 18 代码格式化工具。 |
| libasio-dev | `/usr/include/` | 0.57 | ASIO 异步网络通信开发库。 |
| libcgal-dev | `/usr/include/` | 0.57 | CGAL 计算几何库。 |
| libcpprest-dev | `/usr/include/` | 0.57 | C++ REST API 开发库。 |
| libcpprest2.10 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | C++ REST SDK 运行库。 |
| libcrypto++-dev | `/usr/include/` | 0.57 | Crypto++ 加密库头文件。 |
| libcrypto++8t64 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | Crypto++ 运行库。 |
| libgmp-dev | `/usr/include/` | 0.57 | GMP 多精度数学运算开发库。 |
| libgmpxx4ldbl | `/usr/lib/x86_64-linux-gnu/` | 10.39 | GMP C++ 接口运行库。 |
| libmpfr-dev | `/usr/include/` | 0.57 | 高精度浮点计算开发库。 |
| libnl-genl-3-dev | `/usr/include/` | 0.57 | Linux Netlink 网络接口开发库。 |
| libpcap0.8-dev | `/usr/include/` | 0.57 | 网络数据包捕获开发库。 |
| libpng++-dev | `/usr/include/` | 0.57 | PNG 图像 C++ 接口。 |
| librange-v3-dev | `/usr/include/` | 0.57 | C++ Range 操作库。 |
| librange-v3-doc | `/usr/share/doc/` | 0.35 | Range-v3 开发文档。 |
| libsimde-dev | `/usr/include/` | 0.57 | SIMD 指令兼容与移植。 |
| libwebsocketpp-dev | `/usr/include/` | 0.57 | WebSocket C++ 开发库。 |
| tzdata-legacy | `/usr/share/zoneinfo/` | 0.00 | 兼容旧版本时区数据。 |

### 6.3 Python 和其他系统依赖

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| -------------------- | --------------------------------- | ---: | ------------------------------------------------------ |
| python3-asgiref | `/usr/lib/python3/dist-packages/` | 0.76 | Python 异步应用接口。 |
| python3-bson | `/usr/lib/python3/dist-packages/` | 0.76 | BSON 数据编码与解码。 |
| python3-bson-ext | `/usr/lib/python3/dist-packages/` | 0.76 | BSON C 扩展。 |
| python3-flask | `/usr/lib/python3/dist-packages/` | 0.76 | Flask Web 应用框架。 |
| python3-itsdangerous | `/usr/lib/python3/dist-packages/` | 0.76 | 数据签名与验证。 |
| python3-pyasyncore | `/usr/lib/python3/dist-packages/` | 0.76 | 异步网络接口兼容库。 |
| python3-pyinotify | `/usr/lib/python3/dist-packages/` | 0.76 | Linux 文件变化通知。 |
| python3-pyproj | `/usr/lib/python3/dist-packages/` | 0.76 | 地理坐标投影转换。 |
| python3-rospkg | `/usr/lib/python3/dist-packages/` | 0.76 | ROS 软件包路径与元数据处理。 |
| python3-simplejson | `/usr/lib/python3/dist-packages/` | 0.76 | JSON 序列化工具。 |
| python3-tornado | `/usr/lib/python3/dist-packages/` | 0.76 | Python 异步网络框架。 |
| python3-werkzeug | `/usr/lib/python3/dist-packages/` | 0.76 | Web 服务基础工具。 |
| chrony | `/usr/sbin/chronyd` 等 | 0.00 | NTP 网络时间同步服务，替换了原有 `systemd-timesyncd`。 |

以上是你那次 rosdep 模拟安装日志中列出的新增软件包。对于每个 APT 包，若需要查看精确的全部安装文件，可以执行：

```
dpkg -L 软件包名称
```

例如：

```
dpkg -L ros-jazzy-lanelet2-core
```

## 七、CUDA 13.0 安装时新增的 APT 软件包

你安装 `cuda-toolkit-13-0` 时，APT 模拟显示：

```
0 upgraded, 62 newly installed, 0 to remove
```

下面进一步列出 CUDA 安装过程中涉及的主要独立软件包。它们大部分位于 `/usr/local/cuda-13.0/`，少量管理工具安装在其他系统目录。

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ------------------------------- | ------------------------------- | ---: | -------------------------------------------- |
| cuda-toolkit-13-0 | `/usr/local/cuda-13.0/` | 5.18 | CUDA 13.0 完整开发工具包。 |
| cuda-cccl-13-0 | CUDA 13.0 目录 | 5.18 | CUDA C++ 核心库。 |
| cuda-command-line-tools-13-0 | CUDA 13.0 `bin/` | 0.22 | CUDA 命令行开发工具集合。 |
| cuda-compiler-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 编译器组件集合。 |
| cuda-crt-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 编译运行支持组件。 |
| cuda-cudart-13-0 | CUDA 13.0 `lib/` | 4.54 | CUDA Runtime 运行库。 |
| cuda-cudart-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUDA Runtime 开发文件。 |
| cuda-culibos-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 操作系统抽象接口。 |
| cuda-cuobjdump-13-0 | CUDA 13.0 `bin/` | 0.22 | 分析 CUDA 编译对象文件。 |
| cuda-cupti-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 性能数据接口。 |
| cuda-cupti-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUPTI 开发文件。 |
| cuda-cuxxfilt-13-0 | CUDA 13.0 `bin/` | 0.22 | CUDA C++ 符号反修饰。 |
| cuda-documentation-13-0 | CUDA 文档目录 | 0.00 | CUDA 开发文档。 |
| cuda-driver-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUDA Driver API 开发文件，不是显卡内核驱动。 |
| cuda-gdb-13-0 | CUDA 13.0 `bin/` | 0.22 | CUDA 程序调试器。 |
| cuda-libraries-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 运行库合集。 |
| cuda-libraries-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 数学库开发文件合集。 |
| cuda-nsight-13-0 | NVIDIA 工具目录 | 0.12 | CUDA 开发分析工具。 |
| cuda-nsight-compute-13-0 | NVIDIA 工具目录 | 1.31 | GPU Kernel 性能分析。 |
| cuda-nsight-systems-13-0 | NVIDIA 工具目录 | 1.09 | CPU/GPU 系统级性能分析。 |
| cuda-nvcc-13-0 | `/usr/local/cuda-13.0/bin/nvcc` | 0.03 | CUDA 编译器。 |
| cuda-nvdisasm-13-0 | CUDA 13.0 `bin/` | 0.22 | GPU 指令反汇编工具。 |
| cuda-nvml-dev-13-0 | CUDA 13.0 目录 | 5.18 | NVIDIA GPU 管理接口开发文件。 |
| cuda-nvprune-13-0 | CUDA 13.0 `bin/` | 0.22 | 裁剪 CUDA 库中的架构代码。 |
| cuda-nvrtc-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 运行时编译库。 |
| cuda-nvrtc-dev-13-0 | CUDA 13.0 目录 | 5.18 | NVRTC 开发文件。 |
| cuda-nvtx-13-0 | CUDA 13.0 目录 | 5.18 | GPU 性能分析标记接口。 |
| cuda-opencl-13-0 | CUDA 13.0 目录 | 5.18 | OpenCL 相关组件。 |
| cuda-profiler-api-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 性能分析 API。 |
| cuda-sandbox-dev-13-0 | CUDA 13.0 目录 | 5.18 | CUDA Sandbox 开发组件。 |
| cuda-sanitizer-13-0 | CUDA 13.0 `bin/` | 0.22 | CUDA 内存错误与并发问题检测。 |
| cuda-toolkit-13-0-config-common | NVIDIA CUDA 配置目录 | 0.00 | CUDA 13.0 安装配置。 |
| cuda-toolkit-13-config-common | NVIDIA CUDA 配置目录 | 0.00 | CUDA 13 系列通用配置。 |
| cuda-toolkit-config-common | NVIDIA CUDA 配置目录 | 0.00 | CUDA Toolkit 通用安装配置。 |
| cuda-tools-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 工具合集。 |
| cuda-visual-tools-13-0 | CUDA 13.0 目录 | 5.18 | CUDA 图形与可视化开发工具。 |
| gds-tools-13-0 | NVIDIA 工具目录 | 0.06 | GPUDirect Storage 相关工具。 |
| libcublas-13-0 | CUDA 13.0 库目录 | 4.54 | GPU BLAS 矩阵运算库。 |
| libcublas-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuBLAS 开发文件。 |
| libcufft-13-0 | CUDA 13.0 库目录 | 4.54 | GPU FFT 库。 |
| libcufft-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuFFT 开发文件。 |
| libcufile-13-0 | CUDA 13.0 库目录 | 4.54 | GPUDirect Storage 文件访问库。 |
| libcufile-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuFile 开发文件。 |
| libcurand-13-0 | CUDA 13.0 库目录 | 4.54 | GPU 随机数生成库。 |
| libcurand-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuRAND 开发文件。 |
| libcusolver-13-0 | CUDA 13.0 库目录 | 4.54 | GPU 数值求解库。 |
| libcusolver-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuSOLVER 开发文件。 |
| libcusparse-13-0 | CUDA 13.0 库目录 | 4.54 | GPU 稀疏矩阵运算库。 |
| libcusparse-dev-13-0 | CUDA 13.0 目录 | 5.18 | cuSPARSE 开发文件。 |
| libnpp-13-0 | CUDA 13.0 库目录 | 4.54 | NVIDIA 图像处理库。 |
| libnpp-dev-13-0 | CUDA 13.0 目录 | 5.18 | NPP 开发文件。 |
| libnvfatbin-13-0 | CUDA 13.0 库目录 | 4.54 | GPU Fat Binary 处理库。 |
| libnvfatbin-dev-13-0 | CUDA 13.0 目录 | 5.18 | NVFatbin 开发文件。 |
| libnvjitlink-13-0 | CUDA 13.0 库目录 | 4.54 | CUDA 即时链接库。 |
| libnvjitlink-dev-13-0 | CUDA 13.0 目录 | 5.18 | NVJitLink 开发文件。 |
| libnvjpeg-13-0 | CUDA 13.0 库目录 | 4.54 | GPU JPEG 编解码。 |
| libnvjpeg-dev-13-0 | CUDA 13.0 目录 | 5.18 | nvJPEG 开发文件。 |
| libnvptxcompiler-13-0 | CUDA 13.0 库目录 | 4.54 | NVIDIA PTX 编译组件。 |
| libnvvm-13-0 | CUDA 13.0 目录 | 5.18 | NVVM 编译中间表示相关组件。 |
| nsight-compute-2025.3.1 | NVIDIA Nsight 安装目录 | 1.31 | Nsight Compute 实际版本。 |
| nsight-systems-2025.3.2 | NVIDIA Nsight 安装目录 | 1.09 | Nsight Systems 实际版本。 |
| libxcb-cursor0 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | X11 鼠标指针支持库，供部分图形工具使用。 |

其中 CUDA 的具体安装文件可以使用：

```
dpkg -L cuda-nvcc-13-0
```

查询。上述目录是主要路径概述，实际文件分布以 `dpkg -L` 为准。

## 八、TensorRT 的 8 个独立软件包

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ----------------------------- | ----------------------------------- | ---: | ----------------------------- |
| libnvinfer10 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | TensorRT 核心推理运行库。 |
| libnvinfer-dev | `/usr/lib/x86_64-linux-gnu/` 等 | 10.39 | TensorRT C++ 开发与链接文件。 |
| libnvinfer-headers-dev | `/usr/include/x86_64-linux-gnu/` | 0.04 | TensorRT 核心 API 头文件。 |
| libnvinfer-headers-plugin-dev | `/usr/include/x86_64-linux-gnu/` | 0.04 | TensorRT 插件开发头文件。 |
| libnvinfer-plugin10 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | TensorRT 插件运行库。 |
| libnvinfer-plugin-dev | `/usr/lib/x86_64-linux-gnu/` 等 | 10.39 | TensorRT 插件开发文件。 |
| libnvonnxparsers10 | `/usr/lib/x86_64-linux-gnu/` | 10.39 | ONNX 模型解析运行库。 |
| libnvonnxparsers-dev | `/usr/include/x86_64-linux-gnu/` 等 | 0.04 | ONNX Parser 开发文件。 |

以上八个软件包的安装版本均已验证为 **10.13.3.9-1+cuda13.0**。

## 九、安装时生成的其他文件

这些文件不是独立安装的软件，但也是本次工作产生的重要内容。

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| --------------------- | ------------------------------------- | ---: | ------------------------------------- |
| rosdep 模拟安装日志 | `/tmp/autoware-rosdep-simulate.log` | 0.00 | 记录 Autoware 系统依赖安装计划。 |
| APT 依赖模拟日志 | `/tmp/autoware-apt-simulate.log` | 0.00 | 记录 82 个软件包及时间同步服务变更。 |
| Core 编译日志 | `/tmp/autoware-core-build.log` | 0.00 | 保存 Core 编译输出。 |
| CUDA 安装模拟日志 | `/tmp/cuda-13-install-simulate.log` | 0.00 | 保存 CUDA Toolkit 的 APT 安装计划。 |
| TensorRT 安装模拟日志 | `/tmp/autoware-tensorrt-simulate.log` | 0.00 | 保存 TensorRT 的安装计划。 |
| spconv CMake 验证项目 | `/tmp/autoware-spconv-check/` | 0.00 | 验证 cumm、spconv 及 CMake 链接目标。 |
| CUDA 环境变量 | `~/.bashrc`（如已写入） | 0.00 | 配置 CUDA_HOME 和 CUDA 命令搜索路径。 |
| acados 环境变量 | 当前终端会话 | 非磁盘文件 | 为 Autoware 编译提供 acados 路径。 |

**特别提醒：`/tmp` 目录中的日志不保证长期保留。** 如果以后需要追溯安装历史，建议复制到 `~/autoware/` 下的备份目录。

## 十、安装目录总览

按照我们这次的安装流程，最重要的目录是：

| 名称 | 安装路径 | 磁⁠盘⁠占⁠用⁠（⁠G⁠B⁠） | 功能概述 |
| ------------------- | ---------------------------------- | ---: | ---------------------------------------------- |
| Autoware 工作区 | `/home/maple/autoware/` | 7.27 | 主源码、编译结果和 ROS 2 工作区。 |
| spconv_cpp 工作区 | `/home/maple/spconv_cpp/` | 2.88 | cumm/spconv 源码、CUDA 编译文件和 DEB 安装包。 |
| acados | `/opt/acados/` | 1.01 | 优化算法库、Tera Renderer 和 Python 虚拟环境。 |
| CUDA Toolkit | `/usr/local/cuda-13.0/` | 5.18 | CUDA 13.0 开发环境。 |
| TensorRT | `/usr/lib/x86_64-linux-gnu/` 等 | 10.39 | TensorRT 运行库、插件和头文件。 |
| cumm | `/usr/local/` | 15.41 | GPU 计算基础库。 |
| spconv | `/usr/local/` | 15.41 | GPU 稀疏卷积库。 |
| ROS 2 Jazzy（原有） | `/opt/ros/jazzy/` | 1.00 | ROS 2 基础环境，以及本次新增的 ROS 依赖。 |
| GeographicLib 数据 | `/usr/share/GeographicLib/geoids/` | 0.47 | 高程模型数据。 |
| Ubuntu APT 管理目录 | `/var/lib/dpkg/` | 0.15 | 系统已安装软件包的管理记录。 |

### 如何获得系统中每个软件包的精确安装路径？

上面这份表是根据**我们的安装过程、你返回的成功日志和 APT 模拟记录**整理的，覆盖了已确认的主要新增组件及依赖。它不是对当前整台 Ubuntu 进行实时扫描，因此不应把它视为系统软件变更的审计结果。

你可以通过下面的命令查看任意软件包的实际安装文件：

```
dpkg -L spconv
dpkg -L cumm
dpkg -L libnvinfer10
dpkg -L cuda-toolkit-13-0
dpkg -L geographiclib-tools
```

如果需要以后维护和卸载，**最重要的是保留三个源码/开发目录**：

`~/autoware`、`~/spconv_cpp` 和 `/opt/acados`。

但需要注意：**不要认为删除这些目录就等于卸载软件。** CUDA、TensorRT、cumm、spconv，以及通过 APT 安装的 ROS 依赖，已经安装到系统目录中，需要由 APT 分别管理。