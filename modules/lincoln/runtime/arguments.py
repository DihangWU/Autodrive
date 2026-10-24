"""Lincoln 启动参数；帮助信息不依赖 CARLA、ROS 2 或图形界面。"""
import argparse
import math
from pathlib import Path
from modules.lincoln.runtime.paths import RVIZ_TEMPLATE_FILE
from foundation.cobridge.control import SOCKET_FILE

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2000
DEFAULT_TM_PORT = 8000
DEFAULT_TIMEOUT = 15.0


def parse_lincoln_arguments():
    parser = argparse.ArgumentParser(description='直接在 CARLA 添加 Lincoln，向 CoBridge 登记后同步到 SUMO。')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--tm-port', type=int, default=DEFAULT_TM_PORT, help='Traffic Manager 首选端口；被占用时从此端口起查找空闲端口')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    position = parser.add_mutually_exclusive_group()
    position.add_argument('--pose', type=float, nargs=4, metavar=('X', 'Y', 'Z', 'YAW'), help='CARLA 世界坐标（米）与 yaw（度）；保留 z，不自动吸附道路')
    position.add_argument('--spawn-index', type=int, help='使用 CARLA 地图出生点编号')
    parser.add_argument('--sensors', choices=['all', 'lidar', 'none'], default='all')
    parser.add_argument('--camera-rows', type=int, choices=[4, 6], default=4)
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--rviz-config', type=Path, default=RVIZ_TEMPLATE_FILE)
    parser.add_argument('--headless', action='store_true', help='不显示驾驶面板；不等于关闭传感器或 RViz')
    parser.add_argument('--duration', type=float, default=0, help='运行现实秒数后退出；0 为持续运行')
    parser.add_argument('--autopilot', action='store_true', help='生成后立即开启选定驾驶算法；默认手动驾驶')
    parser.add_argument('--driver', default='carla', help='carla 使用 Traffic Manager；或填写 Python模块:类名')
    parser.add_argument('--bridge-socket', type=Path, default=SOCKET_FILE)
    args = parser.parse_args()
    if (not 1 <= args.port <= 65535 or not 1024 <= args.tm_port <= 65535
            or args.port == args.tm_port or not math.isfinite(args.timeout) or args.timeout <= 0
            or not math.isfinite(args.duration) or args.duration < 0
            or (args.pose and not all(math.isfinite(value) for value in args.pose))
            or (args.spawn_index is not None and args.spawn_index < 0)):
        parser.error('端口、出生坐标、超时或运行时长无效。')
    return args
