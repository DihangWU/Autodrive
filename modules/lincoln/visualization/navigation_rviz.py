"""导航专用 RViz：独立窗口、复用现有车道与车轮、后台准备静态路线。

与 --no-sensors/--no-rviz 无关；只在接受有效路线后启动，每车一个窗口。
导航取消后保留窗口并清空路线，到达后保留路线，车辆退出时关闭窗口。
"""
import logging
import queue
import signal
import subprocess
import threading

from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray

from modules.lincoln.runtime.paths import NAV_RVIZ_TEMPLATE_FILE
from modules.lincoln.visualization.navigation_route import clear_route_markers, navigation_route_markers
from modules.lincoln.visualization.rviz_launcher import launch_rviz_template
from modules.lincoln.visualization.road_map import road_markers

RVIZ_EXIT_TIMEOUT = 3.0
WORKER_JOIN_TIMEOUT = 5.0
CONFIG_NAME = 'lincoln_nav.rviz'
LOG_NAME = 'lincoln_nav_rviz.log'


class NavigationRviz:
    def __init__(self, node):
        self.node = node
        self.route_pub = node.create_publisher(MarkerArray, f'/{node.uid}/navigation/route',
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vehicle_pub = node.create_publisher(MarkerArray, f'/{node.uid}/navigation/vehicle', 2)
        self.lock = threading.RLock()
        self.stopped = threading.Event()
        self.tasks = queue.Queue(maxsize=1)
        self.worker = None
        self.process = None
        self.generation = 0
        self.roads_ready = bool(node.roads.markers)

    def has_vehicle_subscribers(self):
        return self.vehicle_pub.get_subscription_count() > 0

    def publish_vehicle(self):
        """原封不动复用正在发布的车身与四轮，排除传感器安装点。"""
        if self.has_vehicle_subscribers():
            self.vehicle_pub.publish(MarkerArray(markers=[self.node.model, *self.node.wheel_markers]))

    def show_route(self, waypoints, road_map):
        """驾驶线程只提交任务，不生成地图网格或等待 RViz 启动。"""
        with self.lock:
            if self.stopped.is_set():
                return
            self.generation += 1
            generation = self.generation
            try:
                self.tasks.get_nowait()
            except queue.Empty:
                pass
            self.tasks.put_nowait((generation, list(waypoints), road_map))
            if self.worker is None:
                self.worker = threading.Thread(target=self.prepare_views, name='lincoln-navigation-rviz', daemon=True)
                self.worker.start()

    def clear_route(self):
        with self.lock:
            self.generation += 1
            self.route_pub.publish(clear_route_markers())

    def prepare_roads(self, road_map):
        if self.roads_ready:
            return
        # --no-sensors 默认不打开普通 RViz，因此在第一次导航时才补齐车道。
        roads = self.node.roads if self.node.roads.markers else road_markers(road_map)
        self.node.roads = roads
        self.node.map_pub.publish(roads)
        self.roads_ready = True

    def prepare_views(self):
        while not self.stopped.is_set():
            try:
                generation, waypoints, road_map = self.tasks.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                self.prepare_roads(road_map)
                markers = navigation_route_markers(waypoints, road_map)
                with self.lock:
                    if self.stopped.is_set() or generation != self.generation:
                        continue
                    self.route_pub.publish(markers)
                    if self.process is None or self.process.poll() is not None:
                        self.process = launch_rviz_template(self.node, NAV_RVIZ_TEMPLATE_FILE,
                                                            CONFIG_NAME, LOG_NAME,
                                                            ros_node_name=self.node.uid + '_navigation_rviz')
            except Exception:
                logging.exception('导航 RViz 准备失败；车辆导航继续运行，详情见 Lincoln 日志')

    def close(self):
        self.stopped.set()
        if self.worker is not None:
            self.worker.join(WORKER_JOIN_TIMEOUT)
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                return
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=RVIZ_EXIT_TIMEOUT)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
