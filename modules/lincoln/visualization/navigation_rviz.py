"""导航专用 RViz：独立窗口、复用现有车道与车轮、后台准备静态路线。

与 --no-sensors/--no-rviz 无关；只在接受有效路线后启动，每车一个窗口。
导航取消后保留窗口并清空路线，到达后保留路线，车辆退出时关闭窗口。
"""
import logging
import queue
import signal
import subprocess
import threading
import time

from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray

from modules.lincoln.runtime.paths import NAV_RVIZ_TEMPLATE_FILE
from modules.lincoln.visualization.navigation_route import clear_route_markers, navigation_route_markers, tm_path_markers
from modules.lincoln.visualization.rviz_launcher import launch_rviz_template
from modules.lincoln.visualization.road_map import road_markers

RVIZ_EXIT_TIMEOUT = 3.0
WORKER_JOIN_TIMEOUT = 5.0
PREPARATION_TIMEOUT = 30.0
READINESS_POLL_INTERVAL = 0.05
SUBSCRIBER_SETTLE_SECONDS = 0.5
CONFIG_NAME = 'lincoln_nav.rviz'
LOG_NAME = 'lincoln_nav_rviz.log'


class NavigationRviz:
    def __init__(self, node):
        self.node = node
        self.route_pub = node.create_publisher(MarkerArray, f'/{node.uid}/navigation/route',
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vehicle_pub = node.create_publisher(MarkerArray, f'/{node.uid}/navigation/vehicle', 2)
        self.tm_path_pub = node.create_publisher(MarkerArray, f'/{node.uid}/navigation/tm_path', 1)
        self.lock = threading.RLock()
        self.stopped = threading.Event()
        self.tasks = queue.Queue(maxsize=1)
        self.worker = None
        self.process = None
        self.generation = 0
        self.prepared_generation = 0
        self.preparation_error = None
        self.window_ready = False
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
            self.preparation_error = None
            try:
                self.tasks.get_nowait()
            except queue.Empty:
                pass
            self.tasks.put_nowait((generation, list(waypoints), road_map))
            if self.worker is None:
                self.worker = threading.Thread(target=self.prepare_views, name='lincoln-navigation-rviz', daemon=True)
                self.worker.start()
            return generation

    def prepare_navigation(self, waypoints, road_map, cancelled):
        """仅规划线程等待首次地图、路线和 RViz 订阅就绪，驾驶线程持续制动。"""
        generation = self.show_route(waypoints, road_map)
        deadline = time.monotonic() + PREPARATION_TIMEOUT
        connected_since = None
        while time.monotonic() < deadline and not self.stopped.is_set() and not cancelled():
            with self.lock:
                if generation != self.generation:
                    return False
                error = self.preparation_error
                prepared = self.prepared_generation == generation
                process, ready = self.process, self.window_ready
            if error is not None and error[0] == generation:
                raise RuntimeError(error[1])
            if prepared and process is not None:
                if process.poll() is not None:
                    raise RuntimeError("RViz exited during startup; check lincoln_nav_rviz.log.")
                if ready:
                    return True
                # 订阅出现后留出短暂稳定时间；没有从驾驶线程等待窗口的循环。
                connected = (self.has_vehicle_subscribers()
                             and self.route_pub.get_subscription_count() > 0
                             and self.node.map_pub.get_subscription_count() > 0)
                if connected:
                    if connected_since is None:
                        connected_since = time.monotonic()
                    if time.monotonic() - connected_since >= SUBSCRIBER_SETTLE_SECONDS:
                        with self.lock:
                            self.window_ready = True
                        return True
                else:
                    connected_since = None
            self.stopped.wait(READINESS_POLL_INTERVAL)
        if self.stopped.is_set() or cancelled():
            return False
        raise RuntimeError("RViz preparation timed out; check lincoln_nav_rviz.log.")

    def clear_route(self):
        with self.lock:
            self.generation += 1
            self.route_pub.publish(clear_route_markers())
            self.tm_path_pub.publish(clear_route_markers())

    def show_tm_path(self, location, waypoints):
        """仅显示本车控制器刚返回的动作点；失效后自动清除。"""
        self.tm_path_pub.publish(tm_path_markers(location, waypoints))

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
                        self.window_ready = False
                        self.process = launch_rviz_template(self.node, NAV_RVIZ_TEMPLATE_FILE,
                                                            CONFIG_NAME, LOG_NAME,
                                                            ros_node_name=self.node.uid + '_navigation_rviz')
                    self.prepared_generation = generation
            except Exception:
                with self.lock:
                    self.preparation_error = (generation, 'Map or RViz preparation failed; check Lincoln logs.')
                logging.exception('导航 RViz 准备失败；车辆保持停车，详情见 Lincoln 日志')

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
