"""默认控制器：使用 CARLA Traffic Manager，跟随世界的同步/异步状态。"""
import logging
import time
from modules.lincoln.driving.controller import DrivingController
from modules.lincoln.driving.traffic_manager_port import reserve_traffic_manager_port

MODE_CHECK_INTERVAL = 0.5
DEFAULT_SPEED_DIFFERENCE = 30.0  # 官方 TM 默认比道路限速低 30%。


class CarlaAutopilotController(DrivingController):
    supports_junction_local_control = True

    def __init__(self, client, world, vehicle, args):
        super().__init__(client, world, vehicle, args)
        self.manager = None
        self.port = None
        self.enabled = False
        self.mode = None
        self.next_check = 0
        self.navigation_speed = None
        # 车辆启动前台准备，不把创建服务、加载地图等冷启动留给驾驶按钮。
        started = time.monotonic()
        print('[Lincoln] 正在准备 Traffic Manager…', flush=True)
        with reserve_traffic_manager_port(self.args.tm_port) as port:
            self.manager = self.client.get_trafficmanager(port)
            self.port = port
        self.refresh_clock_mode()
        logging.info('TM 初始化完成：端口 %s，耗时 %.3f 秒', self.port, time.monotonic() - started)
        print(f'[Lincoln] Traffic Manager 已就绪，端口 {self.port}', flush=True)

    def refresh_clock_mode(self):
        mode = self.world.get_settings().synchronous_mode
        if mode != self.mode:
            self.manager.set_synchronous_mode(mode)
            self.mode = mode
        self.next_check = time.monotonic() + MODE_CHECK_INTERVAL

    def enable(self):
        if self.enabled:
            return
        started = time.monotonic()
        self.refresh_clock_mode()
        self.vehicle.set_autopilot(True, self.port)
        self.enabled = True
        logging.info('自动驾驶启用：端口 %s，耗时 %.3f 秒', self.port, time.monotonic() - started)

    def disable(self):
        if self.enabled:
            self.vehicle.set_autopilot(False, self.port)
            self.enabled = False

    def set_route(self, locations):
        """沿已验证的 CARLA 路线驾驶，交通规则和避障继续由 TM 负责。"""
        self.manager.set_path(self.vehicle, locations, True)
        self.enable()

    def clear_route(self):
        self.manager.set_path(self.vehicle, [], True)
        if self.navigation_speed is not None:
            self.manager.vehicle_percentage_speed_difference(self.vehicle, DEFAULT_SPEED_DIFFERENCE)
            self.navigation_speed = None

    def set_navigation_speed(self, speed_kmh):
        """仅设置 TM 目标速度，减速时不注销车辆、不丢弃前瞻路点。"""
        speed_kmh = max(0., round(speed_kmh, 1))
        if speed_kmh != self.navigation_speed:
            self.manager.set_desired_speed(self.vehicle, speed_kmh)
            self.navigation_speed = speed_kmh

    def navigation_preview(self):
        """读取本车 TM 的真实动作路点；它们不是完整的内部轨迹。"""
        if not self.enabled:
            return []
        actions = self.manager.get_all_actions(self.vehicle)
        points = [waypoint.transform.location for _, waypoint in actions if waypoint is not None]
        # 直道常只有缓冲起点；LaneFollow 的 next_action 指向缓冲末端。
        action, waypoint = self.manager.get_next_action(self.vehicle)
        if waypoint is not None and action == 'LaneFollow':
            location = waypoint.transform.location
            if not points or points[-1].distance(location) > .1:
                points.append(location)
        return points

    @property
    def traffic_manager_port(self):
        return self.port  # 手动模式也提前向桥登记，开关按钮不再触发首次连接。

    def update(self, snapshot):
        if self.enabled and time.monotonic() >= self.next_check:
            self.refresh_clock_mode()
        return None
