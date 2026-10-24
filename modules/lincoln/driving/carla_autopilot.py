"""默认控制器：使用 CARLA Traffic Manager，跟随世界的同步/异步状态。"""
import time
from modules.lincoln.driving.controller import DrivingController
from modules.lincoln.driving.traffic_manager_port import reserve_traffic_manager_port

MODE_CHECK_INTERVAL = 0.5


class CarlaAutopilotController(DrivingController):
    def __init__(self, client, world, vehicle, args):
        super().__init__(client, world, vehicle, args)
        self.manager = None
        self.port = None
        self.enabled = False
        self.mode = None
        self.next_check = 0

    def refresh_clock_mode(self):
        mode = self.world.get_settings().synchronous_mode
        if mode != self.mode:
            self.manager.set_synchronous_mode(mode)
            self.mode = mode
        self.next_check = time.monotonic() + MODE_CHECK_INTERVAL

    def enable(self):
        if self.enabled:
            return
        # 首次准备在调用线程完成，不在后台偷偷创建 Traffic Manager。
        if self.manager is None:
            with reserve_traffic_manager_port(self.args.tm_port) as port:
                self.manager = self.client.get_trafficmanager(port)
                self.port = port
            print(f'[Lincoln] Traffic Manager 已就绪，端口 {self.port}', flush=True)
        self.refresh_clock_mode()
        self.vehicle.set_autopilot(True, self.port)
        self.enabled = True

    def disable(self):
        if self.enabled:
            self.vehicle.set_autopilot(False, self.port)
            self.enabled = False

    @property
    def traffic_manager_port(self):
        return self.port if self.enabled else None

    def update(self, snapshot):
        if self.enabled and time.monotonic() >= self.next_check:
            self.refresh_clock_mode()
        return None
