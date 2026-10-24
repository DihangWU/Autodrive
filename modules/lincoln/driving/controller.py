"""可替换驾驶算法接口；算法返回 CARLA VehicleControl，不持有仿真时钟。"""
from abc import ABC, abstractmethod
import importlib

DEFAULT_DRIVER = 'carla'


class DrivingController(ABC):
    def __init__(self, client, world, vehicle, args):
        self.client, self.world, self.vehicle, self.args = client, world, vehicle, args

    @abstractmethod
    def enable(self):
        """接管本车控制；不要调用 world.tick()。"""

    @abstractmethod
    def disable(self):
        """释放本车控制。"""

    @abstractmethod
    def update(self, snapshot):
        """返回 VehicleControl；外部控制器已施加控制时返回 None。"""

    @property
    def traffic_manager_port(self):
        return None

    def close(self):
        self.disable()


def load_driving_controller(spec, client, world, vehicle, args):
    if spec == DEFAULT_DRIVER:
        from modules.lincoln.driving.carla_autopilot import CarlaAutopilotController
        controller = CarlaAutopilotController
    else:
        module, separator, name = spec.partition(':')
        if not separator:
            raise ValueError('--driver 应为 carla 或 Python模块:类名。')
        controller = getattr(importlib.import_module(module), name)
        if not isinstance(controller, type) or not issubclass(controller, DrivingController):
            raise TypeError('自定义驾驶类必须继承 DrivingController。')
    return controller(client, world, vehicle, args)
