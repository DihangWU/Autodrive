"""手动驾驶和定速控制；自动驾驶交给可替换的 DrivingController。"""
import math
import carla

FORWARD_THROTTLE = 0.75
REVERSE_THROTTLE = 0.45


class ManualDrivingMixin:
    def drive(self):
        with self.control_lock:
            if self.autopilot:
                return  # Only Traffic Manager may control the car in this mode.
            active = self.active or ''
            if 'go' in active or 'brake' in active:
                self.cruise_speed = None
            throttle = (REVERSE_THROTTLE if self.reverse else FORWARD_THROTTLE) if 'go' in active else 0.
            if self.cruise_speed is not None:
                velocity = self.car.get_velocity()
                speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                now = self.world.get_snapshot().timestamp.elapsed_seconds
                dt = 0. if self.cruise_time is None else max(0., min(now - self.cruise_time, .2))
                self.cruise_time = now
                error = self.cruise_speed - speed
                self.cruise_integral = max(0., min(.85, self.cruise_integral + .08 * error * dt))
                throttle = max(0., min(.85, self.cruise_integral + .25 * error))
            control = carla.VehicleControl(throttle=throttle,
                steer=-.45 if 'left' in active else (.45 if 'right' in active else 0.),
                brake=1. if 'brake' in active else 0., reverse=self.reverse)
            if self.car.is_alive:
                self.car.apply_control(control)


    def set_autopilot(self, enabled):
        """面板入口保持不变，具体算法由 --driver 指定的实现提供。"""
        with self.control_lock:
            if enabled == self.autopilot:
                return
            if enabled:
                self.driver.enable()
            else:
                self.driver.disable()
            self.autopilot = enabled
            self.active = None
            self.cruise_speed = None
            self.reverse = False

    def update_automatic_driving(self, snapshot=None):
        """由独立工作线程调用，不与 ROS 传感器发布争用事件循环。"""
        with self.control_lock:
            if self.autopilot:
                control = self.driver.update(snapshot or self.world.get_snapshot())
                if control is not None:
                    self.car.apply_control(control)
