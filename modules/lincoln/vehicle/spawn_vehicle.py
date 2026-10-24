"""直接连接 CARLA 生成 Lincoln；绝不创建 SUMO 或推进时钟。"""
import random
import time
import carla


MIN_SPAWN_DISTANCE = 7.0
SNAPSHOT_WAIT_INTERVAL = 0.05


class SpawnVehicleMixin:
    def spawn_car(self):
        bp = self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        bp.set_attribute('role_name', self.uid)
        bp.set_attribute('color', '30,90,230')
        points = self.world.get_map().get_spawn_points()
        if self.args.pose:
            x, y, z, yaw = self.args.pose
            points = [carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))]
        elif self.args.spawn_index is not None:
            if not 0 <= self.args.spawn_index < len(points):
                raise ValueError(f'spawn-index must be between 0 and {len(points) - 1}')
            points = [points[self.args.spawn_index]]
        else:
            occupied = [a.get_location() for a in self.world.get_actors().filter('vehicle.*')]
            points = [p for p in points if all(p.location.distance(v) > MIN_SPAWN_DISTANCE for v in occupied)]
            random.shuffle(points)
        for transform in points:
            self.car = self.world.try_spawn_actor(bp, transform)
            if self.car:
                self.actors.append(self.car)
                # Newly spawned vehicles are manual by default. Calling
                # set_autopilot(False) would unnecessarily create/contact a TM.
                self.car.apply_control(carla.VehicleControl(brake=1.))
                # spawn RPC can complete before the client receives the next
                # actor snapshot. is_alive is temporarily false in that gap.
                deadline = time.monotonic() + self.args.timeout
                while time.monotonic() < deadline:
                    snapshot = self.world.get_snapshot()
                    if snapshot.find(self.car.id) is not None and self.car.is_alive:
                        return
                    time.sleep(SNAPSHOT_WAIT_INTERVAL)
                self.car.destroy()
                raise RuntimeError('车辆未出现在 CARLA 新帧中；若世界为同步模式，请检查是否有时钟持续推进。')
        raise RuntimeError('无法在指定位置或可用出生点生成车辆；请检查碰撞或更换 --pose / --spawn-index。')

