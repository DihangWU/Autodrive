"""在全程轨迹上复用现有 Pure Pursuit / PID；保持独立于 Traffic Manager。"""
import bisect
import math
from types import SimpleNamespace
import carla
from modules.lincoln.driving.junction_local_controller import JunctionLocalController


class EmergencyTrajectoryController(JunctionLocalController):
    def find_junction_windows(self):
        return [dict(start=0., entry=0., exit=self.route_lengths[-1], exit_index=len(self.route)-1)]

    def __init__(self, node, route, road_map, maximum_speed=120.):
        lengths = [0.]
        for first, second in zip(route, route[1:]):
            lengths.append(lengths[-1] + first.transform.location.distance(second.transform.location))
        super().__init__(node, route, road_map, lengths)
        self.ignore_traffic_lights = True
        self.maximum_speed = maximum_speed
        self.original_samples = tuple(self.samples)
        self.target_speed = 0.
        self.borrowing = False
        self.borrow_until = None
        self.activate(0.)

    def replace_samples(self, samples, borrowing=False, borrow_until=None):
        self.samples = list(samples)
        self.distances = [0.]
        for first, second in zip(self.samples, self.samples[1:]):
            self.distances.append(self.distances[-1] + math.dist(first.center, second.center))
        self.borrowing, self.borrow_until = borrowing, borrow_until

    def run_step(self, snapshot, speed_limit_kmh=None):
        actor = snapshot.find(self.vehicle.id)
        if actor is None:
            raise RuntimeError('Lincoln is absent from the current CARLA snapshot.')
        speed = actor.get_velocity().length()
        now = snapshot.timestamp.elapsed_seconds
        dt = .05 if self.last_time is None else max(.01, min(.2, now-self.last_time))
        self.last_time = now
        steer = self.lateral_control(actor.get_transform(), speed, dt)
        self.wait_reason = ''
        target = min(self.maximum_speed, self.maximum_speed if speed_limit_kmh is None else speed_limit_kmh) / 3.6
        target = self.curve_speed_limit(target, speed)
        remaining = max(0., self.distances[-1] - self.progress)
        target = min(target, math.sqrt(2. * 2.5 * max(0., remaining-2.)))
        target = self.obstacle_speed_limit(snapshot, target, speed, now)
        self.target_speed = target * 3.6
        throttle, brake = self.longitudinal_control(speed, target, dt)
        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)

    def route_payload(self, maximum_points=600):
        begin = max(0, bisect.bisect_left(self.distances, self.progress)-1)
        remaining = self.samples[begin:]
        stride = max(1, math.ceil(len(remaining)/maximum_points))
        selected = remaining[::stride]
        if remaining and selected[-1] is not remaining[-1]:
            selected.append(remaining[-1])
        return [dict(x=round(s.center[0], 3), y=round(s.center[1], 3), z=round(s.center[2], 3)) for s in selected]

    def display_waypoints(self):
        """让既有 RViz 绘制实际借道轨迹，方向取行驶切线而非对向车道朝向。"""
        result=[]
        for index,sample in enumerate(self.samples):
            forward=sample.forward
            rotation=carla.Rotation(yaw=math.degrees(math.atan2(forward[1],forward[0])),
                                    pitch=math.degrees(math.atan2(forward[2],math.hypot(forward[0],forward[1]))))
            result.append(SimpleNamespace(transform=carla.Transform(carla.Location(*sample.center),rotation),
                                          lane_width=sample.width,road_id=-1,lane_id=-1,s=float(index)))
        return result
