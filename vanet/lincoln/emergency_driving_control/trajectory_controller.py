"""在全程轨迹上复用现有 Pure Pursuit / PID；保持独立于 Traffic Manager。"""
import bisect
import math
from types import SimpleNamespace
import carla
from modules.lincoln.driving.junction_local_controller import JunctionLocalController
from .collision_guard import collision_speed_limit
from .guard_actuation import GuardActuation
from .control_configuration import load_collision_configuration


class EmergencyTrajectoryController(JunctionLocalController):
    def find_junction_windows(self):
        return [dict(start=0., entry=0., exit=self.route_lengths[-1], exit_index=len(self.route)-1)]

    def __init__(self, node, route, road_map, maximum_speed=120.):
        lengths = [0.]
        for first, second in zip(route, route[1:]):
            lengths.append(lengths[-1] + first.transform.location.distance(second.transform.location))
        super().__init__(node, route, road_map, lengths)
        self.ignore_traffic_lights = True
        self.maximum_speed = math.inf if maximum_speed is None else maximum_speed
        self.original_samples = tuple(self.samples)
        self.target_speed = 0.
        self.borrowing = False
        self.borrow_until = None
        self.collision_configuration=load_collision_configuration()
        self.collision_obstacles=None
        self.collision_guard_state=None
        self.collision_active_actors=set()
        self.collision_minimum_brake=0.
        self.guard_actuation=GuardActuation(self.collision_configuration)
        self.longitudinal_state=None
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
        minimum=getattr(self,'collision_minimum_brake',0.)
        if not hasattr(self,'guard_actuation'):
            self.guard_actuation=GuardActuation(getattr(self,'collision_configuration',None) or load_collision_configuration())
        throttle,brake=self.guard_actuation.apply(throttle,brake,minimum,bool(getattr(self,'collision_active_actors',set())),dt)
        if self.guard_actuation.recovering:self.integral=0.
        self.longitudinal_state=dict(target_speed_kmh=self.target_speed,throttle=throttle,brake=brake,
            guard_active=bool(getattr(self,'collision_active_actors',set())),recovering=self.guard_actuation.recovering)
        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)

    def obstacle_speed_limit(self,snapshot,target,speed,now):
        if self.collision_obstacles is None:
            return super().obstacle_speed_limit(snapshot,target,speed,now)
        return collision_speed_limit(self,self.collision_obstacles,target,speed)

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
