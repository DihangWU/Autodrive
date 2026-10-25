"""分流出口局部驾驶：沿绿色路线接管转向与速度，稳定驶出后交还 TM。

Pure Pursuit 负责转向，纵向 PID 负责速度；红黄灯及轨迹上的动态障碍物
触发减速停车等待，不主动绕过障碍物。使用 CARLA 快照的仿真时间，
普通路口直行、转弯及换道保持 TM；只为能确认的主线分流出口建立接管区间。
不创建传感器、不推进世界、不修改地图或交通灯。控制权切换由 navigation.py 负责。
"""
import bisect
import math
import xml.etree.ElementTree as ET

import carla

APPROACH_DISTANCE = 30.0
LANE_CHANGE_LEAD = 8.0
EXIT_DISTANCE = 8.0
EXIT_STABLE_SECONDS = 0.5
EXIT_LATERAL_ERROR = 0.6
EXIT_HEADING_ERROR = 10.0
LOOKAHEAD_MIN = 3.5
LOOKAHEAD_MAX = 9.0
LOOKAHEAD_SECONDS = 0.7
STEER_RATE = 1.5
OBSTACLE_REFRESH = 0.25
OBSTACLE_MARGIN = 0.4
BRAKING_DECELERATION = 2.5
MAX_LATERAL_ACCELERATION = 10.0
SPEED_KP, SPEED_KI, SPEED_KD = 0.9, 0.35, 0.05
SPEED_INTEGRAL_LIMIT = 10.0
MAX_THROTTLE = 1.0
EXIT_BRANCH_PROBE_DISTANCE = 18.0
EXIT_BRANCH_MAX_ANGLE = 45.0


def heading_error(first, second):
    """返回两个朝向的最小夹角，单位为度。"""
    return abs((first - second + 180.) % 360. - 180.)


def junction_spans(waypoints):
    """按 junction_id 分组，给出口识别与 TM 普通路口引导共用。"""
    index = 0
    while index < len(waypoints):
        if not waypoints[index].is_junction:
            index += 1
            continue
        entry = index
        junction_id = waypoints[entry].junction_id
        while (index < len(waypoints) and waypoints[index].is_junction
               and waypoints[index].junction_id == junction_id):
            index += 1
        yield entry, min(index, len(waypoints)-1)


class ExitBranchClassifier:
    """识别驶离主线的较窄、缓分岔支路；普通十字路口转向不因此接管。"""
    def __init__(self, road_map):
        root = ET.fromstring(road_map.to_opendrive())
        self.roads = {int(road.get('id')): road for road in root.findall('road')}
        self.connections = {int(junction.get('id')): list(junction.findall('connection'))
                            for junction in root.findall('junction')}
        self.junction_pairs = {}

    def lane_count(self, road_id, lane_id):
        """只统计相同行驶侧的 driving 车道，避开对向车道与人行道。"""
        road = self.roads.get(road_id)
        if road is None:
            return 0
        side = 'left' if lane_id > 0 else 'right'
        return max((sum(lane.get('type') == 'driving' for lane in section.findall(side+'/lane'))
                    for section in road.findall('lanes/laneSection')), default=0)

    def outgoing_lane(self, connection, incoming_id):
        """连接道路的另一端给出出口道路及驶入方向，不能用 road_id 变化代表出口。"""
        road = self.roads.get(int(connection.get('connectingRoad')))
        if road is None:
            return None
        for link in road.findall('link/*'):
            if link.get('elementType') != 'road' or int(link.get('elementId')) == incoming_id:
                continue
            outgoing_id = int(link.get('elementId'))
            outgoing = self.roads.get(outgoing_id)
            if outgoing is None or outgoing.get('junction', '-1') != '-1':
                continue
            lane_id = -1 if link.get('contactPoint') == 'start' else 1
            if outgoing.get('rule', 'RHT') == 'LHT':
                lane_id = -lane_id
            return outgoing_id, lane_id
        return None

    def gentle_branch(self, entry, connector_id, incoming_yaw):
        """看分岔起始一段的方向，排除普通约 90° 左右转。"""
        junction_id = entry.junction_id
        if junction_id not in self.junction_pairs:
            self.junction_pairs[junction_id] = entry.get_junction().get_waypoints(carla.LaneType.Driving)
        for start, _ in self.junction_pairs[junction_id]:
            if start.road_id != connector_id or heading_error(start.transform.rotation.yaw, incoming_yaw) > EXIT_BRANCH_MAX_ANGLE:
                continue
            for probe in start.next(EXIT_BRANCH_PROBE_DISTANCE):
                if heading_error(probe.transform.rotation.yaw, incoming_yaw) <= EXIT_BRANCH_MAX_ANGLE:
                    return True
        return False

    def requires_exit_selection(self, entry, incoming=None):
        """必须同时存在主线延续和较窄分流，且当前规划选择的是后者。"""
        connections = self.connections.get(entry.junction_id, [])
        selected = [connection for connection in connections
                    if int(connection.get('connectingRoad')) == entry.road_id]
        if incoming is not None:
            selected = [connection for connection in selected
                        if int(connection.get('incomingRoad')) == incoming.road_id]
        for chosen in selected:
            incoming_id = int(chosen.get('incomingRoad'))
            links = chosen.findall('laneLink')
            if not links:
                continue
            incoming_lane = incoming.lane_id if incoming is not None else int(links[0].get('from'))
            incoming_count = self.lane_count(incoming_id, incoming_lane)
            chosen_exit = self.outgoing_lane(chosen, incoming_id)
            if chosen_exit is None or incoming_count < 2:
                continue
            chosen_count = self.lane_count(*chosen_exit)
            incoming_yaw = incoming.transform.rotation.yaw if incoming is not None else entry.transform.rotation.yaw
            if chosen_count < 1 or not self.gentle_branch(entry, entry.road_id, incoming_yaw):
                continue
            for alternative in connections:
                if int(alternative.get('incomingRoad')) != incoming_id:
                    continue
                same_direction = any(int(link.get('from')) * incoming_lane > 0
                                     for link in alternative.findall('laneLink'))
                alternative_exit = self.outgoing_lane(alternative, incoming_id)
                if not same_direction or alternative_exit is None or alternative_exit[0] == chosen_exit[0]:
                    continue
                count = self.lane_count(*alternative_exit)
                if (count >= incoming_count and count > chosen_count
                        and self.gentle_branch(entry, int(alternative.get('connectingRoad')), incoming_yaw)):
                    return True
        return False


class JunctionLocalController:
    def __init__(self, node, waypoints, road_map, route_lengths):
        # 复用绘制绿色带的曲线与换道插值，只生成一次；本构造在规划线程运行。
        from modules.lincoln.visualization.navigation_route import sample_route
        self.vehicle, self.world = node.car, node.world
        self.route = waypoints
        self.route_lengths = route_lengths
        self.exit_classifier = ExitBranchClassifier(road_map)
        self.windows = self.find_junction_windows()
        self.window_index = 0
        self.active = False
        self.ignore_traffic_lights = False
        self.exit_stable_since = None
        self.progress = 0.
        self.last_time = None
        self.integral = self.previous_error = self.steer = 0.
        self.wait_reason = ''
        if not self.windows:
            return
        self.samples = sample_route(waypoints, road_map)
        self.distances = [0.]
        for first, second in zip(self.samples, self.samples[1:]):
            self.distances.append(self.distances[-1] + math.dist(first.center, second.center))
        self.obstacles = []
        self.next_obstacle_refresh = 0.
        self.front = self.vehicle.bounding_box.extent.x
        self.half_width = self.vehicle.bounding_box.extent.y
        physics = self.vehicle.get_physics_control()
        front = physics.wheels[:2]
        rear = physics.wheels[2:4]
        self.wheelbase = 0.
        if len(front) == len(rear) == 2:
            front_center = tuple(sum(getattr(w.position, axis) for w in front)/2 for axis in ('x', 'y'))
            rear_center = tuple(sum(getattr(w.position, axis) for w in rear)/2 for axis in ('x', 'y'))
            self.wheelbase = math.dist(front_center, rear_center)/100.
        if self.wheelbase < 1.:
            raise ValueError('Vehicle wheelbase is unavailable for junction local control.')
        self.max_steer = math.radians(max(w.max_steer_angle for w in front))
        self.steering_curve = sorted((point.x, point.y) for point in physics.steering_curve)
        self.signal_stops = self.find_signal_stops()

    def find_junction_windows(self):
        """只为选中的分流出口建立接管区间；直行主线和普通转弯仍由 TM 驾驶。"""
        windows = []
        for entry, exit_index in junction_spans(self.route):
            incoming = self.route[entry-1] if entry > 0 and not self.route[entry-1].is_junction else None
            if not self.exit_classifier.requires_exit_selection(self.route[entry], incoming):
                continue
            start = max(0., self.route_lengths[entry] - APPROACH_DISTANCE)
            for prior in range(entry-1, -1, -1):
                first, second = self.route[prior:prior+2]
                if first.is_junction:
                    break
                if first.road_id == second.road_id and first.lane_id != second.lane_id:
                    start = min(start, max(0., self.route_lengths[prior] - LANE_CHANGE_LEAD))
            windows.append(dict(start=start, entry=self.route_lengths[entry],
                                exit=self.route_lengths[exit_index], exit_index=exit_index))
        return windows

    def next_window(self):
        return self.windows[self.window_index] if self.window_index < len(self.windows) else None

    def should_take_over(self, progress, speed):
        window = self.next_window()
        return window is not None and (progress >= window['start'] or
                                       window['entry'] - progress <= max(APPROACH_DISTANCE, speed*3.))

    def is_preparing(self, progress):
        window = self.next_window()
        return window is not None and window['start'] - progress < 20.

    def activate(self, progress):
        self.active = True
        self.progress = progress
        self.last_time = None
        self.integral = self.previous_error = 0.
        self.steer = self.vehicle.get_control().steer
        self.exit_stable_since = None

    def can_return_to_tm(self, progress, transform, current, simulation_time):
        """正确出口车道连续稳定半秒后交还，近邻路口不反复切换。"""
        window = self.next_window()
        if window is None or self.wait_reason:
            return False
        if self.window_index+1 < len(self.windows):
            following = self.windows[self.window_index+1]
            if following['start'] <= window['exit'] + EXIT_DISTANCE + 10.:
                if progress >= following['start']:
                    self.window_index += 1
                return False
        expected_index = max(window['exit_index'], bisect.bisect_right(self.route_lengths, progress)-1)
        exit_point = self.route[min(expected_index, len(self.route)-1)]
        lane = lambda point: (point.road_id, point.section_id, point.lane_id)
        same_lane = current is not None and not current.is_junction and lane(current) == lane(exit_point)
        _, lateral_error = self.closest_sample(transform.location)
        stable = (progress >= window['exit'] + EXIT_DISTANCE and same_lane
                  and lateral_error < EXIT_LATERAL_ERROR
                  and heading_error(transform.rotation.yaw, current.transform.rotation.yaw) < EXIT_HEADING_ERROR)
        if not stable:
            self.exit_stable_since = None
            return False
        if self.exit_stable_since is None:
            self.exit_stable_since = simulation_time
        return simulation_time - self.exit_stable_since >= EXIT_STABLE_SECONDS

    def finish(self):
        self.active = False
        self.window_index += 1
        self.exit_stable_since = None

    def closest_sample(self, location):
        """只在当前里程附近找轨迹，保留高度，避免跨层或跳到交叉路段。"""
        begin = max(0, bisect.bisect_left(self.distances, self.progress-10.))
        end = min(len(self.samples), bisect.bisect_right(self.distances, self.progress+45.))
        index = min(range(begin, end), key=lambda i: math.dist(self.samples[i].center,
                                                               (location.x, location.y, location.z)))
        distance = math.dist(self.samples[index].center, (location.x, location.y, location.z))
        return index, distance

    def steering_scale(self, speed):
        for first, second in zip(self.steering_curve, self.steering_curve[1:]):
            if speed <= second[0]:
                fraction = max(0., min(1., (speed-first[0])/max(.001, second[0]-first[0])))
                return first[1] + fraction*(second[1]-first[1])
        return self.steering_curve[-1][1] if self.steering_curve else 1.

    def lateral_control(self, transform, speed, dt):
        """Pure Pursuit 使用后轴参考点，CARLA 中正转角表示向右。"""
        index, _ = self.closest_sample(transform.location)
        self.progress = max(self.progress, self.distances[index])
        lookahead = max(LOOKAHEAD_MIN, min(LOOKAHEAD_MAX, LOOKAHEAD_MIN + speed*LOOKAHEAD_SECONDS))
        target_index = min(len(self.samples)-1, bisect.bisect_left(self.distances, self.progress+lookahead))
        target = self.samples[target_index].center
        yaw = math.radians(transform.rotation.yaw)
        rear_x = transform.location.x - self.wheelbase*.5*math.cos(yaw)
        rear_y = transform.location.y - self.wheelbase*.5*math.sin(yaw)
        dx, dy = target[0]-rear_x, target[1]-rear_y
        alpha = math.atan2(dy, dx) - yaw
        angle = math.atan2(2.*self.wheelbase*math.sin(alpha), max(.5, math.hypot(dx,dy)))
        desired = max(-1., min(1., angle/max(.05, self.max_steer*self.steering_scale(speed))))
        self.steer += max(-STEER_RATE*dt, min(STEER_RATE*dt, desired-self.steer))
        return self.steer

    def find_signal_stops(self):
        """在规划线程把红绿灯停止位置匹配到当前轨迹，不读取或修改相位。"""
        stops = []
        for light in self.world.get_actors().filter('traffic.traffic_light*'):
            for waypoint in light.get_stop_waypoints():
                location = waypoint.transform.location
                index = min(range(len(self.samples)), key=lambda i: math.dist(
                    self.samples[i].center, (location.x, location.y, location.z)))
                sample = self.samples[index]
                forward = waypoint.transform.get_forward_vector()
                aligned = sample.forward[0]*forward.x + sample.forward[1]*forward.y > .5
                if aligned and math.dist(sample.center, (location.x,location.y,location.z)) < 1.8:
                    stops.append((self.distances[index], light))
        return stops

    def signal_speed_limit(self, speed_limit):
        if self.ignore_traffic_lights:
            return speed_limit
        for station, light in self.signal_stops:
            gap = station - self.progress - self.front - .5
            # 前轴已经通过停止位置时，不在路口中间响应身后的相位变化。
            if gap < -1. or gap > 50.:
                continue
            if light.get_state() in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow):
                limit = math.sqrt(2.*BRAKING_DECELERATION*max(0., gap-1.))
                if limit < speed_limit:
                    speed_limit = limit
                    self.wait_reason = 'traffic_light'
        return speed_limit

    def curve_speed_limit(self, speed_limit, speed):
        """根据前方实际曲率进一步降速，让急弯和短换道有足够的转向空间。"""
        begin = bisect.bisect_left(self.distances, self.progress)
        end = min(len(self.samples)-1, bisect.bisect_right(self.distances,
                                                         self.progress+max(15., speed*3.)))
        for index in range(begin, end):
            first, second = self.samples[index:index+2]
            yaw_a = math.atan2(first.forward[1],first.forward[0])
            yaw_b = math.atan2(second.forward[1],second.forward[0])
            angle = abs((yaw_b-yaw_a+math.pi) % (2.*math.pi)-math.pi)
            curvature = angle/max(.1, self.distances[index+1]-self.distances[index])
            if curvature > .001:
                speed_limit = min(speed_limit, math.sqrt(MAX_LATERAL_ACCELERATION/curvature))
        return speed_limit

    def obstacle_speed_limit(self, snapshot, speed_limit, speed, now):
        """对当前轨迹上的车辆、行人、实体设施减速等待，不绕行、不新增传感器。"""
        if now >= self.next_obstacle_refresh:
            self.obstacles = [actor for actor in self.world.get_actors()
                              if actor.id != self.vehicle.id and
                              actor.type_id.startswith(('vehicle.', 'walker.', 'static.prop.'))]
            self.next_obstacle_refresh = now + OBSTACLE_REFRESH
        begin = bisect.bisect_left(self.distances, self.progress)
        horizon = max(12., speed*2. + speed*speed/(2.*BRAKING_DECELERATION))
        end = bisect.bisect_right(self.distances, self.progress+horizon)
        for actor in self.obstacles:
            state = snapshot.find(actor.id)
            if state is None:
                continue
            transform, box = state.get_transform(), actor.bounding_box
            center = transform.transform(carla.Location(x=box.location.x,y=box.location.y,z=box.location.z))
            if math.hypot(center.x-self.samples[begin].center[0], center.y-self.samples[begin].center[1]) > horizon + 10.:
                continue
            angle = math.radians(transform.rotation.yaw + box.rotation.yaw)
            cosine, sine = math.cos(angle), math.sin(angle)
            for index in range(begin, min(end, len(self.samples))):
                point = self.samples[index].center
                dx, dy = point[0]-center.x, point[1]-center.y
                longitudinal, lateral = dx*cosine + dy*sine, -dx*sine + dy*cosine
                if (abs(longitudinal) < box.extent.x + self.front + OBSTACLE_MARGIN
                        and abs(lateral) < box.extent.y + self.half_width + OBSTACLE_MARGIN
                        and abs(point[2]-center.z) < box.extent.z + 1.):
                    gap = max(0., self.distances[index]-self.progress-2.)
                    limit = math.sqrt(2.*BRAKING_DECELERATION*gap)
                    if limit < speed_limit:
                        speed_limit = limit
                        self.wait_reason = 'obstacle'
                    break
        return speed_limit

    def longitudinal_control(self, speed, target_speed, dt):
        if target_speed < .15:
            self.integral = self.previous_error = 0.
            return 0., 1.
        error = target_speed - speed
        # 原来的积分上限只能提供很小的稳态油门，车辆会长期达不到目标速度。
        candidate = max(-SPEED_INTEGRAL_LIMIT, min(SPEED_INTEGRAL_LIMIT, self.integral + error*dt))
        derivative = (error-self.previous_error)/dt
        self.previous_error = error
        acceleration = SPEED_KP*error + SPEED_KI*candidate + SPEED_KD*derivative
        # 油门或刹车已饱和时，不继续积累同方向误差，避免恢复后突然加速。
        if not ((acceleration > 3.*MAX_THROTTLE and error > 0.) or (acceleration < -4. and error < 0.)):
            self.integral = candidate
        else:
            acceleration = SPEED_KP*error + SPEED_KI*self.integral + SPEED_KD*derivative
        return max(0., min(MAX_THROTTLE, acceleration/3.)), max(0., min(1., -acceleration/4.))

    def run_step(self, snapshot, speed_limit_kmh):
        state = snapshot.find(self.vehicle.id)
        if state is None:
            raise RuntimeError('Vehicle snapshot unavailable for junction local control.')
        now = snapshot.timestamp.elapsed_seconds
        dt = .05 if self.last_time is None else max(.01, min(.2, now-self.last_time))
        self.last_time = now
        transform, speed = state.get_transform(), state.get_velocity().length()
        steer = self.lateral_control(transform, speed, dt)
        self.wait_reason = ''
        target = self.curve_speed_limit(speed_limit_kmh/3.6, speed)
        target = self.signal_speed_limit(target)
        target = self.obstacle_speed_limit(snapshot, target, speed, now)
        throttle, brake = self.longitudinal_control(speed, target, dt)
        return carla.VehicleControl(throttle=throttle,brake=brake,steer=steer)
