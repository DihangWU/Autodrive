"""Lincoln 导航：后台沿 CARLA 有向车道规划，原有驾驶线程执行与停车。

任务来自既有桥登记连接；不连接 SUMO，不推进仿真，不修改地图。
手动接管会取消任务，过期规划结果不会重新抢回驾驶权。
"""
import heapq
import itertools
import logging
import math
import queue
import threading
import time

import carla

ROUTE_SPACING = 4.0
MAX_SEARCH_NODES = 60000
PLANNING_TIMEOUT = 12.0
LANE_CHANGE_DISTANCE = 12.0
LANE_CHANGE_PENALTY = 8.0
LANE_CHANGE_JUNCTION_MARGIN = 12.0  # 完成换道后，留出进入路口前的稳定距离。
ARRIVAL_DISTANCE = 4.0
BRAKING_DISTANCE = 18.0
STOP_SPEED = 0.3
TARGET_SNAP_LIMIT = 8.0
TM_LOOKAHEAD_EXTENSION = 200.0
NAVIGATION_SPEED_KMH = 40.0
LANE_CHANGE_SPEED_KMH = 20.0
TURN_SPEED_KMH = 25.0
SLOWDOWN_DECELERATION = 2.0  # 用于提前减速的规划减速度，m/s²。
DEVIATION_SECONDS = 0.8  # 仿真秒；不受现实时间倍率或暂停影响。
PREVIEW_INTERVAL = 0.2


def project_on_segment(location, first, second):
    """将位置投影到有高度的路线段，返回段内比例和三维距离。"""
    delta = second - first
    length_squared = delta.x**2 + delta.y**2 + delta.z**2
    offset = location - first
    fraction = max(0., min(1., (offset.x*delta.x + offset.y*delta.y + offset.z*delta.z)
                          / length_squared)) if length_squared > 1e-8 else 0.
    projected = carla.Location(x=first.x + fraction*delta.x, y=first.y + fraction*delta.y,
                               z=first.z + fraction*delta.z)
    return fraction, location.distance(projected)


class PlanningCancelled(Exception):
    pass


def waypoint_key(waypoint):
    return waypoint.road_id, waypoint.section_id, waypoint.lane_id, int(waypoint.s / ROUTE_SPACING)


def lane_change_finishes_before_junction(adjacent, following):
    """换道只能在目标车道内完成，不能用一次连接跳进路口或省略入口车道。"""
    lane = (adjacent.road_id, adjacent.section_id, adjacent.lane_id)
    def same_lane_outside_junction(point):
        return not point.is_junction and (point.road_id, point.section_id, point.lane_id) == lane
    if not same_lane_outside_junction(following):
        return False
    clearance = following.next(LANE_CHANGE_JUNCTION_MARGIN)
    return bool(clearance) and all(same_lane_outside_junction(point) for point in clearance)


def route_successors(waypoint):
    """顺道路行进，仅在允许换道且同向的车道之间建立平滑前向连接。"""
    for following in waypoint.next(ROUTE_SPACING):
        yield following, 0.
    if waypoint.is_junction:
        return
    for flag, get_lane in ((carla.LaneChange.Left, waypoint.get_left_lane),
                           (carla.LaneChange.Right, waypoint.get_right_lane)):
        if not int(waypoint.lane_change) & int(flag):
            continue
        adjacent = get_lane()
        if adjacent is None or adjacent.lane_type != carla.LaneType.Driving:
            continue
        first, second = waypoint.transform.get_forward_vector(), adjacent.transform.get_forward_vector()
        if first.x * second.x + first.y * second.y < .5:
            continue
        for following in adjacent.next(LANE_CHANGE_DISTANCE):
            if lane_change_finishes_before_junction(adjacent, following):
                yield following, LANE_CHANGE_PENALTY


def plan_route(road_map, origin, target, cancelled=lambda: False):
    """A* 搜索有向可行驶车道，不跨越隔离带或伪造地图连接。"""
    start = road_map.get_waypoint(origin, project_to_road=True, lane_type=carla.LaneType.Driving)
    goal = road_map.get_waypoint(target, project_to_road=True, lane_type=carla.LaneType.Driving)
    if start is None or goal is None or goal.transform.location.distance(target) > TARGET_SNAP_LIMIT:
        raise ValueError('No matching CARLA driving lane at the target position and height.')
    goal_lane = (goal.road_id, goal.section_id, goal.lane_id)
    counter = itertools.count()
    pending = [(0., next(counter), 0., start)]
    costs, parents, points = {waypoint_key(start): 0.}, {}, {waypoint_key(start): start}
    deadline = time.monotonic() + PLANNING_TIMEOUT
    while pending:
        if cancelled():
            raise PlanningCancelled()
        if len(costs) > MAX_SEARCH_NODES or time.monotonic() > deadline:
            raise ValueError('Route search limit reached; select a closer reachable road.')
        _, _, cost, current = heapq.heappop(pending)
        key = waypoint_key(current)
        if cost > costs[key]:
            continue
        location = current.transform.location
        if (current.road_id, current.section_id, current.lane_id) == goal_lane and location.distance(goal.transform.location) <= ROUTE_SPACING * 1.5:
            route = [goal]
            while key in parents:
                route.append(points[key])
                key = parents[key]
            route.append(start)
            return list(reversed(route)), goal
        for following, penalty in route_successors(current):
            candidate = waypoint_key(following)
            new_cost = cost + location.distance(following.transform.location) + penalty
            if new_cost >= costs.get(candidate, math.inf):
                continue
            costs[candidate], parents[candidate], points[candidate] = new_cost, key, following
            heuristic = following.transform.location.distance(goal.transform.location)
            heapq.heappush(pending, (new_cost + heuristic, next(counter), new_cost, following))
    raise ValueError('No legal road or lane-change route reaches the destination.')


def path_with_lookahead(route):
    """给 TM 明确的路口出口道路；完整的路口曲线仍留给绿色路线及里程监测。"""
    waypoints = list(route)
    waypoint = route[-1]
    for _ in range(int(TM_LOOKAHEAD_EXTENSION / ROUTE_SPACING)):
        following = waypoint.next(ROUTE_SPACING)
        if not following:
            break
        waypoint = following[0]
        waypoints.append(waypoint)
    # TM ImportPath 按路口后的 road_id 选分支。内部连接路点的 road_id
    # 与出口不同，且多个连接在入口可能重合，直接上传会使 TM 选错出口。
    # 跳过内部连接点，让下一处引导点落到已规划的出口道路上；不改绿色路线。
    points = [point.transform.location for point in waypoints if not point.is_junction]
    if not points:
        raise ValueError('No junction exit guide is available for Traffic Manager.')
    return points


class LincolnNavigation:
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.tasks = queue.Queue(maxsize=1)
        self.stopped = threading.Event()
        self.worker = None
        self.last_received = None
        self.status = dict(state='idle', detail='No destination')
        self.result = None
        self.goal = None
        self.road_map = None
        self.reset_requested = False
        self.hold_brake = False
        self.route = []
        self.route_lengths = []
        self.route_progress = 0.
        self.stopping = None
        self.deviation_since = None
        self.next_preview = 0.

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def submit(self, task):
        """登记线程仅入队；重复轮询同一任务不会重复启动导航。"""
        if not isinstance(task, dict) or not isinstance(task.get('id'), str) or not isinstance(task.get('target'), dict):
            return
        with self.lock:
            if task['id'] == self.last_received:
                return
            self.last_received = task['id']
            if getattr(self.node, 'last_manual_control', 0.) > task.get('issued_at', time.monotonic()):
                self.status = dict(id=task['id'], state='cancelled', detail='Manual control superseded this navigation request')
                return
            self.status = dict(id=task['id'], state='planning', detail='Planning CARLA route')
            self.result = None
            self.reset_requested = True
        try:
            self.tasks.get_nowait()
        except queue.Empty:
            pass
        view = getattr(self.node, 'navigation_view', None)
        if view is not None:
            view.clear_route()
        self.tasks.put_nowait(task)
        if self.worker is None:
            self.worker = threading.Thread(target=self.plan_tasks, name='lincoln-navigation', daemon=True)
            self.worker.start()

    def is_current(self, identity):
        with self.lock:
            return self.status.get('id') == identity and self.status['state'] == 'planning' and not self.stopped.is_set()

    def plan_tasks(self):
        while not self.stopped.is_set():
            try:
                task = self.tasks.get(timeout=.2)
            except queue.Empty:
                continue
            identity = task['id']
            try:
                if not self.is_current(identity):
                    continue
                if self.road_map is None:
                    self.road_map = self.node.world.get_map()
                target = carla.Location(**{key: float(task['target'][key]) for key in ('x', 'y', 'z')})
                route, goal = plan_route(self.road_map, self.node.car.get_location(), target,
                                         lambda: not self.is_current(identity))
                outcome = (path_with_lookahead(route), goal, route, '')
            except PlanningCancelled:
                continue
            except Exception as error:
                outcome = (None, None, [], str(error))
            with self.lock:
                if self.status.get('id') == identity and self.status['state'] == 'planning':
                    self.result = (identity, outcome)

    def cancel(self, detail='Navigation cancelled by manual control'):
        with self.lock:
            active = self.status['state'] in ('planning', 'navigating', 'arrived') or bool(self.route)
            if active:
                self.status.update(state='cancelled', detail=detail)
            self.result = None
            self.reset_requested = False
        self.goal = None
        self.hold_brake = False
        self.stopping = None
        self.route = []
        self.deviation_since = None
        if active:
            self.node.driver.clear_route()
            view = getattr(self.node, 'navigation_view', None)
            if view is not None:
                view.clear_route()

    def consume_plan(self):
        with self.lock:
            result, self.result = self.result, None
            if result is None:
                return
            identity, (route, goal, display_waypoints, error) = result
            if not self.is_current(identity):
                return
            if self.node.active:
                self.cancel()
                return
            if error:
                self.status.update(state='unreachable', detail=error)
                return
            try:
                self.node.driver.set_route(route)
                self.node.autopilot = True
                self.node.active = None
                self.node.cruise_speed = None
                self.node.reverse = False
                self.goal = goal
                self.prepare_route_tracking(display_waypoints)
                self.hold_brake = False
                self.status.update(state='navigating', detail='Navigating to destination')
            except Exception as error:
                self.node.driver.disable()
                self.node.autopilot = False
                self.status.update(state='error', detail=str(error))
                return
            view = getattr(self.node, 'navigation_view', None)
            if view is not None:
                try:
                    view.show_route(display_waypoints, self.road_map)
                except Exception:
                    logging.exception('导航可视化提交失败；车辆导航继续运行')

    def prepare_route_tracking(self, waypoints):
        """累计路线里程；只沿车辆附近的路线段推进，避免跳到交叉处的终点。"""
        self.route = list(waypoints)
        self.route_lengths = [0.]
        for first, second in zip(self.route, self.route[1:]):
            self.route_lengths.append(self.route_lengths[-1] +
                                      first.transform.location.distance(second.transform.location))
        self.route_progress = 0.
        self.stopping = None
        self.deviation_since = None
        self.next_preview = 0.

    def track_route(self, location, speed):
        """返回剩余里程、距路线的距离及当前段，不因已经越过终点而增加剩余距离。"""
        if len(self.route) < 2:
            return location.distance(self.goal.transform.location), 0., 0
        candidates = []
        for index, (first, second) in enumerate(zip(self.route, self.route[1:])):
            if self.route_lengths[index+1] < self.route_progress - 8.:
                continue
            if self.route_lengths[index] > self.route_progress + max(50., speed*3.):
                break
            fraction, distance = project_on_segment(location, first.transform.location, second.transform.location)
            progress = self.route_lengths[index] + fraction*(self.route_lengths[index+1]-self.route_lengths[index])
            candidates.append((distance, index, progress))
        distance, index, progress = min(candidates)
        self.route_progress = max(self.route_progress, progress)
        return self.route_lengths[-1] - self.route_progress, distance, index

    def planned_speed(self, index, remaining, speed):
        """在换道/转弯之前限速；终点速度按剩余里程持续降到零。"""
        limit = NAVIGATION_SPEED_KMH
        preview_distance = max(25., speed*3.)
        for i in range(index, max(index, len(self.route)-1)):
            if self.route_lengths[i] > self.route_progress + preview_distance:
                break
            first, second = self.route[i:i+2]
            if first.road_id == second.road_id and first.lane_id != second.lane_id:
                limit = min(limit, LANE_CHANGE_SPEED_KMH)
            turn = abs((second.transform.rotation.yaw - self.route[index].transform.rotation.yaw + 180.) % 360. - 180.)
            if turn > 20. or first.is_junction:
                limit = min(limit, TURN_SPEED_KMH)
        road_limit = self.node.car.get_speed_limit()
        if road_limit > 0.:
            limit = min(limit, road_limit)
        # 预留半秒控制响应距离及 2.4 m 的终点容差，不在减速中重新启用 TM。
        stopping_space = max(0., remaining - ARRIVAL_DISTANCE*.6 - speed*.5)
        return min(limit, math.sqrt(2.*SLOWDOWN_DECELERATION*stopping_space)*3.6)

    def begin_stopping(self, reason):
        """停止阶段只进入一次，保持制动，直到停车或收到显式新指令。"""
        if self.stopping is None:
            self.stopping = reason
            self.node.driver.disable()
            self.node.driver.clear_route()
            with self.lock:
                self.status['detail'] = 'Stopping at destination' if reason == 'destination' else reason
            logging.info('导航停车：%s，任务 %s', reason, self.status.get('id'))

    def update_stopping(self, speed, distance, matching_lane):
        if speed < STOP_SPEED:
            arrived = self.stopping == 'destination' and matching_lane and distance <= ARRIVAL_DISTANCE
            self.node.autopilot = False
            self.hold_brake = True
            self.goal = None
            with self.lock:
                self.status.update(state='arrived' if arrived else 'error',
                                   detail='Destination reached' if arrived else self.stopping + '; vehicle stopped')
        return carla.VehicleControl(brake=1.)

    def update_preview(self, now, location):
        """低频读取本车 TM 动作点，不用规划路线伪造 TM 的前瞻。"""
        if now < self.next_preview:
            return
        self.next_preview = now + PREVIEW_INTERVAL
        view = getattr(self.node, 'navigation_view', None)
        if view is not None:
            try:
                view.show_tm_path(location, self.node.driver.navigation_preview())
            except Exception:
                logging.exception('读取 TM 导航前瞻失败')

    def update(self, snapshot):
        """只由现有驾驶线程调用：路线监测、提前减速、单向进入终点停车。"""
        with self.lock:
            reset, self.reset_requested = self.reset_requested, False
        if reset:
            self.node.driver.disable()
            self.node.driver.clear_route()
            self.node.autopilot = False
            self.goal = None
            self.stopping = None
            self.hold_brake = True
        self.consume_plan()
        if self.goal is None or not self.node.autopilot:
            return None
        actor = snapshot.find(self.node.car.id)
        if actor is None:
            return None
        transform = actor.get_transform()
        distance = transform.location.distance(self.goal.transform.location)
        velocity = actor.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        current = self.road_map.get_waypoint(transform.location, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
        matching_lane = current is not None and (current.road_id, current.section_id, current.lane_id) == (
            self.goal.road_id, self.goal.section_id, self.goal.lane_id)
        remaining, deviation, index = self.track_route(transform.location, speed)
        now = time.monotonic()
        self.update_preview(now, transform.location)
        simulation_time = snapshot.timestamp.elapsed_seconds
        if self.stopping is None:
            width = getattr(self.route[index], 'lane_width', 3.5) if self.route else 3.5
            if deviation > max(2.5, width*.75):
                if self.deviation_since is None:
                    self.deviation_since = simulation_time
                if simulation_time - self.deviation_since >= DEVIATION_SECONDS:
                    self.begin_stopping('Route deviation')
            else:
                self.deviation_since = None
            near_end = remaining <= BRAKING_DISTANCE
            if near_end and matching_lane and distance <= ARRIVAL_DISTANCE:
                self.begin_stopping('destination')
            elif near_end and remaining <= ARRIVAL_DISTANCE and not matching_lane:
                self.begin_stopping('Destination lane missed')
            elif near_end and self.route:
                delta = transform.location - self.goal.transform.location
                forward = self.goal.transform.get_forward_vector()
                passed = delta.x*forward.x + delta.y*forward.y + delta.z*forward.z > 0.
                if passed:
                    self.begin_stopping('Destination passed')
        if self.stopping is not None:
            return self.update_stopping(speed, distance, matching_lane)
        self.node.driver.set_navigation_speed(self.planned_speed(index, remaining, speed))
        return None

    def close(self):
        self.stopped.set()
        self.cancel('Vehicle exited')
        if self.worker is not None:
            self.worker.join(timeout=2.)
