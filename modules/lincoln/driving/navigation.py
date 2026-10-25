"""Lincoln 导航：后台沿 CARLA 有向车道规划，原有驾驶线程执行与停车。

任务来自既有桥登记连接；不连接 SUMO，不推进仿真，不修改地图。
手动接管会取消任务，过期规划结果不会重新抢回驾驶权。
"""
import heapq
import itertools
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
ARRIVAL_DISTANCE = 4.0
BRAKING_DISTANCE = 18.0
STOP_SPEED = 0.3
TARGET_SNAP_LIMIT = 8.0
TM_LOOKAHEAD_EXTENSION = 200.0


class PlanningCancelled(Exception):
    pass


def waypoint_key(waypoint):
    return waypoint.road_id, waypoint.section_id, waypoint.lane_id, int(waypoint.s / ROUTE_SPACING)


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
    """为 TM 保留目标后的前瞻点，实际车辆由到达逻辑在目标处停车。"""
    points = [waypoint.transform.location for waypoint in route]
    waypoint = route[-1]
    for _ in range(int(TM_LOOKAHEAD_EXTENSION / ROUTE_SPACING)):
        following = waypoint.next(ROUTE_SPACING)
        if not following:
            break
        waypoint = following[0]
        points.append(waypoint.transform.location)
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
                outcome = (path_with_lookahead(route), goal, '')
            except PlanningCancelled:
                continue
            except Exception as error:
                outcome = (None, None, str(error))
            with self.lock:
                if self.status.get('id') == identity and self.status['state'] == 'planning':
                    self.result = (identity, outcome)

    def cancel(self, detail='Navigation cancelled by manual control'):
        with self.lock:
            active = self.status['state'] in ('planning', 'navigating', 'arrived')
            if active:
                self.status.update(state='cancelled', detail=detail)
            self.result = None
            self.reset_requested = False
        self.goal = None
        self.hold_brake = False
        if active:
            self.node.driver.clear_route()

    def consume_plan(self):
        with self.lock:
            result, self.result = self.result, None
            if result is None:
                return
            identity, (route, goal, error) = result
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
                self.hold_brake = False
                self.status.update(state='navigating', detail='Navigating to destination')
            except Exception as error:
                self.node.driver.disable()
                self.node.autopilot = False
                self.status.update(state='error', detail=str(error))

    def update(self, snapshot):
        """只由现有驾驶线程调用：应用规划结果、接近目标减速、到达停车。"""
        with self.lock:
            reset, self.reset_requested = self.reset_requested, False
        if reset:
            self.node.driver.disable()
            self.node.driver.clear_route()
            self.node.autopilot = False
            self.goal = None
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
        same_lane = matching_lane and distance <= ARRIVAL_DISTANCE and abs(transform.location.z - self.goal.transform.location.z) < 2.
        if same_lane and speed < STOP_SPEED:
            self.node.driver.disable()
            self.node.autopilot = False
            self.hold_brake = True
            self.goal = None
            with self.lock:
                self.status.update(state='arrived', detail='Destination reached')
            return carla.VehicleControl(brake=1.)
        if matching_lane and distance < BRAKING_DISTANCE:
            # 近目标时制动指令由现有控制出口发送，不再让 TM 同时写控制。
            desired = max(0., (distance - ARRIVAL_DISTANCE * .6) * .5)
            if speed > desired or distance < ARRIVAL_DISTANCE:
                self.node.driver.disable()
                return carla.VehicleControl(brake=min(1., max(.2, (speed - desired) * .3)))
            self.node.driver.enable()
        return None

    def close(self):
        self.stopped.set()
        self.cancel('Vehicle exited')
        if self.worker is not None:
            self.worker.join(timeout=2.)
