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
from modules.lincoln.driving.junction_local_controller import JunctionLocalController, junction_spans
from modules.lincoln.driving.controller import ROAD_SPEED_LIMIT_FACTOR

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
TM_EXIT_GUIDE_DISTANCE = 10.0  # 出口引导避开路口边缘重合的内部连接道路。
TM_TURN_GUIDE_DISTANCE = 60.0
TM_TURN_STRAIGHT_ANGLE = 35.0
TM_LANE_CHANGE_MARGIN = 15.0
NAVIGATION_SPEED_KMH = 120.0
LANE_CHANGE_SPEED_KMH = 80.0
TURN_SPEED_KMH = 80.0
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
    points = []
    outside_distance = None
    previous = None
    for index, point in enumerate(waypoints):
        if point.is_junction:
            outside_distance = 0.
            previous = point
            continue
        if outside_distance is not None and previous is not None and not previous.is_junction:
            outside_distance += previous.transform.location.distance(point.transform.location)
        next_is_junction = index+1 < len(waypoints) and waypoints[index+1].is_junction
        # 很短的出口道路仍保留末端锚点，不悄悄跳过整条道路。
        if (outside_distance is None or outside_distance >= TM_EXIT_GUIDE_DISTANCE
                or next_is_junction or index == len(waypoints)-1):
            points.append(point.transform.location)
        previous = point
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
        self.gta_request = None
        self.gta_status = dict(enabled=False, request_id=None, error='')
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
        self.local_controller = None
        self.tm_turns = []
        self.tm_turn_index = 0
        self.tm_turn_active = False

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def gta_snapshot(self):
        """登记线程只读取已执行状态，不把待执行指令当作成功。"""
        with self.lock:
            return dict(self.gta_status)

    def submit_gta_mode(self, request):
        """登记线程提交开关；TM 调用留在唯一驾驶线程。"""
        if (not isinstance(request, dict) or not isinstance(request.get('id'), str)
                or type(request.get('enabled')) is not bool):
            return
        with self.lock:
            if request['id'] != self.gta_status['request_id']:
                self.gta_request = dict(request)

    def apply_gta_mode(self):
        """无论是否正在导航都处理开关；失败回报旧状态，不中断驾驶。"""
        with self.lock:
            request, self.gta_request = self.gta_request, None
        if request is None:
            return
        error = ''
        enabled = self.gta_status['enabled']
        try:
            self.node.driver.set_gta_mode(request['enabled'])
            enabled = request['enabled']
        except Exception:
            logging.exception('GTA Mode 切换失败')
            error = 'GTA Mode could not be applied; check Lincoln logs.'
        with self.lock:
            self.gta_status = dict(enabled=enabled, request_id=request['id'], error=error)

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
                lengths = [0.]
                for first, second in zip(route, route[1:]):
                    lengths.append(lengths[-1] + first.transform.location.distance(second.transform.location))
                local = None
                if (getattr(self.node.driver, 'supports_junction_local_control', False) is True
                        and any(point.is_junction for point in route)):
                    local = JunctionLocalController(self.node, route, self.road_map, lengths)
                    if not local.windows:
                        local = None
                outcome = (path_with_lookahead(route), goal, route, '', local)
            except PlanningCancelled:
                continue
            except Exception as error:
                outcome = (None, None, [], str(error), None)
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
        self.local_controller = None
        self.tm_turns = []
        self.tm_turn_active = False
        with self.lock:
            self.status['controller'] = 'manual'
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
            identity, (route, goal, display_waypoints, error, local) = result
            if not self.is_current(identity):
                return
            if self.node.active:
                self.cancel()
                return
            if error:
                self.status.update(state='unreachable', detail=error)
                return
            try:
                if local is None or not local.should_take_over(0., 0.):
                    self.node.driver.set_route(route)
                self.node.autopilot = True
                self.node.active = None
                self.node.cruise_speed = None
                self.node.reverse = False
                self.goal = goal
                self.prepare_route_tracking(display_waypoints)
                self.local_controller = local
                self.prepare_tm_turns()
                self.hold_brake = False
                self.status.update(state='navigating', detail='Navigating to destination', controller='tm')
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

    def prepare_tm_turns(self):
        """只给未接管的直行连接提供 TM 引导；转向与分支选择留给局部控制。"""
        self.tm_turns = []
        self.tm_turn_index = 0
        self.tm_turn_active = False
        if getattr(self.node.driver, 'supports_junction_local_control', False) is not True:
            return
        local_entries = {window['entry'] for window in self.local_controller.windows} if self.local_controller else set()
        for entry, exit_index in junction_spans(self.route):
            if self.route_lengths[entry] in local_entries:
                continue
            if self.route[exit_index].is_junction:
                continue
            before = self.route[max(0,entry-1)].transform.rotation.yaw
            after = self.route[exit_index].transform.rotation.yaw
            angle = (after-before+180.) % 360. - 180.
            action = 'Straight' if abs(angle) < TM_TURN_STRAIGHT_ANGLE else ('Right' if angle > 0. else 'Left')
            self.tm_turns.append(dict(entry=self.route_lengths[entry],exit_index=exit_index,action=action))

    def update_tm_turn_guidance(self, index, current):
        """TM 始终持有驾驶权，仅在普通路口前后更新其原生行驶动作。"""
        if self.tm_turn_index >= len(self.tm_turns):
            return
        turn = self.tm_turns[self.tm_turn_index]
        if self.tm_turn_active:
            if index >= turn['exit_index'] and current is not None and not current.is_junction:
                remaining = self.route[min(index+1,len(self.route)-1):]
                self.node.driver.set_turn_guidance([], path_with_lookahead(remaining))
                self.tm_turn_index += 1
                self.tm_turn_active = False
            return
        if self.route_progress >= self.route_lengths[turn['exit_index']]:
            self.tm_turn_index += 1
            return
        if turn['entry']-self.route_progress <= TM_TURN_GUIDE_DISTANCE:
            actions = [item['action'] for item in self.tm_turns[self.tm_turn_index:]]
            self.node.driver.set_turn_guidance(actions)
            self.tm_turn_active = True

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
            limit = min(limit, road_limit * ROAD_SPEED_LIMIT_FACTOR)
        # 预留半秒控制响应距离及 2.4 m 的终点容差，不在减速中重新启用 TM。
        stopping_space = max(0., remaining - ARRIVAL_DISTANCE*.6 - speed*.5)
        return min(limit, math.sqrt(2.*SLOWDOWN_DECELERATION*stopping_space)*3.6)

    def tm_lane_change_deviation(self, location, current, deviation):
        """TM 可在计划换道附近调整时机；允许源/目标车道，不放过选错道路。"""
        if current is None or (self.local_controller is not None and self.local_controller.active):
            return deviation
        for first, second in zip(self.route, self.route[1:]):
            same_road = (first.road_id,first.section_id) == (second.road_id,second.section_id)
            if not same_road or first.lane_id == second.lane_id:
                continue
            if (current.road_id,current.section_id) != (first.road_id,first.section_id):
                continue
            if current.lane_id not in (first.lane_id,second.lane_id):
                continue
            if min(first.s,second.s)-TM_LANE_CHANGE_MARGIN <= current.s <= max(first.s,second.s)+TM_LANE_CHANGE_MARGIN:
                return min(deviation, location.distance(current.transform.location))
        return deviation

    def begin_stopping(self, reason):
        """停止阶段只进入一次，保持制动，直到停车或收到显式新指令。"""
        if self.stopping is None:
            self.stopping = reason
            self.node.driver.disable()
            self.node.driver.clear_route()
            with self.lock:
                self.status['detail'] = 'Stopping at destination' if reason == 'destination' else reason
                self.status['controller'] = 'stopping'
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

    def set_controller_status(self, mode, wait_reason=''):
        """面板直接读本车状态，桥和 SUMO 继续使用原有导航状态格式。"""
        with self.lock:
            if self.status.get('controller') != mode:
                logging.info('导航控制权：%s → %s，任务 %s', self.status.get('controller'), mode, self.status.get('id'))
            self.status.update(controller=mode, control_detail=wait_reason)

    def update_junction_control(self, snapshot, transform, current, index, speed_limit):
        """驾驶线程内切换唯一控制者；恢复 TM 时只上传当前位置之后的路线。"""
        local = self.local_controller
        if local is None:
            return None
        actor = snapshot.find(self.node.car.id)
        if not local.active and local.should_take_over(self.route_progress, actor.get_velocity().length()):
            self.node.driver.disable()
            self.node.driver.clear_route()
            local.activate(self.route_progress)
            self.set_controller_status('local')
            view = getattr(self.node, 'navigation_view', None)
            if view is not None:
                view.show_tm_path(transform.location, [])
        if local.active:
            local.ignore_traffic_lights = self.gta_status['enabled']
            control = local.run_step(snapshot, speed_limit)
            self.set_controller_status('local', local.wait_reason)
            if local.can_return_to_tm(self.route_progress, transform, current, snapshot.timestamp.elapsed_seconds):
                remaining = self.route[min(index+1, len(self.route)-1):]
                self.node.driver.set_navigation_speed(speed_limit)
                self.node.driver.set_route(path_with_lookahead(remaining))
                local.finish()
                self.set_controller_status('tm')
                return None
            return control
        self.set_controller_status('preparing' if local.is_preparing(self.route_progress) else 'tm')
        return None

    def update(self, snapshot):
        """只由现有驾驶线程调用：路线监测、提前减速、单向进入终点停车。"""
        self.apply_gta_mode()
        with self.lock:
            reset, self.reset_requested = self.reset_requested, False
        if reset:
            self.node.driver.disable()
            self.node.driver.clear_route()
            self.node.autopilot = False
            self.goal = None
            self.stopping = None
            self.local_controller = None
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
        deviation = self.tm_lane_change_deviation(transform.location, current, deviation)
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
        speed_limit = self.planned_speed(index, remaining, speed)
        try:
            control = self.update_junction_control(snapshot, transform, current, index, speed_limit)
            if control is None:
                self.update_tm_turn_guidance(index, current)
        except Exception as error:
            logging.exception('导航控制、TM 路口引导或交接失败')
            self.begin_stopping('Navigation control failed: ' + str(error))
            return self.update_stopping(speed, distance, matching_lane)
        if control is not None:
            return control
        self.node.driver.set_navigation_speed(speed_limit)
        return None

    def close(self):
        self.stopped.set()
        self.cancel('Vehicle exited')
        if self.worker is not None:
            self.worker.join(timeout=2.)
