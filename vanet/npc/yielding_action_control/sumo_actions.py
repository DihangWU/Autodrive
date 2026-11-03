"""SUMO 车辆避让动作；保留安全模式，恢复时平稳交还正常驾驶。"""


class YieldingActions:
    def __init__(self, vehicle_id, config):
        self.vehicle_id, self.config = vehicle_id, config
        self.saved = None
        self.action = None
        self.restore_started = None

    def remember(self, connection):
        if self.saved is None:
            vehicle = connection.vehicle
            self.saved = dict(speed_mode=vehicle.getSpeedMode(self.vehicle_id),
                              lane_change_mode=vehicle.getLaneChangeMode(self.vehicle_id),
                              max_speed=vehicle.getMaxSpeed(self.vehicle_id))

    def apply(self, connection, state, assessment):
        vehicle, name = connection.vehicle, self.vehicle_id
        if state.get('in_junction') or str(state.get('lane_id', '')).startswith(':'):
            return 'yielding', 'clear_junction_first'
        self.remember(connection)
        if assessment['kind'] == 'crossing' and not assessment['inside_corridor']:
            if self.action != 'wait_before_conflict':
                vehicle.slowDown(name, 0., self.config.get('braking_seconds', 2.))
                self.action = 'wait_before_conflict'
            if float(state.get('speed', 0.)) < .3:
                vehicle.setSpeed(name, 0.)
                return 'cleared', 'waiting_outside_corridor'
            return 'yielding', 'braking_before_conflict'
        if self.action == 'lane_change':
            return 'yielding', 'changing_to_clear_lane'
        lane_index = vehicle.getLaneIndex(name)
        road_id = vehicle.getRoadID(name)
        count = connection.edge.getLaneNumber(road_id)
        for direction in (-1, 1):
            target = lane_index+direction
            if 0 <= target < count and vehicle.couldChangeLane(name, direction):
                # 本边上的正常相邻车道；不关闭 SUMO 安全约束，不强制冲撞。
                vehicle.changeLane(name, target, self.config.get('lane_change_seconds', 3.))
                self.action = 'lane_change'
                return 'yielding', 'safe_adjacent_lane'
        lane_id = vehicle.getLaneID(name)
        width = connection.lane.getWidth(lane_id)
        vehicle_width = float(state.get('width', 1.85))
        lateral = vehicle.getLateralLanePosition(name)
        target = -max(0., (width-vehicle_width)/2-self.config.get('road_edge_margin', .15))
        shift = target-lateral
        if shift < -.1 and self.action != 'pull_right':
            vehicle.changeSublane(name, shift)
            self.action = 'pull_right'
            return 'yielding', 'pull_right_within_drivable_lane'
        # 单车道靠边后仍不足以清空时，如实报告，避免在走廊中强制停车。
        return 'unable', 'no_safe_clearance_available'

    def restore(self, connection, state, now):
        if self.saved is None:
            return True
        vehicle, name = connection.vehicle, self.vehicle_id
        if self.restore_started is None:
            self.restore_started = now
            vehicle.setSpeedMode(name, self.saved['speed_mode'])
            vehicle.setLaneChangeMode(name, self.saved['lane_change_mode'])
            vehicle.setMaxSpeed(name, self.saved['max_speed'])
        # 保留正常跟车安全约束，分步释放原来的停车/速度命令。
        speed = float(state.get('speed', 0.))
        elapsed = now-self.restore_started
        if elapsed < self.config.get('recovery_seconds', 2.):
            vehicle.setSpeed(name, min(self.saved['max_speed'], speed+self.config.get('recovery_acceleration', 1.)*.05))
            return False
        vehicle.setSpeed(name, -1.)
        self.saved, self.action, self.restore_started = None, None, None
        return True
