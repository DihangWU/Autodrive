"""按主路到达顺序预约一比一合流时隙；匝道配速，主路保持正常驾驶。"""
import logging
import math

from vanet.npc.highway_background_traffic.junction_route_split import value

LOG = logging.getLogger(__name__)


class ZipperReleaseMeter:
    def __init__(self, connection, configuration):
        self.connection = connection
        self.configuration = configuration
        self.reset()

    def reset(self):
        self.previous = {}
        self.main_passed = 0
        self.events = []
        self.reservations = {}
        self.main_crossings = {}
        self.ramp_crossings = {}
        self.commanded = {}
        self.last_update = None
        self.start_time = None

    def start(self, sim_time):
        if self.start_time is None:
            self.start_time = sim_time

    def main_distance(self, lane, position):
        if position is None:
            return None
        remaining = self.configuration['main_conflict_position_metres']-position
        if lane == self.configuration['counting_lane']:
            return remaining
        extra = self.configuration['main_upstream_distances_metres'].get(lane)
        return None if extra is None else remaining+extra

    def ramp_distance(self, lane, position):
        if position is None:
            return None
        remaining = self.configuration['ramp_conflict_position_metres']-position
        if lane == self.configuration['ramp_internal_lane']:
            return remaining
        if lane == self.configuration['ramp_approach_lane']:
            return remaining+self.configuration['ramp_approach_lane_length_metres']
        return None

    def update_slots(self, created, states, constants, released, completed, sim_time):
        if not self.configuration or self.start_time is None:
            return
        self.observe(created,states,constants,sim_time)
        if self.last_update is not None and sim_time-self.last_update < self.configuration['control_interval_seconds']-1e-6:
            return
        self.last_update = sim_time
        policy = self.configuration
        main = []
        for name,state in states.items():
            entry = created.get(name,{}).get('scenario',{})
            if entry.get('role') != 'same_direction' or entry.get('yielding_enabled') is not False:
                continue
            lane = value(state,constants.VAR_LANE_ID)
            distance = self.main_distance(lane,value(state,constants.VAR_LANEPOSITION))
            if distance is not None and distance > 0.:
                speed = value(state,constants.VAR_SPEED) or 0.
                acceleration = policy['prediction_acceleration_mps2']
                limit = entry['maximum_speed_kmh']/3.6
                accelerating = max(0.,(limit*limit-speed*speed)/(2*acceleration))
                if distance <= accelerating:
                    eta = (math.sqrt(speed*speed+2*acceleration*distance)-speed)/acceleration
                else:
                    eta = max(0.,(limit-speed)/acceleration)+(distance-accelerating)/limit
                main.append((distance,name,sim_time+eta,min(limit,math.sqrt(speed*speed+2*acceleration*distance))))
        main.sort()
        reserved = {record['main'] for record in self.reservations.values() if record['main'] is not None}
        queue = sorted((record['scenario']['merge_queue_index'],name) for name,record in created.items()
                       if record['scenario'].get('role')=='ramp_merging' and name not in completed)
        predictions = {name:(eta,speed) for _,name,eta,speed in main}
        for _,name in queue:
            state = states.get(name)
            if not isinstance(state,dict):
                continue
            distance = self.ramp_distance(value(state,constants.VAR_LANE_ID),value(state,constants.VAR_LANEPOSITION))
            if distance is None:
                continue
            if distance <= -float(created[name]['spec']['length']):
                self.command_speed(name,-1.,sim_time)
                released.add(name)
                continue
            reservation = self.reservations.get(name)
            if reservation is None:
                candidate = next((item for item in main if item[1] not in reserved),None)
                if candidate is not None:
                    _,leader,eta,speed = candidate
                    reservation = dict(main=leader,target=eta+policy['slot_headway_seconds'],speed=speed)
                    reserved.add(leader)
                elif not main:
                    reservation = dict(main=None,target=sim_time,speed=policy['ramp_merge_speed_kmh']/3.6)
                else:
                    continue
                self.reservations[name] = reservation
                event = dict(vehicle=name,sim_time=sim_time,main_vehicle=reservation['main'],
                    target_merge_time=reservation['target'],ramp_speed_limit_kmh=policy['ramp_merge_speed_kmh'],
                    required_main_vehicles=1,reason='main_slot' if reservation['main'] else 'main_empty')
                self.events.append(event)
                LOG.info('RAMP_ZIPPER_SLOT %s',event)
            leader = reservation['main']
            if leader is not None and leader not in predictions and leader not in self.main_crossings:
                reservation.update(main=None,target=sim_time)
                LOG.info('RAMP_ZIPPER_SLOT_CANCEL sim_time=%.3f vehicle=%s absent_main=%s',sim_time,name,leader)
                leader = None
            if leader in self.main_crossings:
                reservation['target'] = self.main_crossings[leader]+policy['slot_headway_seconds']
            elif leader in predictions:
                eta,speed = predictions[leader]
                reservation.update(target=eta+policy['slot_headway_seconds'],speed=speed)
            speed = min(reservation['speed'],policy['ramp_merge_speed_kmh']/3.6)
            remaining_time = reservation['target']-sim_time
            target = speed if remaining_time <= 0. else max(0.,min(speed,2*max(0.,distance)/remaining_time-speed))
            self.command_speed(name,target,sim_time)
            released.add(name)

    def command_speed(self, name, speed, sim_time):
        previous = self.commanded.get(name)
        if previous is None or abs(previous-speed) >= .15:
            self.connection.vehicle.setSpeed(name,speed)
            self.commanded[name] = speed

    def observe(self, created, states, constants, sim_time=None):
        if not self.configuration:
            return
        for name,state in states.items():
            if not isinstance(state,dict):
                continue
            ramp = created.get(name,{}).get('scenario',{}).get('role')=='ramp_merging'
            lane = value(state,constants.VAR_LANE_ID)
            position = value(state,constants.VAR_LANEPOSITION)
            previous_lane,previous_position = self.previous.get(name,(None,None))
            target_lane = self.configuration['ramp_internal_lane'] if ramp else self.configuration['counting_lane']
            line = self.configuration['ramp_conflict_position_metres' if ramp else 'main_conflict_position_metres']
            crossed = lane==target_lane and position is not None and position>=line and (
                previous_lane!=lane or previous_position is not None and previous_position<line)
            crossings = self.ramp_crossings if ramp else self.main_crossings
            if crossed and name not in crossings:
                crossings[name] = sim_time if sim_time is not None else 0.
                if ramp:
                    LOG.info('RAMP_ZIPPER_CROSS sim_time=%s vehicle=%s preceding_main=%s',sim_time,name,
                             self.reservations.get(name,{}).get('main'))
                else:
                    self.main_passed += 1
            self.previous[name] = (lane,position)

    def snapshot(self):
        return dict(main_passed=self.main_passed,main_credit=max(0,self.main_passed-len(self.ramp_crossings)),
                    ramp_crossed=len(self.ramp_crossings),gap_main=None,releases=list(self.events))
