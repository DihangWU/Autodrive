"""只在安全时间窗口内排除被长路口误判成前车的合流后车。"""
import math
import logging

from vanet.npc.highway_background_traffic.junction_route_split import value

LOG = logging.getLogger(__name__)


class RollingMergePriority:
    PARAMETER = 'junctionModel.ignoreIDs'

    def __init__(self, connection, configuration):
        self.connection = connection
        self.configuration = configuration
        self.reset()

    def reset(self):
        self.original = {}
        self.applied = {}
        self.external = []
        self.reservations = {}
        self.main_crossings = {}
        self.held=set()
        self.use_physical_order=False

    def update(self, created, states, constants, active):
        if not self.configuration:
            return
        policy = self.configuration
        for name in list(self.applied):
            if name not in states:
                self.applied.pop(name);self.original.pop(name,None)
        main_states = {name:state for name,state in states.items() if isinstance(state,dict)
                       and value(state,constants.VAR_LANE_ID) in
                       (*policy['main_approach_lanes'],policy['counting_lane'])}
        order={reservation['main']:created[ramp]['scenario']['merge_queue_index']
               for ramp,reservation in self.reservations.items() if reservation['main'] is not None}
        order={reservation['main']:created[ramp]['scenario']['merge_queue_index']
               for ramp,reservation in self.reservations.items() if reservation['main'] is not None}
        def main_distance(state):
            lane=value(state,constants.VAR_LANE_ID);position=value(state,constants.VAR_LANEPOSITION)
            if position is None:return None
            if lane==policy['counting_lane']:return policy['main_conflict_position_metres']-position
            extra=policy['main_upstream_distances_metres'].get(lane)
            return None if extra is None else policy['main_conflict_position_metres']+extra-position
        for name in sorted(active|set(self.applied)|set(main_states)):
            state = states.get(name)
            if not isinstance(state,dict):continue
            lane = value(state,constants.VAR_LANE_ID)
            position = value(state,constants.VAR_LANEPOSITION)
            ignored = set()
            clear_time = 0.
            if name in main_states:
                # A reserved leader has priority over its following ramp car.
                # SUMO's long internal links otherwise make both wait for each
                # other even before their physical paths meet.
                for ramp,record in created.items():
                    if record['scenario'].get('role')!='ramp_merging':continue
                    leader = self.reservations.get(ramp,{}).get('main')
                    ramp_state = states.get(ramp)
                    if not isinstance(ramp_state,dict):continue
                    ramp_lane = value(ramp_state,constants.VAR_LANE_ID)
                    ramp_station = value(ramp_state,constants.VAR_LANEPOSITION)
                    if ramp_lane not in (policy['ramp_approach_lane'],policy['ramp_internal_lane']):continue
                    before_merge = (ramp_lane==policy['ramp_approach_lane'] or
                                    ramp_station is not None and ramp_station<policy['ramp_conflict_position_metres'])
                    own_leader_ahead = (leader==name and lane==policy['counting_lane'] and position is not None
                        and ramp_station is not None and
                        position-policy['main_conflict_position_metres'] >
                        ramp_station-policy['ramp_conflict_position_metres']+6.)
                    # An upstream car without a slot is still behind the
                    # scheduled ramp queue; it must not trigger an early yield
                    # by a main car that already owns an earlier turn.
                    leader_distance=main_distance(states.get(leader,{}))
                    distance=main_distance(state)
                    ramp_remaining=policy['ramp_conflict_position_metres']-(ramp_station or 0.)
                    if ramp_lane==policy['ramp_approach_lane']:ramp_remaining+=policy['ramp_approach_lane_length_metres']
                    ramp_speed=value(ramp_state,constants.VAR_SPEED) or 0.
                    safely_upstream=ramp_remaining>ramp_speed*ramp_speed/(2*policy['ramp_braking_mps2'])+6.
                    follows_main=(ramp not in self.reservations and safely_upstream or leader_distance is not None
                                  and distance is not None and distance<=leader_distance)
                    if not self.use_physical_order:
                        follows_main=record['scenario']['merge_queue_index']>=order.get(name,float('inf'))
                    if not self.use_physical_order:
                        follows_main=record['scenario']['merge_queue_index']>=order.get(name,float('inf'))
                    if own_leader_ahead or before_merge and (follows_main or ramp in self.held):
                        ignored.add(ramp)
            if name in active and lane==policy['ramp_internal_lane'] and position is not None:
                length = float(created[name]['spec']['length'])
                distance = policy['ramp_conflict_position_metres']+length+1.-position
                if 0. < distance < 40.:
                    speed = value(state,constants.VAR_SPEED) or 0.
                    acceleration = 1.5
                    clear_time = (math.sqrt(speed*speed+2*acceleration*distance)-speed)/acceleration+.5
                    for other,other_state in states.items():
                        if other==name or not isinstance(other_state,dict):continue
                        other_lane = value(other_state,constants.VAR_LANE_ID)
                        station = value(other_state,constants.VAR_LANEPOSITION)
                        other_speed = value(other_state,constants.VAR_SPEED)
                        if station is None or other_speed is None:continue
                        remaining = policy['main_conflict_position_metres']-station
                        if other_lane in policy['main_approach_lanes']:
                            remaining += policy['main_approach_lane_length_metres']
                        elif other_lane != policy['counting_lane']:continue
                        if remaining < 6.:continue
                        managed = created.get(other,{}).get('scenario',{}).get('yielding_enabled') is False
                        if managed:
                            limit = created[other]['scenario'].get('maximum_speed_kmh',100.)/3.6
                            arrival = remaining/max(other_speed,limit)
                        else:
                            arrival = (math.sqrt(other_speed*other_speed+6*remaining)-other_speed)/3.
                        if arrival > clear_time:ignored.add(other)
                    point = policy['main_conflict_carla']
                    for external in self.external:
                        angle = math.radians(external.get('heading',-90.))
                        half = external.get('length',5.)/2
                        x = external['x']+half*math.cos(angle)
                        y = external['y']+half*math.sin(angle)
                        remaining = y-point['y']
                        if abs(x-point['x']) <= 2.5 and remaining > 6.:
                            velocity = external.get('speed',0.)
                            arrival = (math.sqrt(velocity*velocity+16*remaining)-velocity)/8.
                            if arrival > clear_time:ignored.add(external.get('sumo_id','lincoln0'))
            if name in active and position is not None:
                upstream = (lane==policy['ramp_approach_lane'] or
                            lane==policy['ramp_internal_lane'] and position < 8.)
                if upstream:
                    remaining = policy['ramp_conflict_position_metres']-position
                    if lane==policy['ramp_approach_lane']:
                        remaining += policy['ramp_approach_lane_length_metres']
                    speed = value(state,constants.VAR_SPEED) or 0.
                    stopping = speed*speed/(2*policy['ramp_braking_mps2'])+speed*.2+6.
                    if remaining > stopping:
                        ignored.update(main_states)
            if ignored:
                if name not in self.original:
                    self.original[name] = self.connection.vehicle.getParameter(name,self.PARAMETER)
                combined = set(self.original[name].split())|ignored
                text = ' '.join(sorted(combined))
                if self.applied.get(name) != text:
                    self.connection.vehicle.setParameter(name,self.PARAMETER,text)
                    self.applied[name] = text
                    LOG.debug('RAMP_ROLLING_PRIORITY vehicle=%s clearance_seconds=%.3f trailing_vehicles=%s',
                              name,clear_time,sorted(ignored))
            elif name in self.applied:
                self.connection.vehicle.setParameter(name,self.PARAMETER,self.original.pop(name))
                self.applied.pop(name)
                LOG.debug('RAMP_ROLLING_PRIORITY_RESTORE vehicle=%s',name)
