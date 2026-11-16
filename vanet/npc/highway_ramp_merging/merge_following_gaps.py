"""汇入附近使用短时距跟车，离开后恢复；不向主路下发速度或停车命令。"""
import logging

from vanet.npc.highway_background_traffic.junction_route_split import value

LOG = logging.getLogger(__name__)


class MergeFollowingGaps:
    def __init__(self, connection, configuration):
        self.connection = connection
        self.configuration = configuration
        self.saved = {}
        self.lane_speeds = {}
        self.next_restore_check = {}

    def reset(self):
        # Scene reset removes the owned vehicles before resetting controllers.
        self.saved.clear()
        self.next_restore_check.clear()
        for lane,speed in self.lane_speeds.items():
            self.connection.lane.setMaxSpeed(lane,speed)
        self.lane_speeds.clear()

    def start(self):
        if not self.configuration:
            return
        for lane in self.configuration['ramp_acceleration_lanes']:
            if lane not in self.lane_speeds:
                self.lane_speeds[lane] = self.connection.lane.getMaxSpeed(lane)
                self.connection.lane.setMaxSpeed(lane,self.configuration['ramp_merge_speed_kmh']/3.6)

    def update(self, created, states, constants, completed, sim_time=0.):
        if not self.configuration:
            return
        policy = self.configuration
        ramps_pending = any(record['scenario'].get('role')=='ramp_merging' and name not in completed
                            for name,record in created.items())
        for name in list(self.saved):
            if name not in states:
                self.saved.pop(name)
                self.next_restore_check.pop(name,None)
        for name,record in created.items():
            entry = record['scenario']
            ramp = entry.get('role')=='ramp_merging'
            main = (entry.get('role')=='same_direction' and entry.get('yielding_enabled') is False
                    and entry.get('lane_index')==1)
            state = states.get(name)
            if not (ramp or main) or not isinstance(state,dict):
                continue
            lane = value(state,constants.VAR_LANE_ID)
            inside = lane in policy['following_control_lanes']
            if inside and (ramps_pending or name in self.saved) and name not in self.saved:
                self.saved[name] = (self.connection.vehicle.getTau(name),self.connection.vehicle.getMinGap(name))
                self.connection.vehicle.setTau(name,policy['following_time_seconds'])
                self.connection.vehicle.setMinGap(name,policy['following_minimum_gap_metres'])
                LOG.debug('RAMP_ZIPPER_FOLLOWING vehicle=%s lane=%s tau=%s min_gap=%s',name,lane,
                          policy['following_time_seconds'],policy['following_minimum_gap_metres'])
            elif not inside and name in self.saved:
                if sim_time < self.next_restore_check.get(name,0.):
                    continue
                self.next_restore_check[name] = sim_time+1.
                tau,gap = self.saved[name]
                speed = value(state,getattr(constants,'VAR_SPEED',64)) or 0.
                leader = self.connection.vehicle.getLeader(name,100.)
                # Increasing minGap/tau while cars are still close can create
                # an artificial collision or a sudden brake at the zone exit.
                if leader and leader[1] < gap+speed*tau+2.:
                    continue
                self.saved.pop(name)
                self.next_restore_check.pop(name,None)
                self.connection.vehicle.setTau(name,tau)
                self.connection.vehicle.setMinGap(name,gap)
                LOG.debug('RAMP_ZIPPER_FOLLOWING_RESTORE vehicle=%s lane=%s',name,lane)
