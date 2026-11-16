"""编排匝道提前配速与逐辆拉链汇入，进入主路后恢复普通选道。"""
import logging

from vanet.npc.highway_background_traffic.junction_route_split import value
from vanet.npc.highway_ramp_merging.zipper_release_meter import ZipperReleaseMeter
from vanet.npc.highway_ramp_merging.rolling_merge_priority import RollingMergePriority
from vanet.npc.highway_ramp_merging.merge_following_gaps import MergeFollowingGaps
from vanet.npc.highway_ramp_merging.emergency_hold import RampEmergencyHold

LOG = logging.getLogger(__name__)


class RampMergeController:
    def __init__(self, connection, configuration=None):
        self.connection = connection
        self.configuration = configuration
        self.meter = ZipperReleaseMeter(connection,(configuration or {}).get('zipper_meter'))
        self.rolling_priority = RollingMergePriority(connection,(configuration or {}).get('zipper_meter'))
        self.following = MergeFollowingGaps(connection,(configuration or {}).get('zipper_meter'))
        self.emergency_hold = RampEmergencyHold(connection,self.meter)
        self.reset()

    def reset(self):
        self.start_time = None
        self.released = set()
        self.completed = set()
        self.next_request = {}
        self.meter.reset()
        self.rolling_priority.reset()
        self.following.reset()
        self.emergency_hold.reset()

    def start(self, created, sim_time):
        self.start_time = float(sim_time)
        self.following.start()
        self.meter.start(sim_time)
        self.release(created,sim_time)

    def release(self, created, sim_time):
        if not self.configuration or self.start_time is None:
            return
        if self.meter.configuration:
            return
        for name,record in created.items():
            entry = record['scenario']
            if not entry.get('hold_for_merge_release') or name in self.released:
                continue
            if sim_time < self.start_time+entry['merge_release_delay_seconds']:
                continue
            self.connection.vehicle.setSpeed(name,-1.)
            self.released.add(name)
            LOG.info('RAMP_MERGE_RELEASE sim_time=%.3f vehicle=%s target_left_to_right_lane=%s',
                     sim_time,name,self.configuration['target_left_to_right_lane'])

    def update_lanes(self, created, states, constants, sim_time):
        if not self.configuration:
            return
        self.following.update(created,states,constants,self.completed,sim_time)
        held=self.emergency_hold.update(states,constants,sim_time)
        self.meter.update_slots(created,states,constants,self.released,self.completed,sim_time,held)
        self.rolling_priority.reservations = self.meter.reservations
        self.rolling_priority.main_crossings = self.meter.main_crossings
        self.rolling_priority.held=held
        self.rolling_priority.use_physical_order=self.meter.rebooking
        self.rolling_priority.update(created,states,constants,self.released-self.completed-held)
        target = self.configuration['target_sumo_lane_index']
        for name in sorted(self.released-self.completed):
            if sim_time < self.next_request.get(name,0.):
                continue
            state = states.get(name)
            if not isinstance(state,dict):
                continue
            road = value(state,constants.VAR_ROAD_ID)
            if road not in self.configuration['merge_edges']:
                continue
            self.next_request[name] = sim_time+self.configuration['lane_request_interval_seconds']
            lane = value(state,constants.VAR_LANE_ID)
            if lane is None:
                lane_index = self.connection.vehicle.getLaneIndex(name)
            else:
                lane_index = int(lane.rsplit('_',1)[1])
            lateral = value(state,constants.VAR_LANEPOSITION_LAT)
            if lateral is None:
                lateral = self.connection.vehicle.getLateralLanePosition(name)
            # Finish the current single-lane transition before asking for the
            # next one. A lane index changes before the body reaches its centre.
            if abs(lateral) > .2:
                continue
            if lane_index == target:
                self.connection.vehicle.setMaxSpeed(name,self.configuration['maximum_speed_kmh']/3.6)
                self.connection.vehicle.setLaneChangeMode(name,self.configuration['lane_change_mode_after_merge'])
                self.completed.add(name)
                LOG.info('RAMP_MERGE_COMPLETE sim_time=%.3f vehicle=%s road=%s sumo_lane_index=%s',
                         sim_time,name,road,lane_index)
                continue
            next_lane = lane_index+(1 if lane_index < target else -1)
            # A persistent request lets SUMO adapt longitudinal speed to find
            # a gap. Lane-change mode retains its gap/braking safety checks.
            self.connection.vehicle.changeLane(name,next_lane,self.configuration['lane_request_duration_seconds'])
            LOG.info('RAMP_MERGE_REQUEST sim_time=%.3f vehicle=%s road=%s from_lane=%s requested_lane=%s',
                     sim_time,name,road,lane_index,next_lane)

    def snapshot(self):
        return dict(start_time=self.start_time,released=len(self.released),completed=len(self.completed))
