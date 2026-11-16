"""持续并入左数第 3 道；仅由 SUMO 阻止车身重叠，取消跟车间距门槛。"""
import logging
from vanet.npc.mode2_route_clearance.lane_space import available_space
from vanet.npc.mode2_route_clearance.rear_gap_assistance import RearGapAssistance

LOG=logging.getLogger(__name__)
# NOOVERLAP still computes lateral limits from normal following headways.
# Suppress that proportional headway; native overlap checks remain enabled.
COLLISION_ONLY_ASSERTIVENESS=1e6


class LeftLaneManeuver:
    def __init__(self, vehicle_id, policy):
        self.vehicle_id,self.policy=vehicle_id,policy
        self.saved=None
        self.target_lane=None
        self.next_request=0.
        self.centred=False
        self.waiting_for_space=False
        self.rear_assistance=RearGapAssistance()

    def start(self, connection, lane_index):
        vehicle=connection.vehicle;name=self.vehicle_id
        if self.saved is None:
            self.saved=dict(max_speed_lat=vehicle.getMaxSpeedLat(name),parameters={
                key:vehicle.getParameter(name,'laneChangeModel.'+key)
                for key in ('lcAccelLat','lcAssertive')})
        vehicle.setMaxSpeedLat(name,self.policy.get('lateral_speed_mps',3.5))
        vehicle.setParameter(name,'laneChangeModel.lcAccelLat',str(self.policy.get('lateral_acceleration_mps2',8.)))
        vehicle.setParameter(name,'laneChangeModel.lcAssertive',str(COLLISION_ONLY_ASSERTIVENESS))
        # 1024: sublane motion; 256: TraCI NOOVERLAP with speed adaptation.
        # No speed-based following-gap veto, but never ignore physical overlap.
        vehicle.setLaneChangeMode(name,1280)
        self.target_lane=lane_index+1
        self.next_request=0.
        self.centred=False
        self.waiting_for_space=False
        LOG.info('MODE2_LEFT_START vehicle=%s from_lane=%s target_lane=%s policy=%s',
                 name,lane_index,self.target_lane,self.policy)

    def update(self, connection, state, now):
        vehicle=connection.vehicle;name=self.vehicle_id
        road=vehicle.getRoadID(name)
        index=vehicle.getLaneIndex(name)
        lateral=vehicle.getLateralLanePosition(name)
        lateral_speed=vehicle.getLateralSpeed(name)
        count=connection.edge.getLaneNumber(road)
        # The shoulder disappears on internal edges: indices change from
        # 1->2 to 0->1, but the destination remains left-counted lane three.
        self.target_lane=max(index,max(0,count-3))
        # This marks the motion's completion, not the earliest passing space.
        # YieldingActions may report clearance sooner while updates continue.
        if index==self.target_lane and abs(lateral)<=.1 and abs(lateral_speed)<=.1:
            if not self.centred:
                LOG.info('MODE2_LEFT_CENTRED vehicle=%s sim_time=%.3f lane=%s lateral=%.3f',name,now,index,lateral)
            self.centred=True
            return True
        if now<self.next_request:return False
        if index<self.target_lane and abs(lateral)<.1:
            self.rear_assistance.request(connection,name,now)
        # Evaluate actual body space rather than TraCI's minGap-subtracted
        # distance. Supporting rear vehicles can decelerate briefly to make
        # room; ordinary following headways never veto the manoeuvre.
        available,blocker=(available_space(connection,name,state,self.policy)
                           if index<self.target_lane and abs(lateral)<.1 else (True,None))
        if not available:
            if not self.waiting_for_space:
                LOG.info('MODE2_LEFT_SPACE_OCCUPIED vehicle=%s sim_time=%.3f blocker=%s',name,now,blocker)
            self.waiting_for_space=True
            self.next_request=now+self.policy.get('lane_request_interval_seconds',.2)
            return False
        self.waiting_for_space=False
        shift=-lateral
        for lane_index in range(index,self.target_lane):
            shift+=(connection.lane.getWidth(f'{road}_{lane_index}')+
                    connection.lane.getWidth(f'{road}_{lane_index+1}'))/2
        # A changeLane request targets the minimum move onto the lane and can
        # override subsequent centering with STAY. Use one full centre target.
        # SUMO keeps physical overlap protection and adapts longitudinal speed.
        vehicle.changeSublane(name,shift)
        self.next_request=now+self.policy.get('lane_request_interval_seconds',.2)
        return False

    def restore(self, connection):
        if self.saved is None:return
        vehicle=connection.vehicle;name=self.vehicle_id
        vehicle.setMaxSpeedLat(name,self.saved['max_speed_lat'])
        for key,value in self.saved['parameters'].items():
            vehicle.setParameter(name,'laneChangeModel.'+key,value)
        self.saved=None
        self.target_lane=None
        self.centred=False
        self.waiting_for_space=False
        LOG.info('MODE2_LEFT_RESTORE vehicle=%s',name)
        self.rear_assistance=RearGapAssistance()
