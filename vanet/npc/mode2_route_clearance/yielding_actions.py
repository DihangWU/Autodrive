"""Mode2 第 4 道前车立即左移，仅保留原生车身防碰撞；匝道单独停车。"""
from vanet.npc.yielding_action_control.sumo_actions import YieldingActions
from vanet.npc.mode2_route_clearance.route_assessment import on_ramp
from vanet.npc.mode2_route_clearance.left_lane_maneuver import LeftLaneManeuver
from vanet.npc.mode2_route_clearance.lane_eligibility import fourth_lane
from vanet.npc.mode2_route_clearance.passage_clearance import leaves_passage


class Mode2YieldingActions(YieldingActions):
    def __init__(self, vehicle_id, config):
        super().__init__(vehicle_id,config)
        self.left_maneuver=LeftLaneManeuver(vehicle_id,config.get('mode2',{}))

    def apply(self, connection, state, assessment):
        if on_ramp(state,self.config):
            controller=self.config.get('ramp_controller')
            if controller is None:
                return 'unable','ramp_controller_unavailable'
            controller.emergency_hold.request(self.vehicle_id,self.config.get('mode2',{}))
            self.saved=dict(ramp_hold=True)
            self.action='ramp_hold'
            status=controller.emergency_hold.status(self.vehicle_id)
            if status=='stopped':return 'cleared','waiting_in_ramp_for_lincoln'
            if status=='committed':return 'yielding','clear_merge_before_stopping'
            return 'yielding','braking_inside_ramp'
        if self.action=='lane_change':
            if self.left_maneuver.update(connection,state,assessment.get('sim_time',0.)):
                return 'cleared','left_lane_clear_for_lincoln'
            if leaves_passage(state,assessment,self.config):
                return 'cleared','left_lane_clear_for_lincoln'
            if self.left_maneuver.waiting_for_space:
                return 'yielding','waiting_for_unoccupied_lane_space'
            return 'yielding','changing_left_for_lincoln'
        vehicle=connection.vehicle;name=self.vehicle_id
        if not fourth_lane(connection,name):
            return 'cleared','outside_lane4_keep_normal_driving'
        if assessment['kind']!='same_direction':
            return 'cleared','outside_same_direction_route'
        gap=assessment['station']-assessment['emergency_station']
        if gap<=0.:return 'cleared','lincoln_already_ahead'
        if gap>self.config.get('mode2',{}).get('lane_change_trigger_metres',45.):
            if assessment.get('distance',0.)>self.config.get('corridor_half_width',1.25)+state.get('width',1.8)/2:
                return 'cleared','outside_lincoln_lane_waiting_for_left_yield_distance'
            return 'received','waiting_for_left_yield_distance'
        index=vehicle.getLaneIndex(name)
        self.remember(connection)
        self.left_maneuver.start(connection,index)
        self.left_maneuver.update(connection,state,assessment.get('sim_time',0.))
        self.action='lane_change'
        if self.left_maneuver.waiting_for_space:
            return 'yielding','waiting_for_unoccupied_lane_space'
        return 'yielding','left_lane_requested'

    def restore(self, connection, state, now):
        if self.saved and self.saved.get('ramp_hold'):
            self.config['ramp_controller'].emergency_hold.release(self.vehicle_id)
            self.saved=self.action=self.restore_started=None
            return True
        self.left_maneuver.restore(connection)
        return super().restore(connection,state,now)
