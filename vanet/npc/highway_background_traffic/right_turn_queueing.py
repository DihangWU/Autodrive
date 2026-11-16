"""进城右转入口允许利用弯道排队空间，保留红灯、跟车和冲突让行。"""
import logging

from vanet.npc.highway_background_traffic.junction_route_split import value

LOG = logging.getLogger(__name__)
KEEP_CLEAR_PARAMETER = 'junctionModel.jmIgnoreKeepClearTime'


class RightTurnQueueing:
    def __init__(self, connection, configuration=None):
        self.connection = connection
        self.configuration = configuration
        self.active = {}

    def reset(self):
        """所属车辆删除后清理记录；独立车型随车辆销毁。"""
        self.active.clear()

    def update(self, created, states, constants, sim_time):
        if not self.configuration:
            return
        approach = self.configuration['approach_edge']
        exit_edge = self.configuration['exit_edge']
        area = {approach, *self.configuration['internal_edges']}
        vehicle = self.connection.vehicle
        types = self.connection.vehicletype
        for name, record in created.items():
            if (record['scenario'].get('stationary',False)
                    or record['scenario'].get('role') == 'non_network_background'):
                continue
            state = states.get(name)
            if not isinstance(state,dict):
                continue
            road = value(state,constants.VAR_ROAD_ID)
            if road is None:
                continue
            if name in self.active:
                if road not in area:
                    type_id, original = self.active[name]
                    types.setParameter(type_id,KEEP_CLEAR_PARAMETER,original)
                    del self.active[name]
                    LOG.info('RIGHT_TURN_QUEUE sim_time=%.3f vehicle=%s road=%s action=restore keep_clear_time=%s',
                             sim_time,name,road,original)
                continue
            if road not in area:
                continue
            route = value(state,constants.VAR_EDGES)
            if route is None:
                route = vehicle.getRoute(name)
            if not any(first == approach and second == exit_edge for first,second in zip(route,route[1:])):
                continue
            # SUMO 1.22 exposes this JM setting on vehicle types only. Setting
            # the unchanged max speed creates a private type for this vehicle,
            # so the shared Audi type and other intersections remain untouched.
            vehicle.setMaxSpeed(name,vehicle.getMaxSpeed(name))
            type_id = vehicle.getTypeID(name)
            original = types.getParameter(type_id,KEEP_CLEAR_PARAMETER) or '-1'
            types.setParameter(type_id,KEEP_CLEAR_PARAMETER,
                               str(self.configuration['ignore_keep_clear_after_seconds']))
            self.active[name] = (type_id,original)
            LOG.info('RIGHT_TURN_QUEUE sim_time=%.3f vehicle=%s road=%s action=allow_curve_queue type=%s',
                     sim_time,name,road,type_id)
