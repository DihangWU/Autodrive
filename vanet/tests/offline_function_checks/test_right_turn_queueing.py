"""右转排队规则只作用于指定转向车辆，并在驶出入口后恢复。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.npc.highway_background_traffic.right_turn_queueing import RightTurnQueueing,KEEP_CLEAR_PARAMETER

TC = SimpleNamespace(VAR_ROAD_ID=80,VAR_EDGES=84)
POLICY = dict(approach_edge='-48',internal_edges=[':890_2',':890_13'],
              exit_edge='-7',ignore_keep_clear_after_seconds=0.0)


class RightTurnQueueingTests(unittest.TestCase):
    def test_scope_private_type_and_restoration_without_speed_or_light_override(self):
        c=Mock();c.vehicle.getMaxSpeed.return_value=80/3.6
        c.vehicle.getTypeID.return_value='audi@turn'
        c.vehicletype.getParameter.return_value='7'
        policy=RightTurnQueueing(c,POLICY)
        created={name:dict(scenario={}) for name in ('turn','straight','other')}
        created['static']=dict(scenario=dict(stationary=True))
        states={'turn':{'80':'-48','84':['-48','-7','-8']},
                'straight':{80:'-48',84:['-48','-49']},
                'other':{80:'-41',84:['-41','-48','-7']},
                'static':{80:'-48',84:['-48','-7']}}
        policy.update(created,states,TC,1.)
        c.vehicle.setMaxSpeed.assert_called_once_with('turn',80/3.6)
        c.vehicletype.setParameter.assert_called_once_with('audi@turn',KEEP_CLEAR_PARAMETER,'0.0')
        states['turn']={80:':890_13',84:['-48','-7','-8']}
        policy.update(created,states,TC,2.)
        self.assertEqual(c.vehicletype.setParameter.call_count,1)
        states['turn']={80:'-7',84:['-48','-7','-8']}
        policy.update(created,states,TC,3.)
        self.assertEqual(policy.active,{})
        c.vehicletype.setParameter.assert_called_with('audi@turn',KEEP_CLEAR_PARAMETER,'7')
        c.vehicle.setSpeed.assert_not_called()
        c.vehicle.setSpeedMode.assert_not_called()
        c.vehicle.moveTo.assert_not_called()
        c.trafficlight.setRedYellowGreenState.assert_not_called()
        c.vehicle.getRoute.assert_not_called()

    def test_missing_route_default_parameter_and_removed_vehicle_reset(self):
        c=Mock();c.vehicle.getRoute.return_value=['-48','-7']
        c.vehicle.getTypeID.return_value='audi@old_front'
        c.vehicletype.getParameter.return_value=''
        policy=RightTurnQueueing(c,POLICY)
        created={'old_front':dict(scenario={})}
        policy.update(created,{'old_front':{80:'-48'}},TC,1.)
        self.assertEqual(policy.active['old_front'],('audi@old_front','-1'))
        policy.update(created,{},TC,2.)
        self.assertIn('old_front',policy.active)
        policy.reset()
        self.assertEqual(policy.active,{})


if __name__=='__main__':unittest.main()
