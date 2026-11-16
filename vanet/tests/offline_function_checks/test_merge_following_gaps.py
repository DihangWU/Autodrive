"""短时距参数局限于汇入车流，离开和复位会恢复。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.highway_ramp_merging.merge_following_gaps import MergeFollowingGaps

TC=SimpleNamespace(VAR_LANE_ID=81)


class MergeFollowingGapTests(unittest.TestCase):
    def test_main_flow_keeps_its_speed_and_only_local_following_parameters_change(self):
        _,policy=load_merging_vehicles();c=Mock()
        c.lane.getMaxSpeed.return_value=13.89
        c.vehicle.getTau.return_value=1.;c.vehicle.getMinGap.return_value=2.5
        c.vehicle.getLeader.return_value=None
        c.vehicle.getLeader.return_value=None
        controller=MergeFollowingGaps(c,policy['zipper_meter']);controller.start()
        created={'main':dict(scenario=dict(role='same_direction',yielding_enabled=False,lane_index=1)),
                 'ramp':dict(scenario=dict(role='ramp_merging')),
                 'background':dict(scenario=dict(role='non_network_background'))}
        states={'main':{81:'-46_1'},'ramp':{81:'31_1'},'background':{81:'-46_3'}}
        controller.update(created,states,TC,set())
        self.assertEqual(set(controller.saved),{'main','ramp'})
        c.vehicle.setTau.assert_any_call('main',.45)
        c.vehicle.setMinGap.assert_any_call('ramp',1.5)
        c.vehicle.setSpeed.assert_not_called();c.vehicle.setMaxSpeed.assert_not_called()
        c.vehicle.slowDown.assert_not_called();c.vehicle.openGap.assert_not_called()
        controller.update(created,dict(states,main={81:'-50_2'}),TC,set())
        c.vehicle.setTau.assert_any_call('main',1.)
        c.vehicle.setMinGap.assert_any_call('main',2.5)
        self.assertNotIn('main',controller.saved)
        controller.reset()
        c.lane.setMaxSpeed.assert_any_call('31_1',13.89)
        c.lane.setMaxSpeed.assert_any_call(':1061_0_1',13.89)
        self.assertFalse(controller.saved);self.assertFalse(controller.lane_speeds)

    def test_removed_vehicles_do_not_receive_restore_commands(self):
        _,policy=load_merging_vehicles();c=Mock()
        controller=MergeFollowingGaps(c,policy['zipper_meter'])
        controller.saved={'gone':(1.,2.5)}
        controller.update({}, {},TC,set())
        self.assertFalse(controller.saved)
        c.vehicle.setTau.assert_not_called();c.vehicle.setMinGap.assert_not_called()

    def test_parameter_restore_waits_for_space_instead_of_creating_a_gap_collision(self):
        _,policy=load_merging_vehicles();c=Mock()
        controller=MergeFollowingGaps(c,policy['zipper_meter'])
        controller.saved={'main':(1.,2.5)}
        created={'main':dict(scenario=dict(role='same_direction',yielding_enabled=False,lane_index=1))}
        states={'main':{81:'-50_2',64:10.}}
        c.vehicle.getLeader.return_value=('ahead',2.)
        controller.update(created,states,TC,set(),0.)
        c.vehicle.setMinGap.assert_not_called()
        controller.update(created,states,TC,set(),.05)
        self.assertEqual(c.vehicle.getLeader.call_count,1)
        c.vehicle.getLeader.return_value=('ahead',30.)
        controller.update(created,states,TC,set(),1.)
        c.vehicle.setMinGap.assert_called_once_with('main',2.5)


if __name__=='__main__':unittest.main()
