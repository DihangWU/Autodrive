"""联网背景车应继续发消息，但任何紧急任务都不得改变其正常驾驶。"""
import unittest

from vanet.npc.cooperation import NpcCooperation
from vanet.npc.highway_background_traffic.load_staggered_vehicles import load_staggered_traffic
from vanet.tests.offline_function_checks.test_cooperation import message, state, connection


class HighwayBackgroundTrafficTests(unittest.TestCase):
    def test_emergency_cancel_and_expiry_never_write_driving_commands(self):
        for ending in ('cancel','expiry'):
            with self.subTest(ending=ending):
                npc=NpcCooperation('new_highway_npc',dict(yielding_enabled=False,lease_seconds=10.))
                c=connection()
                self.assertTrue(npc.receive(message(),0.))
                output=npc.step(0.,state(),c)
                self.assertEqual({v['payload']['type'] for v in output},{'beacon','feedback'})
                feedback=next(v['payload'] for v in output if v['payload']['type']=='feedback')
                self.assertEqual(feedback['status'],'received')
                self.assertEqual(feedback['reason'],'yielding_disabled_for_scenario')
                if ending=='cancel':npc.receive(message('cancel',time=1.,sequence=2),1.)
                npc.step(1.,state(),c);npc.step(12.,state(),c)
                self.assertEqual(c.vehicle.mock_calls,[])
                self.assertEqual(npc.status,'normal')
                self.assertFalse(npc.snapshot()['yielding_enabled'])
                self.assertFalse(npc.snapshot()['out_of_range_yielding'])

    def test_previous_front_vehicle_still_yields(self):
        npc=NpcCooperation('old_front',{})
        c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        c.vehicle.changeLane.assert_called_once()

    def test_leaving_range_does_not_leave_a_false_yield_hold(self):
        npc=NpcCooperation('new_highway_npc',dict(yielding_enabled=False,communication_ranges={},lease_seconds=10.,
                                                out_of_range_yield_seconds=30.))
        c=connection();npc.set_communication_links(['lincoln0'],0.)
        self.assertTrue(npc.receive(message(),0.));npc.step(0.,state(),c)
        npc.set_communication_links([],1.);npc.step(1.,state(),c)
        self.assertEqual(npc.status,'normal')
        self.assertIsNone(npc.task)
        self.assertFalse(npc.snapshot()['out_of_range_yielding'])
        self.assertEqual(c.vehicle.mock_calls,[])

    def test_configuration_retains_forty_distinct_networked_vehicles(self):
        vehicles,limits=load_staggered_traffic()
        self.assertEqual(len(vehicles),40)
        self.assertEqual([sum(v['lane_index']==lane for v in vehicles) for lane in (1,2)],[20,20])
        offsets=[v['initial_placement']['corridor_distance_metres'] for v in vehicles]
        self.assertAlmostEqual(max(offsets)-min(offsets),665.)
        self.assertTrue(all(v['network_enabled'] and not v['yielding_enabled'] for v in vehicles))
        self.assertEqual(limits['maximum_speed_kmh'],100.)
        self.assertFalse(any(lane.endswith('_0') for lane in limits['lanes'] if not lane.startswith(':')))


if __name__=='__main__':unittest.main()
