"""通信范围、实际收包许可及地图方向的边界回归。"""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.rsu.coverage_display.coverage_circle import CoverageCircle
from vanet.omnet.map_and_communication_display import scene_configuration
from vanet.tests.offline_function_checks.test_cooperation import message,state

CONFIG=dict(communication_ranges=dict(v2v_metres=50.,rsu_metres=170.),rsu=dict(id='rsu0',x=193.,y=-179.,z=6.))


class RangeLeaseTests(unittest.TestCase):
    def test_npc_beacon_is_addressed_to_rsu_not_other_npcs(self):
        npc=NpcCooperation('npc0',CONFIG)
        messages=npc.step(0.,state(),Mock())
        beacons=[message for message in messages if message['payload']['type']=='beacon']
        self.assertEqual([message['recipient'] for message in beacons],['rsu0'])

    def npc(self):
        npc=NpcCooperation('npc0',CONFIG)
        npc.actions=Mock(saved=None,action=None,restore_started=None)
        npc.actions.apply.return_value=('yielding','safe_adjacent_lane')
        npc.actions.restore.return_value=False
        return npc

    def test_eligible_peer_is_not_a_received_message(self):
        npc=self.npc();npc.set_communication_links(['lincoln0'],0.)
        npc.step(0.,state(),Mock());npc.actions.apply.assert_not_called()
        self.assertFalse(self.npc().receive(message(),0.))

    def test_exit_restores_before_lease_and_reentry_does_not_revive_old_permission(self):
        npc=self.npc();npc.set_communication_links(['lincoln0'],0.)
        npc.receive(message(),0.);npc.step(0.,state(),Mock())
        npc.set_communication_links([],1.);npc.step(1.,state(),Mock())
        self.assertEqual((npc.status,npc.reason),('recovering','out_of_communication_range'))
        npc.actions.restore.assert_called_once()
        npc.set_communication_links(['lincoln0'],1.05);npc.step(1.05,state(),Mock())
        self.assertTrue(npc.recovering)
        self.assertTrue(npc.receive(message(time=1.1,sequence=2),1.1))
        self.assertFalse(npc.recovering)

    def test_other_actually_received_source_can_continue_but_old_source_cannot_reenter(self):
        npc=self.npc();npc.set_communication_links(['lincoln0','rsu0'],0.)
        npc.receive(message(),0.)
        npc.receive(message('notice',source='rsu0',time=.1,route=message()['route']),.1)
        npc.set_communication_links(['rsu0'],1.);npc.step(1.,state(),Mock())
        self.assertFalse(npc.recovering)
        npc.set_communication_links(['lincoln0'],2.);npc.step(2.,state(),Mock())
        self.assertTrue(npc.recovering)

    def test_rsu_local_cancel_preserves_direct_v2v(self):
        npc=self.npc();npc.set_communication_links(['lincoln0','rsu0'],0.)
        npc.receive(message(),0.)
        npc.receive(message('notice',source='rsu0',route=message()['route']),0.)
        npc.receive(message('cancel',source='rsu0',sequence=2,time=1.,scope='selection'),1.)
        npc.step(1.,state(),Mock());self.assertFalse(npc.recovering)

    def test_rsu_selects_only_received_beacons_and_stops_after_lincoln_leaves(self):
        rsu=RsuCooperation(CONFIG);rsu.set_communication_links(['lincoln0','npc0','unheard'],0.)
        rsu.receive(message(),0.)
        rsu.receive(message('beacon',source='npc0',state=state()),0.)
        notices=[m for m in rsu.step(0.) if m['payload']['type']=='notice']
        self.assertEqual([m['recipient'] for m in notices],['npc0'])
        rsu.set_communication_links(['npc0'],1.)
        self.assertIsNone(rsu.task)
        self.assertFalse(any(m['payload']['type']=='notice' for m in rsu.step(1.)))


class CoverageGeometryTests(unittest.TestCase):
    def test_sumo_coverage_uses_same_radius_and_coordinate_conversion(self):
        c=SimpleNamespace(polygon=Mock(),poi=Mock());c.polygon.getIDList.return_value=[]
        circle=CoverageCircle(c,dict(offset=(500.,400.)),CONFIG)
        circle.setup([CONFIG['rsu']])
        shape=c.polygon.add.call_args.args[1]
        self.assertEqual(len(shape),129)
        self.assertTrue(all(abs(math.dist(point,(693.,579.))-170.)<1e-8 for point in shape))
        self.assertEqual(c.polygon.add.call_args.args[2],(20,210,85,255))
        self.assertFalse(c.polygon.add.call_args.kwargs['fill'])

    def test_town04_canvas_is_aligned_with_sumo_and_circle_excludes_highways(self):
        with tempfile.TemporaryDirectory() as folder:
            scene,_=scene_configuration.generate_scene(Path(folder))
        self.assertGreater(scene['highway_clearance_metres'],173.)
        self.assertGreater(scene['highway_coverage_margin_metres'],3.)
        self.assertEqual(scene['communication_ranges']['v2v_metres'],50.)
        # CARLA y 增大 = SUMO y 减小 = Qtenv 画布 y 增大，均朝屏幕下方。
        first_y=-200.+scene['coordinate_offset_y'];second_y=-100.+scene['coordinate_offset_y']
        self.assertGreater(second_y,first_y)
        self.assertTrue(all(min(point)>=0 for lane in scene['lanes'] for point in lane['points']))

    def test_range_reaching_highway_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(scene_configuration,'communication_ranges',return_value=dict(v2v_metres=50.,rsu_metres=180.)):
                with self.assertRaisesRegex(ValueError,'reaches highway'):
                    scene_configuration.generate_scene(Path(folder))


if __name__=='__main__':unittest.main()
