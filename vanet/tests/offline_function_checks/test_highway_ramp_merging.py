"""汇入车错峰启动、安全逐道请求和无线避让隔离。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.highway_ramp_merging.merge_controller import RampMergeController
from vanet.npc.highway_ramp_merging.zipper_release_meter import ZipperReleaseMeter
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import load_scenario
from vanet.npc.highway_background_traffic.load_staggered_vehicles import load_staggered_traffic
from vanet.npc.cooperation import NpcCooperation
from vanet.tests.offline_function_checks.test_cooperation import message,state,connection

TC=SimpleNamespace(VAR_ROAD_ID=80,VAR_LANE_ID=81,VAR_LANEPOSITION_LAT=184)


class HighwayRampMergingTests(unittest.TestCase):
    def test_thirty_networked_non_yielding_vehicles_and_existing_main_traffic_unchanged(self):
        vehicles,policy=load_merging_vehicles()
        self.assertEqual(len(vehicles),30)
        self.assertEqual(len({v['id'] for v in vehicles}),30)
        self.assertEqual(sum('placement_lane_id' in v for v in vehicles),6)
        self.assertEqual([v['position'] for v in vehicles[:10]],[round(140.-i*10.8,6) for i in range(10)])
        self.assertTrue(all(v['depart_offset']==0. for v in vehicles))
        self.assertEqual(sum(v['route'][0]=='31' for v in vehicles),13)
        self.assertEqual(sum(v['route'][0]=='39' for v in vehicles),17)
        self.assertEqual(policy['target_sumo_lane_index'],1)
        self.assertEqual(policy['target_left_to_right_lane'],4)
        self.assertEqual([v['merge_release_delay_seconds'] for v in vehicles],policy['release_offsets_seconds'])
        for entry in vehicles:
            self.assertEqual(entry['route'][-5:],['31','-47','-48','-49','-50'])
            self.assertTrue(entry['network_enabled']);self.assertFalse(entry['yielding_enabled'])
            self.assertEqual(entry['speed_mode'],31);self.assertEqual(entry['lane_change_mode'],1536)
            self.assertTrue(entry['hold_for_merge_release'])
        scene=load_scenario();original,_=load_staggered_traffic()
        self.assertEqual([v for v in scene['npcs'] if v['id'].startswith('vanet_npc_highway_')],original)
        self.assertEqual(sum(v.get('network_enabled',True) for v in scene['npcs']),74)
        self.assertEqual(sum(v.get('role')=='non_network_background' for v in scene['npcs']),100)
        npc=NpcCooperation(vehicles[0]['id'],dict(yielding_enabled=False))
        c=connection();npc.receive(message(),0.);output=npc.step(0.,state(),c)
        self.assertEqual(c.vehicle.mock_calls,[])
        self.assertEqual(next(v['payload'] for v in output if v['payload']['type']=='feedback')['status'],'received')

    def test_release_from_start_is_staggered_not_conditioned_on_lincoln(self):
        vehicles,policy=load_merging_vehicles();c=Mock()
        controller=RampMergeController(c,dict(policy,zipper_meter=None))
        created={entry['id']:dict(scenario=entry) for entry in vehicles}
        controller.release(created,100.)
        c.vehicle.setSpeed.assert_not_called()
        controller.start(created,100.)
        c.vehicle.setSpeed.assert_called_once_with(vehicles[0]['id'],-1.)
        controller.release(created,105.49)
        self.assertEqual(c.vehicle.setSpeed.call_count,1)
        controller.release(created,105.5)
        self.assertEqual(c.vehicle.setSpeed.call_count,2)
        controller.release(created,165.)
        self.assertEqual(c.vehicle.setSpeed.call_count,10)
        controller.release(created,100.+policy['release_offsets_seconds'][-1])
        self.assertEqual(c.vehicle.setSpeed.call_count,30)
        controller.reset();self.assertEqual(controller.snapshot(),dict(start_time=None,released=0,completed=0))

    def test_queue_crosses_upstream_lane_boundaries_without_compressing_spacing(self):
        vehicles,policy=load_merging_vehicles()
        meter=ZipperReleaseMeter(Mock(),policy['zipper_meter'])
        distances=[]
        for entry in vehicles:
            lane=entry.get('placement_lane_id',entry['route'][0]+'_'+str(entry['lane_index']))
            distances.append(meter.ramp_distance(lane,entry['position']))
        self.assertAlmostEqual(distances[0],63.67)
        self.assertAlmostEqual(distances[-1],376.87)
        for front,back in zip(distances,distances[1:]):
            self.assertAlmostEqual(back-front,10.8)

    def test_each_lane_change_waits_for_center_and_is_rate_limited(self):
        vehicles,policy=load_merging_vehicles();c=Mock()
        controller=RampMergeController(c,dict(policy,target_sumo_lane_index=3,zipper_meter=None))
        name=vehicles[0]['id'];created={name:dict(scenario=vehicles[0])}
        controller.start(created,0.)
        states={name:{80:'-47',81:'-47_1',184:0.}}
        controller.update_lanes(created,states,TC,1.)
        c.vehicle.changeLane.assert_called_once_with(name,2,5.)
        controller.update_lanes(created,states,TC,1.05)
        self.assertEqual(c.vehicle.changeLane.call_count,1)
        states[name]={80:'-47',81:'-47_2',184:-1.}
        controller.update_lanes(created,states,TC,2.)
        self.assertEqual(c.vehicle.changeLane.call_count,1)
        states[name][184]=0.
        controller.update_lanes(created,states,TC,3.)
        c.vehicle.changeLane.assert_called_with(name,3,5.)
        states[name]={80:'-47',81:'-47_3',184:0.}
        controller.update_lanes(created,states,TC,4.)
        self.assertEqual(controller.completed,{name})
        controller.update_lanes(created,states,TC,5.)
        self.assertEqual(c.vehicle.changeLane.call_count,2)
        c.vehicle.moveTo.assert_not_called()
        c.vehicle.setSpeedMode.assert_not_called()

    def test_lane_four_entry_returns_to_normal_route_selection_without_forced_left_crossing(self):
        vehicles,policy=load_merging_vehicles();c=Mock()
        controller=RampMergeController(c,dict(policy,zipper_meter=None))
        name=vehicles[0]['id'];created={name:dict(scenario=vehicles[0])}
        controller.start(created,0.)
        controller.update_lanes(created,{name:{80:'-47',81:'-47_1',184:0.}},TC,1.)
        self.assertEqual(controller.completed,{name})
        c.vehicle.setLaneChangeMode.assert_called_once_with(name,1621)
        c.vehicle.changeLane.assert_not_called()


if __name__=='__main__':unittest.main()
