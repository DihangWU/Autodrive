"""不联网动态车流：密度、随机性、原有车流隔离与简单驾驶配置。"""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.cobridge.mobility_snapshot import collect_mobility
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import load_scenario,FixedNpcTraffic
from vanet.npc.highway_background_traffic.load_staggered_vehicles import load_staggered_traffic
from vanet.npc.non_network_highway_traffic.load_background_vehicles import load_background_vehicles
from vanet.project_paths import SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE
from foundation.cobridge.npcvehicules.vehicle_sync import NpcSumoAdapter
from vanet.npc.highway_background_traffic.right_turn_queueing import RightTurnQueueing


class NonNetworkHighwayTrafficTests(unittest.TestCase):
    def test_density_spacing_straight_routes_and_original_forty_unchanged(self):
        config=load_scenario()
        traffic=[v for v in config['npcs'] if v.get('role')=='non_network_background']
        self.assertEqual(len(traffic),100)
        self.assertFalse(any(v.get('stationary') for v in config['npcs']))
        self.assertEqual(sum(v.get('network_enabled',True) for v in config['npcs']),74)
        pitch=json.loads(SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text())['placement_reference']['same_lane_pitch_metres']
        for lane in (3,4):
            group=[v for v in traffic if v['lane_index']==lane]
            self.assertEqual(len(group),50)
            stations=[v['initial_placement']['corridor_distance_metres'] for v in group]
            gaps=[b-a for a,b in zip(stations,stations[1:])]
            self.assertAlmostEqual(pitch/(sum(gaps)/len(gaps)),1.5,delta=.02)
            self.assertGreater(min(gaps),float(config['vehicle_type_spec']['length'])+3.)
            self.assertGreater(max(gaps)-min(gaps),10.)
        self.assertGreater(len({v['maximum_speed_kmh'] for v in traffic}),90)
        for entry in traffic:
            self.assertEqual(entry['route'][-3:],['-48','-49','-50'])
            self.assertNotIn('-7',entry['route'])
            self.assertFalse(entry['network_enabled']);self.assertFalse(entry['yielding_enabled'])
            self.assertEqual(entry['speed_mode'],31);self.assertEqual(entry['lane_change_mode'],512)
            self.assertNotIn('stop_duration_seconds',entry)
        original,_=load_staggered_traffic()
        actual=[v for v in config['npcs'] if v['id'].startswith('vanet_npc_highway_')]
        self.assertEqual(actual,original)
        self.assertEqual(traffic,load_background_vehicles(config['vehicle_type_spec']))

    def test_release_on_start_no_stop_and_no_wireless_sampling(self):
        config=load_scenario()
        config['npcs']=[v for v in config['npcs'] if v.get('role')=='non_network_background'][:2]
        c=Mock();c.vehicletype.getIDList.return_value=[];c.route.getIDList.return_value=[]
        c.vehicle.getIDList.return_value=[v['id'] for v in config['npcs']]
        traffic=FixedNpcTraffic(c,config);traffic.spawn_next_vehicles(2)
        c.vehicle.setStop.assert_not_called()
        self.assertTrue(all(call.args[1]==0. for call in c.vehicle.setSpeed.call_args_list))
        for entry in config['npcs']:
            c.vehicle.setImperfection.assert_any_call(entry['id'],entry['driver_imperfection'])
        c.vehicle.setSpeed.reset_mock();traffic.start(0.)
        self.assertTrue(all(call.args[1]==-1. for call in c.vehicle.setSpeed.call_args_list))
        self.assertTrue(all(call.args[1]==512 for call in c.vehicle.setLaneChangeMode.call_args_list[-2:]))
        c.vehicle.reset_mock()
        c.vehicle.getIDList.return_value=list(traffic.created)
        bridge=SimpleNamespace(carla=SimpleNamespace(world=Mock()),lincoln=SimpleNamespace(records={}),
            sumo=SimpleNamespace(connection=c,network=dict(offset=(0.,0.))),
            npc=SimpleNamespace(batches={'scene':dict(traffic=traffic)}))
        states,nodes=collect_mobility(bridge,dict(id='rsu0',x=0.,y=0.,z=6.))
        self.assertEqual(states,[]);self.assertEqual([n['id'] for n in nodes],['rsu0'])
        self.assertEqual([call[0] for call in c.vehicle.mock_calls],['getIDList'])

    def test_lightweight_subscription_and_no_intersection_management(self):
        tc=SimpleNamespace(VAR_POSITION3D=57,VAR_ANGLE=67,VAR_SLOPE=54,VAR_SIGNALS=91,
                           VAR_ROUTE_INDEX=105,VAR_ROAD_ID=80,VAR_EDGES=84,VAR_LANEPOSITION=86)
        c=Mock();c.vehicle.getParameter.return_value='scene'
        traffic=SimpleNamespace(batch_id='scene',created={
            'background':dict(scenario=dict(role='non_network_background')),
            'networked':dict(scenario={})})
        adapter=NpcSumoAdapter(SimpleNamespace(connection=c),traffic,SimpleNamespace(constants=tc))
        adapter.subscribe('background');adapter.subscribe('networked')
        c.vehicle.subscribe.assert_any_call('background',[57,67,54,91])
        c.vehicle.subscribe.assert_any_call('networked',[57,67,54,91,105,80,84,86])
        policy=RightTurnQueueing(c,dict(approach_edge='-48',exit_edge='-7',internal_edges=[':890_2']))
        c.vehicle.reset_mock()
        policy.update({'background':traffic.created['background']},{'background':{80:'-48'}},tc,0.)
        self.assertEqual(c.vehicle.mock_calls,[])


if __name__=='__main__':unittest.main()
