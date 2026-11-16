"""独立 SUMO 检查两列初始车距、正常驾驶、80 km/h 上限及限速恢复。"""
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace
import unittest

from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE,SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController
from vanet.cobridge.mobility_snapshot import collect_mobility


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated real SUMO test only')
class HighwayBackgroundTrafficLiveTests(unittest.TestCase):
    def test_placement_speed_normal_driving_and_network_membership(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        import traci.constants as tc
        import sumolib
        c=None
        with tempfile.TemporaryDirectory(prefix='highway_background_') as directory:
            errors=Path(directory)/'errors.log'
            with socket.socket() as listener:
                listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
            try:
                traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(SUMO_TOWN04_CONFIGURATION_FILE),
                    '--step-length','.05','--lateral-resolution','.25','--no-step-log','true',
                    '--error-log',str(errors)],port=port,label='highway_background_test')
                c=traci.getConnection('highway_background_test')
                scenario=ScenarioController(c);scenario.reset()
                original_limits={lane:c.lane.getMaxSpeed(lane) for lane in scenario.config['highway_speed_limits']['lanes']}
                scenario.traffic.spawn_next_vehicles(scenario.traffic.count)
                for _ in range(3):c.simulationStep()
                expected={v['id']:v for v in scenario.config['npcs'] if v.get('yielding_enabled') is False}
                self.assertEqual(len(expected),20)
                present=set(c.vehicle.getIDList())
                self.assertTrue(set(expected)<=present)
                self.assertFalse(c.simulation.getPendingVehicles())
                origins={name:c.vehicle.getPosition(name) for name in expected}
                for name,entry in expected.items():
                    self.assertEqual(c.vehicle.getLaneID(name),entry['initial_placement']['lane_id'])
                    self.assertEqual(c.vehicle.getSpeed(name),0.)
                    self.assertEqual(c.vehicle.getLaneChangeMode(name),0)
                    self.assertAlmostEqual(c.vehicle.getMaxSpeed(name),80./3.6)
                    self.assertEqual(c.vehicle.getSpeedMode(name),31)
                    self.assertEqual(c.vehicle.getParameter(name,'vanet.scenario.yielding_enabled'),'false')
                for lane in original_limits:self.assertAlmostEqual(c.lane.getMaxSpeed(lane),100./3.6)
                # The static cars remain excluded, while all twenty new radios
                # carry their per-vehicle no-yield policy to the coordinator.
                bridge=SimpleNamespace(carla=SimpleNamespace(world=SimpleNamespace(get_snapshot=lambda:None)),
                    lincoln=SimpleNamespace(records={}),sumo=SimpleNamespace(connection=c,network=dict(offset=(503.02,423.76))),
                    npc=SimpleNamespace(batches={'scene':dict(traffic=scenario.traffic)}))
                states,nodes=collect_mobility(bridge,dict(id='rsu0',x=193.,y=-179.,z=6.))
                self.assertTrue(set(expected)<={v['id'] for v in nodes})
                self.assertTrue(all(not v['yielding_enabled'] for v in states if v['id'] in expected))
                # Check actual geometric pitch, including the connecting lanes.
                offsets={}
                for lane_index in (1,2):
                    lanes=[f'-41_{lane_index}',f':134_10_{lane_index-1}',f'-46_{lane_index}',
                           f':1061_2_{lane_index}',f'-47_{lane_index}',f':764_3_{lane_index-1}',f'-48_{lane_index}']
                    starts={};station=0.
                    for lane in lanes:
                        starts[lane]=station;station+=sumolib.geomhelper.polyLength(c.lane.getShape(lane))
                    reference_point=json.loads(SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text())['placement_reference']['sumo_body_center']
                    reference,_=sumolib.geomhelper.polygonOffsetAndDistanceToPoint(reference_point,c.lane.getShape(lanes[0]))
                    for name,entry in expected.items():
                        if entry['lane_index']!=lane_index:continue
                        lane=c.vehicle.getLaneID(name)
                        front,_=sumolib.geomhelper.polygonOffsetAndDistanceToPoint(c.vehicle.getPosition(name),c.lane.getShape(lane))
                        actual=starts[lane]+front-c.vehicle.getLength(name)/2-reference
                        offsets[name]=actual
                        self.assertAlmostEqual(actual,entry['initial_placement']['corridor_distance_metres'],places=3)
                scenario.start(c.simulation.getTime())
                self.assertTrue(all(c.vehicle.getLaneChangeMode(name)==1621 for name in expected))
                first=next(v for v in expected.values() if v['id']=='vanet_npc_highway_lane4_01')
                lane_shape=c.lane.getShape('-41_1')
                first_station,_=sumolib.geomhelper.polygonOffsetAndDistanceToPoint(origins[first['id']],lane_shape)
                self.assertAlmostEqual(first_station-c.vehicle.getLength(first['id'])/2-297.43979019781625,180.,places=3)
                for name in expected:c.vehicle.subscribe(name,[tc.VAR_ROAD_ID,tc.VAR_LANE_ID,tc.VAR_LANEPOSITION])
                peak_speed=0.;collisions=set();travel={name:0. for name in expected}
                merge_space_seen=False
                branches={};lane3_entered_city=set();lane4_merge_lanes={}
                for index in range(2400):
                    scenario.step(index*.05)
                    states=c.vehicle.getAllSubscriptionResults()
                    scenario.traffic.maintain_routes(states,tc)
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    for name in set(c.vehicle.getIDList())&expected.keys():
                        road=c.vehicle.getRoadID(name);lane=c.vehicle.getLaneID(name)
                        if road=='-49':branches[name]='straight'
                        elif road=='-7':
                            branches[name]='city'
                            if expected[name]['lane_index']==2:lane3_entered_city.add(name)
                        if lane=='-48_2' and expected[name]['lane_index']==1:lane4_merge_lanes[name]=lane
                        speed=c.vehicle.getSpeed(name);peak_speed=max(peak_speed,speed)
                        self.assertLessEqual(speed,80./3.6+1e-6)
                        travel[name]=max(travel[name],math.dist(origins[name],c.vehicle.getPosition(name)))
                        if c.vehicle.getLaneID(name)=='-41_1' and c.vehicle.couldChangeLane(name,1):merge_space_seen=True
                self.assertEqual(collisions,set())
                self.assertGreater(min(travel.values()),30.)
                self.assertGreater(peak_speed,75./3.6)
                self.assertTrue(merge_space_seen)
                self.assertEqual(set(branches),set(expected))
                self.assertEqual(lane3_entered_city,set())
                self.assertEqual(sum(branch=='straight' for branch in branches.values()),15)
                self.assertEqual(sum(branch=='city' for branch in branches.values()),5)
                decisions=scenario.traffic.junction_split.decisions
                self.assertEqual(len(decisions),10)
                self.assertEqual([v['branch'] for v in decisions],['straight','city']*5)
                for decision in decisions:
                    self.assertEqual(branches[decision['vehicle']],decision['branch'])
                    if decision['branch']=='straight':self.assertIn(decision['vehicle'],lane4_merge_lanes)
                scenario.traffic.remove_vehicles()
                for lane,speed in original_limits.items():self.assertAlmostEqual(c.lane.getMaxSpeed(lane),speed)
                self.assertEqual(errors.read_text(),'')
                print(json.dumps(dict(networked_added=20,per_lane=10,same_lane_pitch_metres=70.,stagger_metres=35.,
                    peak_speed_kmh=round(peak_speed*3.6,3),minimum_travel_metres=round(min(travel.values()),3),
                    lane_change_space_observed=merge_space_seen,collisions=0,road_limits_restored=True,
                    initial_gap_from_lincoln_metres=180.,straight_departures=15,city_departures=5,
                    lane4_alternating_order=[v['vehicle']+':'+v['branch'] for v in decisions])))
            finally:
                if c is not None:c.close()


if __name__=='__main__':unittest.main()
