"""独立 SUMO：完整主路车流中的十车汇入、逐道并线与 Lincoln 路过。"""
import json
import math
import os
import re
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace
import unittest

from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController
from vanet.cobridge.mobility_snapshot import collect_mobility
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE
from vanet.npc.highway_ramp_merging.zipper_network import prepare_zipper_configuration


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated real SUMO test only')
class HighwayRampMergingLiveTests(unittest.TestCase):
    def test_ten_networked_cars_merge_safely_and_overlap_main_traffic(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        import traci.constants as tc
        with tempfile.TemporaryDirectory(prefix='highway_ramp_') as folder:
            configuration=prepare_zipper_configuration(Path(folder)/'map')
            with socket.socket() as listener:
                listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
            traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(configuration),
                         '--step-length','.05','--lateral-resolution','.25','--seed',os.environ.get('VANET_SUMO_TEST_SEED','20261104'),
                         '--collision.check-junctions','true',
                         '--no-step-log','true','--error-log',str(Path(folder)/'errors.log')],
                        port=port,label='highway_ramp_test')
            c=traci.getConnection('highway_ramp_test')
            try:
                scene=ScenarioController(c);scene.reset();scene.traffic.spawn_next_vehicles(scene.traffic.count)
                for _ in range(3):c.simulationStep()
                self.assertEqual(len(c.vehicle.getIDList()),scene.traffic.initial_count)
                self.assertFalse(c.simulation.getPendingVehicles())
                ramp={v['id']:v for v in scene.config['npcs'] if v.get('role')=='ramp_merging'}
                self.assertEqual(len(ramp),10)
                for name,entry in ramp.items():
                    self.assertEqual(c.vehicle.getSpeed(name),0.)
                    self.assertEqual(c.vehicle.getLaneID(name),entry.get('placement_lane_id','31_1'))
                    self.assertEqual(c.vehicle.getSpeedMode(name),31)
                bridge=SimpleNamespace(carla=SimpleNamespace(world=SimpleNamespace(get_snapshot=lambda:None)),
                    lincoln=SimpleNamespace(records={}),sumo=SimpleNamespace(connection=c,network=dict(offset=(503.02,423.76))),
                    npc=SimpleNamespace(batches={'scene':dict(traffic=scene.traffic)}))
                states,nodes=collect_mobility(bridge,dict(id='rsu0',x=193.,y=-179.,z=6.))
                self.assertTrue(set(ramp)<={v['id'] for v in nodes})
                self.assertTrue(all(not v['yielding_enabled'] for v in states if v['id'] in ramp))
                # A native SUMO proxy checks timing/traffic interaction; it does
                # not replace acceptance with the actual CARLA Lincoln planner.
                c.route.add('lincoln_probe_route',['-41','-46','-47','-48','-7','-8','-9','-18','-19'])
                c.vehicle.add('lincoln_probe','lincoln_probe_route',departLane='1',departPos='293.47',departSpeed='0')
                c.vehicle.setMaxSpeed('lincoln_probe',100./3.6)
                fields=[tc.VAR_ROAD_ID,tc.VAR_EDGES,tc.VAR_LANEPOSITION,tc.VAR_LANE_ID,
                        tc.VAR_LANEPOSITION_LAT,tc.VAR_POSITION,tc.VAR_SPEED]
                c.vehicle.subscribe('lincoln_probe',fields)
                for name in c.vehicle.getIDList():
                    if name.startswith('vanet_npc_'):c.vehicle.subscribe(name,fields)
                scene.start(0.)
                collisions=set();teleports=set();entries={};targets={};arrivals=set();passage=None
                entry_lanes={};straight_exits=set();curves=set()
                previous={};maximum_step=0.;nearby_active_merge=False
                crossings=[];seen_crossings=set();main_cross_speeds=[]
                flowing_main=set();stop_run={};longest_main_stop=0.
                previous_main_speeds={};maximum_main_braking=0.
                for index in range(5000):
                    now=index*.05;states=c.vehicle.getAllSubscriptionResults()
                    probe=states.get('lincoln_probe')
                    if probe and probe[tc.VAR_ROAD_ID]=='-47' and passage is None:passage=now
                    scene.step(now);scene.traffic.spawn_next_vehicles(20);scene.traffic.maintain_routes(states,tc)
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    teleports.update(c.simulation.getStartingTeleportIDList())
                    arrivals.update(set(c.simulation.getArrivedIDList())&ramp.keys())
                    for name,state in c.vehicle.getAllSubscriptionResults().items():
                        if name in ramp or name.startswith('vanet_npc_highway_lane4_'):
                            policy=scene.traffic.ramp_merging.meter.configuration
                            is_ramp=name in ramp
                            line=policy['ramp_conflict_position_metres' if is_ramp else 'main_conflict_position_metres']
                            lane=policy['ramp_internal_lane' if is_ramp else 'counting_lane']
                            if name not in seen_crossings and state[tc.VAR_LANE_ID]==lane and state[tc.VAR_LANEPOSITION]>=line:
                                seen_crossings.add(name)
                                crossings.append(dict(time=now,vehicle=name,stream='ramp' if is_ramp else 'main',speed_kmh=state[tc.VAR_SPEED]*3.6))
                                if not is_ramp and len(main_cross_speeds)<10:main_cross_speeds.append(state[tc.VAR_SPEED]*3.6)
                        if name.startswith('vanet_npc_highway_lane4_'):
                            lane=state[tc.VAR_LANE_ID];speed=state[tc.VAR_SPEED]
                            in_zone=lane in ('-46_1',':1061_2_1')
                            if in_zone and name in previous_main_speeds:
                                maximum_main_braking=max(maximum_main_braking,(previous_main_speeds[name]-speed)/.05)
                            if in_zone:previous_main_speeds[name]=speed
                            else:previous_main_speeds.pop(name,None)
                            if in_zone and speed>2.:flowing_main.add(name)
                            if name in flowing_main and in_zone and speed<.1:
                                stop_run[name]=stop_run.get(name,0.)+.05
                                longest_main_stop=max(longest_main_stop,stop_run[name])
                            else:stop_run[name]=0.
                        if name not in ramp:continue
                        position=state[tc.VAR_POSITION]
                        if name in previous:maximum_step=max(maximum_step,math.dist(previous[name],position))
                        previous[name]=position
                        if state[tc.VAR_ROAD_ID]==':1061_0':curves.add(name)
                        if state[tc.VAR_ROAD_ID]=='-47':
                            entries.setdefault(name,now);entry_lanes.setdefault(name,state[tc.VAR_LANE_ID])
                        if state[tc.VAR_ROAD_ID]=='-49':straight_exits.add(name)
                        self.assertNotEqual(state[tc.VAR_ROAD_ID],'-7')
                        if state[tc.VAR_ROAD_ID]=='-47' and state[tc.VAR_LANE_ID]=='-47_1':
                            targets.setdefault(name,now)
                        if (probe and probe[tc.VAR_ROAD_ID] in ('-46',':1061_2','-47')
                                and name not in scene.traffic.ramp_merging.completed
                                and math.dist(probe[tc.VAR_POSITION],position)<50.):
                            nearby_active_merge=True
                    if arrivals==set(ramp):break
                report=dict(main_entry_times=entries,target_lane_times=targets,lincoln_probe_passage=passage,
                            nearby_active_merge=nearby_active_merge,maximum_step_metres=maximum_step,
                            collisions=sorted(collisions),teleports=sorted(teleports),
                            policy=scene.traffic.ramp_merging.snapshot())
                report['zipper_meter']=scene.traffic.ramp_merging.meter.snapshot()
                report['crossings']=crossings
                report['minimum_first_ten_main_cross_speed_kmh']=min(main_cross_speeds)
                report['longest_main_merge_stop_seconds']=longest_main_stop
                report['maximum_main_merge_braking_mps2']=maximum_main_braking
                print(json.dumps(report),flush=True)
                self.assertEqual(collisions,set());self.assertEqual(teleports,set())
                self.assertEqual(set(entries),set(ramp));self.assertEqual(set(targets),set(ramp))
                self.assertEqual(set(entry_lanes.values()),{'-47_1'})
                self.assertEqual(straight_exits,set(ramp));self.assertEqual(curves,set(ramp))
                self.assertEqual(scene.traffic.ramp_merging.completed,set(ramp))
                self.assertEqual(arrivals,set(ramp));self.assertLess(maximum_step,2.)
                self.assertEqual([entry['stream'] for entry in crossings[:20]],['main','ramp']*10)
                self.assertGreater(min(main_cross_speeds),60.)
                self.assertLess(longest_main_stop,.5)
                self.assertLessEqual(maximum_main_braking,4.51)
                errors=(Path(folder)/'errors.log').read_text()
                self.assertIsNone(re.search(r"Vehicle 'vanet_npc_highway_lane4_\d+'.*emergency braking on lane '(?:-46_1|:1061_2_1)'",errors))
                self.assertIsNotNone(passage)
                self.assertGreaterEqual(sum(t<passage for t in entries.values()),2)
                releases=scene.traffic.ramp_merging.meter.events
                self.assertEqual(len(releases),10)
                self.assertEqual([event['required_main_vehicles'] for event in releases],[1]*10)
                self.assertEqual([event['main_vehicle'] for event in releases],
                                 [entry['vehicle'] for entry in crossings[:20] if entry['stream']=='main'])
                scene.reset();scene.traffic.spawn_next_vehicles(scene.traffic.count)
                for _ in range(3):c.simulationStep()
                self.assertEqual(scene.traffic.ramp_merging.snapshot(),dict(start_time=None,released=0,completed=0))
                self.assertTrue(all(c.vehicle.getSpeed(name)==0. for name in ramp))
                self.assertAlmostEqual(c.lane.getMaxSpeed('31_1'),13.89)
                self.assertAlmostEqual(c.lane.getMaxSpeed(':1061_0_1'),13.89)
                self.assertTrue(all(c.vehicle.getTau(name)==1. for name in ramp))
            finally:
                c.close()


if __name__=='__main__':unittest.main()
