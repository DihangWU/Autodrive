"""独立 SUMO 检查 100 辆不联网车的出生、简单跟车、直行驶离和复位。"""
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from types import SimpleNamespace

from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController,load_scenario
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE
from foundation.cobridge.npcvehicules.vehicle_sync import NpcSumoAdapter
from foundation.cobridge.npcvehicules.official_tools import load_official_tools


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated real SUMO test only')
class NonNetworkHighwayTrafficLiveTests(unittest.TestCase):
    def test_simple_following_straight_exit_and_reset(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        import traci.constants as tc
        with tempfile.TemporaryDirectory(prefix='non_network_traffic_') as folder:
            with socket.socket() as listener:
                listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
            traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(SUMO_TOWN04_CONFIGURATION_FILE),
                         '--step-length','.05','--lateral-resolution','.25','--seed','20261116',
                         '--no-step-log','true','--error-log',str(Path(folder)/'errors.log')],
                        port=port,label='non_network_highway_test')
            c=traci.getConnection('non_network_highway_test')
            try:
                config=load_scenario()
                config['npcs']=[v for v in config['npcs'] if v.get('role')=='non_network_background']
                expected={v['id']:v for v in config['npcs']}
                self.assertEqual(len(expected),100)
                scenario=ScenarioController(c,config);scenario.reset()
                scenario.traffic.spawn_next_vehicles(scenario.traffic.count)
                for _ in range(3):c.simulationStep()
                self.assertEqual(set(c.vehicle.getIDList()),set(expected))
                self.assertFalse(c.simulation.getPendingVehicles())
                origins={name:c.vehicle.getPosition(name) for name in expected}
                adapter=NpcSumoAdapter(SimpleNamespace(connection=c,network=dict(offset=(0.,0.))),
                                       scenario.traffic,load_official_tools())
                adapter.prepare_frame(set(expected),{},set(expected),set(),set(),set())
                for name,entry in expected.items():
                    self.assertEqual(c.vehicle.getLaneID(name),entry['initial_placement']['lane_id'])
                    self.assertEqual(c.vehicle.getColor(name),tuple(entry['color']))
                    self.assertEqual(c.vehicle.getSpeed(name),0.)
                    self.assertFalse(c.vehicle.isStopped(name))
                    self.assertEqual(c.vehicle.getSpeedMode(name),31)
                    self.assertAlmostEqual(c.vehicle.getImperfection(name),entry['driver_imperfection'])
                    self.assertAlmostEqual(c.vehicle.getMaxSpeed(name),entry['maximum_speed_kmh']/3.6)
                    adapter.subscribe(name)
                    subscription=c.vehicle.getSubscriptionResults(name)
                    self.assertEqual(set(subscription),{tc.VAR_POSITION3D,tc.VAR_ANGLE,tc.VAR_SLOPE,tc.VAR_SIGNALS})
                    adapter.states[name]=json.loads(json.dumps(subscription))
                    self.assertIsNotNone(adapter.get_actor(name))
                    self.assertIn(name,adapter.subscribed)
                    self.assertNotIn(name,adapter.stationary_actors)
                    # Observation-only fields for this test, not production subscriptions.
                    c.vehicle.subscribe(name,[tc.VAR_POSITION,tc.VAR_SPEED,tc.VAR_ROAD_ID,tc.VAR_LANE_ID])
                scenario.start(c.simulation.getTime())
                self.assertTrue(all(c.vehicle.getLaneChangeMode(name)==512 for name in expected))
                collisions=set();teleports=set();arrivals=set();exits=set();travel={name:0. for name in expected}
                allowed={3:{'-41_3',':134_10_2','-46_3',':1061_2_3','-47_3',':764_3_2',
                            '-48_3',':890_3_1','-49_3',':1159_10_3','-50_3'},
                         4:{'-41_4',':134_10_3','-46_4',':1061_2_4','-47_4',':764_3_3',
                            '-48_4',':890_3_2','-49_4',':1159_10_4','-50_4'}}
                for _ in range(6000):
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    teleports.update(c.simulation.getStartingTeleportIDList())
                    arrivals.update(c.simulation.getArrivedIDList())
                    for name,state in c.vehicle.getAllSubscriptionResults().items():
                        entry=expected[name]
                        self.assertIn(state[tc.VAR_LANE_ID],allowed[entry['lane_index']])
                        self.assertLessEqual(state[tc.VAR_SPEED],entry['maximum_speed_kmh']/3.6+1e-6)
                        travel[name]=max(travel[name],math.dist(origins[name],state[tc.VAR_POSITION]))
                        if state[tc.VAR_ROAD_ID]=='-50':exits.add(name)
                    if arrivals==set(expected):break
                self.assertEqual(collisions,set());self.assertEqual(teleports,set())
                self.assertGreater(min(travel.values()),150.)
                self.assertEqual(exits,set(expected));self.assertEqual(arrivals,set(expected))
                scenario.reset();scenario.traffic.spawn_next_vehicles(scenario.traffic.count)
                for _ in range(3):c.simulationStep()
                self.assertEqual(set(c.vehicle.getIDList()),set(expected))
                self.assertTrue(all(math.dist(origins[n],c.vehicle.getPosition(n))<1e-6 for n in expected))
                self.assertTrue(all(c.vehicle.getSpeed(n)==0. for n in expected))
                print('Non-network traffic: 100 cars / 50 per lane, all moving and straight exits, '
                      'fixed lanes, individual speed caps respected, collisions=0, teleports=0, reset restored')
            finally:
                c.close()


if __name__=='__main__':unittest.main()
