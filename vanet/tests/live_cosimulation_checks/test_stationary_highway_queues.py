"""独立真实 SUMO：检查堵车车队出生、长期静止和复位，不连接当前联仿。"""
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest

from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController, load_scenario
from vanet.npc.stationary_traffic_obstacles.highway_lane_queue import load_stationary_highway_vehicles
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY, SUMO_TOWN04_CONFIGURATION_FILE


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO') == '1', 'Explicit isolated SUMO test only')
class StationaryHighwayQueueLiveTests(unittest.TestCase):
    def test_dense_queues_remain_immobile_past_teleport_timeout(self):
        sys.path.insert(0, str(SUMO_INSTALLATION_DIRECTORY / 'tools'))
        import traci
        connection = None
        with tempfile.TemporaryDirectory(prefix='stationary_highway_') as folder:
            error_log = Path(folder) / 'errors.log'
            with socket.socket() as listener:
                listener.bind(('127.0.0.1', 0))
                port = listener.getsockname()[1]
            try:
                traci.start([str(SUMO_INSTALLATION_DIRECTORY / 'bin/sumo'),
                    '-c', str(SUMO_TOWN04_CONFIGURATION_FILE), '--step-length', '.05',
                    '--lateral-resolution', '.25', '--no-step-log', 'true',
                    '--error-log', str(error_log)], port=port, label='stationary_highway_test')
                connection = traci.getConnection('stationary_highway_test')
                config = load_scenario()
                # Legacy static mode remains available, but is no longer the default scene.
                config['npcs'] = load_stationary_highway_vehicles(config['vehicle_type_spec'])
                expected = {v['id']: v for v in config['npcs']}
                scenario = ScenarioController(connection, config)
                scenario.reset()
                scenario.traffic.spawn_next_vehicles(scenario.traffic.count)
                for _ in range(3):
                    connection.simulationStep()
                self.assertEqual(set(connection.vehicle.getIDList()), set(expected))
                self.assertEqual(connection.simulation.getPendingVehicles(), ())
                original = {name: connection.vehicle.getPosition(name) for name in expected}
                for name, entry in expected.items():
                    lane = entry.get('placement_lane_id',f"{entry['route'][0]}_{entry['lane_index']}")
                    self.assertEqual(connection.vehicle.getLaneID(name), lane)
                    self.assertTrue(connection.lane.getAllowed(lane) == () or
                                    'passenger' in connection.lane.getAllowed(lane))
                    self.assertEqual(connection.vehicle.getColor(name), tuple(entry['color']))
                    self.assertEqual(connection.vehicle.getSpeed(name), 0.)
                    self.assertTrue(connection.vehicle.isStopped(name))
                    self.assertLessEqual(entry['position'], connection.lane.getLength(lane) - 3 + 1e-6)
                    if lane.startswith(':'):
                        links = connection.lane.getLinks(f"{entry['route'][0]}_{entry['lane_index']}")
                        self.assertTrue(any(link[4] == lane for link in links))
                # Exercise the actual SUMO adapter's one-time subscription path.
                from types import SimpleNamespace
                from foundation.cobridge.npcvehicules.official_tools import load_official_tools
                from foundation.cobridge.npcvehicules.vehicle_sync import NpcSumoAdapter
                tools = load_official_tools()
                adapter = NpcSumoAdapter(SimpleNamespace(connection=connection,network=dict(offset=(0.,0.))),
                                         scenario.traffic,tools)
                adapter.prepare_frame(set(expected),{},set(),set(),set(),set())
                for name in expected:
                    adapter.subscribe(name)
                    # RemoteConnection normally converts variable keys through JSON.
                    adapter.states[name] = json.loads(json.dumps(connection.vehicle.getSubscriptionResults(name)))
                    self.assertIsNotNone(adapter.get_actor(name))
                    self.assertIsNotNone(adapter.get_actor(name))
                connection.simulationStep()  # TraCI clears its local result cache at the next step.
                self.assertEqual(len(connection.vehicle.getAllSubscriptionResults()),0)
                adapter.prepare_frame(set(expected),{},set(),set(),set(),set())
                self.assertEqual(adapter.spawned_actors,set())
                scenario.start(connection.simulation.getTime())
                teleports, collisions = set(), set()
                for _ in range(6200):
                    connection.simulationStep()
                    teleports.update(connection.simulation.getStartingTeleportIDList())
                    collisions.update(connection.simulation.getCollidingVehiclesIDList())
                self.assertEqual(set(connection.vehicle.getIDList()), set(expected))
                self.assertEqual(teleports, set())
                self.assertEqual(collisions, set())
                maximum_drift = max(math.dist(original[name], connection.vehicle.getPosition(name))
                                    for name in expected)
                self.assertLess(maximum_drift, 1e-6)
                scenario.reset()
                scenario.traffic.spawn_next_vehicles(scenario.traffic.count)
                for _ in range(3):
                    connection.simulationStep()
                self.assertEqual(set(connection.vehicle.getIDList()), set(expected))
                self.assertTrue(all(connection.vehicle.getSpeed(name) == 0. for name in expected))
                self.assertEqual(error_log.read_text(), '')
                print(json.dumps(dict(stationary_vehicles=len(expected), seconds=310,
                    maximum_drift_metres=maximum_drift, collisions=len(collisions),
                    teleports=len(teleports), reset_restored=True)))
            finally:
                if connection is not None:
                    connection.close()


if __name__ == '__main__':
    unittest.main()
