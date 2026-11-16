"""堵车障碍的启动保持、无线隔离与静态 CARLA 同步回归。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from foundation.cobridge.npcvehicules.run_synchronization import SimulationSynchronization
from vanet.cobridge.mobility_snapshot import collect_mobility
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import FixedNpcTraffic, load_scenario
from vanet.npc.stationary_traffic_obstacles.highway_lane_queue import load_stationary_highway_vehicles


class StationaryHighwayQueueTests(unittest.TestCase):
    def test_start_releases_only_moving_traffic(self):
        config = load_scenario()
        static = load_stationary_highway_vehicles(config['vehicle_type_spec'])[0]
        moving = next(v for v in config['npcs'] if v['id'] == 'vanet_npc_001')
        config['npcs'] = [static, moving]
        connection = Mock()
        connection.vehicletype.getIDList.return_value = []
        connection.route.getIDList.return_value = []
        connection.vehicle.getIDList.return_value = [v['id'] for v in config['npcs']]
        traffic = FixedNpcTraffic(connection, config)
        traffic.spawn_next_vehicles(10)
        connection.vehicle.setSpeed.reset_mock()
        traffic.start(10.)
        speeds = {call.args[0]: call.args[1] for call in connection.vehicle.setSpeed.call_args_list}
        self.assertEqual(speeds, {static['id']: 0., moving['id']: -1.})
        connection.vehicle.setStop.assert_called_once()
        self.assertEqual(connection.vehicle.setStop.call_args.kwargs['flags'], 0)
        connection.vehicle.setLaneChangeMode.assert_called_once_with(static['id'], 0)

    def test_obstacles_are_not_sampled_as_radio_nodes_or_cooperative_npcs(self):
        vehicle = Mock()
        vehicle.getIDList.return_value = ['obstacle', 'moving']
        vehicle.getPosition3D.return_value = (10., 20., 0.)
        vehicle.getAngle.return_value = 90.
        vehicle.getLength.return_value = 4.
        vehicle.getWidth.return_value = 2.
        vehicle.getSpeed.return_value = 5.
        vehicle.getRoute.return_value = ['edge']
        traffic = SimpleNamespace(created={
            'obstacle': dict(scenario=dict(stationary=True, network_enabled=False)),
            'moving': dict(scenario={}),
        })
        bridge = SimpleNamespace(
            carla=SimpleNamespace(world=Mock()), lincoln=SimpleNamespace(records={}),
            sumo=SimpleNamespace(connection=SimpleNamespace(vehicle=vehicle), network=dict(offset=(0., 0.))),
            npc=SimpleNamespace(batches={'batch': dict(traffic=traffic)}),
        )
        states, nodes = collect_mobility(bridge, dict(id='rsu0', x=0., y=0., z=6.))
        self.assertEqual([v['id'] for v in states], ['moving'])
        self.assertEqual({v['id'] for v in nodes}, {'moving', 'rsu0'})
        self.assertTrue(all(call.args[0] == 'moving' for call in vehicle.mock_calls if call.args))

    def test_static_proxy_is_aligned_once_and_normal_npc_keeps_updating(self):
        sumo = Mock()
        sumo.spawned_actors = set()
        sumo.destroyed_actors = set()
        sumo.available_actors = {'obstacle', 'moving'}
        sumo.stationary_actors = {'obstacle'}
        sumo.get_actor.return_value = SimpleNamespace(transform=object(), extent=object(), signals=0)
        carla = Mock()
        carla.errors = {}
        sync = SimulationSynchronization(sumo, carla, Mock())
        sync.sumo2carla_ids = {'obstacle': 1, 'moving': 2}
        sync.synchronize_vehicles()
        sumo.spawned_actors = {'obstacle'}  # Delayed departure after direct junction placement.
        sync.synchronize_vehicles()
        actors = [call.args[0] for call in carla.synchronize_vehicle.call_args_list]
        self.assertEqual(actors, [1, 2, 2])
        carla.spawn_actor.assert_not_called()
        # A failed batch must not suppress the next attempt at initial alignment.
        sync.stationary_synchronized.clear()
        carla.flush_updates.side_effect = RuntimeError('batch rejected')
        with self.assertRaises(RuntimeError):
            sync.synchronize_vehicles()
        self.assertEqual(sync.stationary_synchronized, set())
        carla.flush_updates.side_effect = None
        sync.close()
        self.assertEqual(sync.stationary_synchronized, set())


if __name__ == '__main__':
    unittest.main()
