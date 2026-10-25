"""无仿真器的登记/归属回归测试。"""
from contextlib import nullcontext
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from foundation.cobridge.lincoln.vehicle_registry import LincolnVehicleRegistry
from modules.lincoln.driving.controller import DrivingController, load_driving_controller
from modules.lincoln.driving.carla_autopilot import CarlaAutopilotController


class VehicleRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.registry = LincolnVehicleRegistry()
        self.owner = object()
        self.command = dict(action='lincoln.register', actor_id=10, world_id=20,
                            role_name='lincoln_1234567890')

    def test_two_vehicles_get_unique_sumo_names(self):
        first = self.registry.handle_request(self.command, self.owner)
        second = self.registry.handle_request(dict(self.command, actor_id=11), object())
        self.assertEqual((first['sumo_id'], second['sumo_id']), ('lincoln0', 'lincoln1'))

    def test_disconnect_retires_only_owned_proxy(self):
        self.registry.handle_request(self.command, self.owner)
        other = object()
        self.registry.handle_request(dict(self.command, actor_id=11), other)
        self.registry.records['20:10']['projected'] = True
        self.registry.disconnected(self.owner)
        self.assertEqual(self.registry.retired, {'lincoln0'})
        self.assertEqual(len(self.registry.records), 1)
        self.assertIs(self.registry.records['20:11']['owner'], other)

    def test_reconnect_transfers_registration_without_old_disconnect_deleting_it(self):
        self.registry.handle_request(self.command, self.owner)
        replacement = object()
        self.registry.handle_request(self.command, replacement)
        self.registry.disconnected(self.owner)
        self.assertEqual(len(self.registry.records), 1)

    def test_wrong_owner_cannot_unregister(self):
        self.registry.handle_request(self.command, self.owner)
        with self.assertRaises(ValueError):
            self.registry.handle_request(dict(self.command, action='lincoln.unregister'), object())
        self.assertEqual(len(self.registry.records), 1)

    def test_missing_actor_never_generates_sumo_proxy(self):
        self.registry.handle_request(self.command, self.owner)
        carla = SimpleNamespace(world=SimpleNamespace(id=20, get_actor=lambda _: None))
        with self.assertRaises(ValueError):
            self.registry.synchronize_one_vehicle('20:10', self.registry.records['20:10'], carla, Mock())

    def test_mismatched_world_never_uses_recycled_actor_id(self):
        self.registry.handle_request(self.command, self.owner)
        world = Mock(id=21)
        with self.assertRaises(ValueError):
            self.registry.synchronize_one_vehicle('20:10', self.registry.records['20:10'], SimpleNamespace(world=world), Mock())
        world.get_actor.assert_not_called()

    def test_lost_sumo_waits_without_deleting_carla(self):
        self.registry.handle_request(self.command, self.owner)
        self.registry.synchronize_before_sumo_step(Mock(), None)
        self.assertEqual(self.registry.snapshot()[0]['state'], 'waiting')
        self.assertEqual(len(self.registry.records), 1)

    def test_custom_driver_must_implement_interface(self):
        with patch('importlib.import_module', return_value=SimpleNamespace(Bad=object)):
            with self.assertRaises(TypeError):
                load_driving_controller('fake:Bad', None, None, None, None)


    def test_tm_readiness_requires_bridge_connection_even_for_manual_car(self):
        command = dict(self.command, tm_port=8000)
        status = self.registry.handle_request(command, self.owner)
        self.assertFalse(status['tm_ready'])
        self.registry.traffic_managers.carla_adapter = SimpleNamespace(world=SimpleNamespace(id=20))
        self.registry.traffic_managers.managers[8000] = object()
        self.assertTrue(self.registry.handle_request(dict(command, action='lincoln.status'), self.owner)['tm_ready'])

    def test_gta_command_round_trip_uses_registered_owner_and_driver_ack(self):
        from modules.lincoln.driving.navigation import LincolnNavigation
        node = SimpleNamespace(driver=Mock(), autopilot=False)
        navigation = LincolnNavigation(node)
        command = dict(self.command, navigation_supported=True, gta_supported=True,
                       gta_status=navigation.gta_snapshot())
        self.registry.handle_request(command, self.owner)
        record = self.registry.records['20:10']
        request = dict(id='mode-on', enabled=True)
        record['gta_request'] = request
        status = self.registry.handle_request(dict(command, action='lincoln.status'), self.owner)
        navigation.submit_gta_mode(status['gta_request'])
        self.assertFalse(record['gta_status']['enabled'])
        navigation.update(None)
        status_command = dict(command, action='lincoln.status', gta_status=navigation.gta_snapshot())
        with self.assertRaises(ValueError):
            self.registry.handle_request(status_command, object())
        self.registry.handle_request(status_command, self.owner)
        self.assertTrue(record['gta_status']['enabled'])
        self.assertEqual(record['gta_status']['request_id'], 'mode-on')
        # 桥重新登记时接收车辆实际状态；新车辆仍默认关闭。
        self.registry.handle_request(dict(status_command, action='lincoln.register'), self.owner)
        self.assertTrue(record['gta_status']['enabled'])
        self.registry.handle_request(dict(self.command, actor_id=11), object())
        self.assertNotIn('gta_request', self.registry.records['20:11'])

    def test_tm_is_prepared_while_vehicle_remains_manual(self):
        client, vehicle, world = Mock(), Mock(), Mock()
        world.get_settings.return_value.synchronous_mode = True
        with patch('modules.lincoln.driving.carla_autopilot.reserve_traffic_manager_port', return_value=nullcontext(8000)):
            controller = CarlaAutopilotController(client, world, vehicle, SimpleNamespace(tm_port=8000))
        client.get_trafficmanager.assert_called_once_with(8000)
        manager = client.get_trafficmanager.return_value
        manager.vehicle_percentage_speed_difference.assert_called_once_with(vehicle, -20.)
        vehicle.set_autopilot.assert_not_called()
        self.assertEqual(controller.traffic_manager_port, 8000)
        controller.enable()
        controller.disable()
        controller.enable()
        # 第一次和之后的开关均不能再次创建服务、选择端口。
        client.get_trafficmanager.assert_called_once_with(8000)
        self.assertEqual(vehicle.set_autopilot.call_args.args, (True, 8000))
        controller.set_gta_mode(True)
        manager.ignore_lights_percentage.assert_called_with(vehicle, 100.)
        controller.disable()
        controller.enable()
        manager.ignore_lights_percentage.assert_called_with(vehicle, 100.)
        controller.set_gta_mode(False)
        manager.ignore_lights_percentage.assert_called_with(vehicle, 0.)
        controller.set_navigation_speed(60.)
        controller.clear_route()
        manager.vehicle_percentage_speed_difference.assert_called_with(vehicle, -20.)


if __name__ == '__main__':
    unittest.main()
