"""无仿真器的登记/归属回归测试。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from foundation.cobridge.lincoln.vehicle_registry import LincolnVehicleRegistry
from modules.lincoln.driving.controller import DrivingController, load_driving_controller


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


if __name__ == '__main__':
    unittest.main()
