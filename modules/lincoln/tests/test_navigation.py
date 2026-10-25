"""导航的坐标、身份隔离、合法路线、任务取消与停车检查；不启动仿真器。"""
from pathlib import Path
import tempfile
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import carla

from foundation.cobridge.lincoln.navigation import LincolnNavigation as BridgeNavigation, resolve_destination
from modules.lincoln.driving.navigation import LincolnNavigation, plan_route
from modules.lincoln.driving.manual_driving import ManualDrivingMixin


class DestinationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'test.net.xml'
        self.network = dict(network_file=self.path, offset=[100, 200])
        self.path.write_text('<net><edge id="a"><lane id="a_0" shape="100,200,7 200,200,7"/></edge></net>')

    def tearDown(self):
        self.directory.cleanup()

    def test_snap_and_convert_including_height(self):
        self.assertEqual(resolve_destination(self.network, 125, 201), dict(x=25, y=0, z=7, sumo_lane='a_0'))

    def test_far_nonfinite_and_boolean_coordinates_rejected(self):
        for x, y in ((500, 500), (float('nan'), 200), (True, 200)):
            with self.assertRaises(ValueError):
                resolve_destination(self.network, x, y)

    def test_overlap_at_different_heights_rejected(self):
        self.path.write_text('<net><edge id="a"><lane id="a_0" shape="100,200,0 200,200,0"/>'
                             '<lane id="a_1" shape="100,200,10 200,200,10"/></edge></net>')
        with self.assertRaisesRegex(ValueError, 'overlap'):
            resolve_destination(self.network, 125, 200)

    def test_vehicle_world_network_and_client_capability_checked(self):
        record = dict(sumo_id='lincoln0', world_id=1, actor_id=2, state='synced', navigation_supported=True)
        registry = SimpleNamespace(records={'1:2': record})
        service = BridgeNavigation(registry)
        command = dict(action='lincoln.navigate', sumo_id='lincoln0', owner_key='lincoln:1:2',
                       network_file=str(self.path), x=125, y=200)
        world = SimpleNamespace(world=SimpleNamespace(id=1))
        sumo = SimpleNamespace(network=self.network)
        result = service.handle(command, world, sumo)
        self.assertEqual(result['state'], 'planning')
        task = record['navigation_request']
        self.assertEqual(task['target']['z'], 7)
        for fields in (dict(owner_key='lincoln:1:3'), dict(network_file='/another.net.xml'), dict(sumo_id='lincoln1')):
            with self.assertRaises(ValueError):
                service.handle(dict(command, **fields), world, sumo)
        record['navigation_supported'] = False
        with self.assertRaisesRegex(ValueError, 'Restart'):
            service.handle(command, world, sumo)

    def test_stale_status_does_not_overwrite_current_task(self):
        record = dict(navigation_request=dict(id='new'), navigation=dict(id='new', state='planning'))
        BridgeNavigation.receive_status(record, dict(id='old', state='arrived'))
        self.assertEqual(record['navigation']['state'], 'planning')
        BridgeNavigation.receive_status(record, dict(id='new', state='arrived', detail='ok'))
        self.assertEqual(record['navigation']['state'], 'arrived')


class Waypoint:
    def __init__(self, x, lane=-1):
        self.road_id, self.section_id, self.lane_id, self.s = 1, 0, lane, x
        self.transform = carla.Transform(carla.Location(x=x))
        self.is_junction = False
        self.lane_change = carla.LaneChange.NONE
        self.lane_type = carla.LaneType.Driving
        self.following = []

    def next(self, distance):
        return self.following

    def get_left_lane(self):
        return None

    def get_right_lane(self):
        return None


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.points = [Waypoint(x) for x in range(0, 41, 4)]
        for first, second in zip(self.points, self.points[1:]):
            first.following = [second]
        self.road_map = SimpleNamespace(get_waypoint=lambda location, **kw: min(
            self.points, key=lambda waypoint: abs(waypoint.s - location.x)))

    def test_directed_route_reaches_target(self):
        route, goal = plan_route(self.road_map, carla.Location(x=0), carla.Location(x=40))
        self.assertIs(goal, self.points[-1])
        self.assertEqual(route[0].s, 0)
        self.assertEqual(route[-1].s, 40)

    def test_no_illegal_reverse_connection_added(self):
        with self.assertRaisesRegex(ValueError, 'No legal'):
            plan_route(self.road_map, carla.Location(x=40), carla.Location(x=0))

    def test_invalid_target_height_rejected(self):
        with self.assertRaisesRegex(ValueError, 'height'):
            plan_route(self.road_map, carla.Location(x=0), carla.Location(x=40, z=15))

    def test_late_plan_after_cancel_never_takes_control(self):
        node = SimpleNamespace(driver=Mock(), autopilot=False)
        navigation = LincolnNavigation(node)
        navigation.status = dict(id='task', state='planning')
        navigation.result = ('task', ([carla.Location(x=40)], self.points[-1], ''))
        navigation.cancel()
        navigation.consume_plan()
        node.driver.set_route.assert_not_called()
        self.assertEqual(navigation.snapshot()['state'], 'cancelled')

    def test_manual_input_during_delivery_delay_cancels_request(self):
        node = SimpleNamespace(driver=Mock(), last_manual_control=time.monotonic())
        navigation = LincolnNavigation(node)
        navigation.submit(dict(id='late', issued_at=0., target=dict(x=40, y=0, z=0)))
        self.assertEqual(navigation.snapshot()['state'], 'cancelled')
        self.assertIsNone(navigation.worker)

    def test_arrival_requires_correct_lane_and_stops_vehicle(self):
        actor = SimpleNamespace(get_transform=lambda: carla.Transform(carla.Location(x=40)),
                                get_velocity=lambda: carla.Vector3D())
        snapshot = SimpleNamespace(find=lambda _: actor)
        node = SimpleNamespace(driver=Mock(), autopilot=True, car=SimpleNamespace(id=2))
        navigation = LincolnNavigation(node)
        navigation.status = dict(id='task', state='navigating')
        navigation.goal = self.points[-1]
        navigation.road_map = self.road_map
        control = navigation.update(snapshot)
        self.assertEqual(control.brake, 1.)
        self.assertFalse(node.autopilot)
        self.assertTrue(navigation.hold_brake)
        self.assertEqual(navigation.snapshot()['state'], 'arrived')


class ManualNavigationTests(unittest.TestCase):
    def setUp(self):
        class Node(ManualDrivingMixin):
            pass
        self.node = node = Node()
        node.control_lock = threading.RLock()
        node.xbox = SimpleNamespace(update=Mock(return_value=None))
        node.navigation = SimpleNamespace(cancel=Mock(), hold_brake=False)
        node.driver = Mock()
        node.autopilot, node.reverse, node.active, node.cruise_speed = False, False, None, None
        node.car = SimpleNamespace(is_alive=True, apply_control=Mock())

    def test_arrival_brake_is_held_until_manual_input(self):
        node = self.node
        node.navigation.hold_brake = True
        node.drive()
        self.assertEqual(node.car.apply_control.call_args.args[0].brake, 1.)
        node.xbox.update.return_value = (.5, 0., .2)
        node.drive()
        node.navigation.cancel.assert_called_once()
        self.assertAlmostEqual(node.car.apply_control.call_args.args[0].throttle, .5)

    def test_panel_cancel_also_cancels_a_pending_plan(self):
        self.node.set_autopilot(False)
        self.node.navigation.cancel.assert_called_once()
        self.assertGreater(self.node.last_manual_control, 0.)

    def test_running_navigation_does_not_receive_manual_zero_controls(self):
        self.node.autopilot = True
        self.node.drive()
        self.node.car.apply_control.assert_not_called()


if __name__ == '__main__':
    unittest.main()
