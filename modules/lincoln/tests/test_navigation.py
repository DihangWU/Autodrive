"""导航的坐标、身份隔离、合法路线、任务取消与停车检查；不启动仿真器。"""
from pathlib import Path
import importlib.util
import tempfile
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import carla

from foundation.cobridge.lincoln.navigation import LincolnNavigation as BridgeNavigation, resolve_destination
from modules.lincoln.driving.navigation import (LincolnNavigation, plan_route,
                                               lane_change_finishes_before_junction, path_with_lookahead)
from modules.lincoln.driving.manual_driving import ManualDrivingMixin


class GtaControlTests(unittest.TestCase):
    def test_manual_mode_applies_on_driver_thread_and_cancel_keeps_mode(self):
        node = SimpleNamespace(driver=Mock(), autopilot=False)
        navigation = LincolnNavigation(node)
        request = dict(id='on', enabled=True)
        navigation.submit_gta_mode(request)
        node.driver.set_gta_mode.assert_not_called()
        self.assertFalse(navigation.gta_snapshot()['enabled'])
        navigation.update(None)  # 没有导航任务也能处理开关。
        node.driver.set_gta_mode.assert_called_once_with(True)
        navigation.submit_gta_mode(request)
        navigation.update(None)
        node.driver.set_gta_mode.assert_called_once_with(True)
        navigation.cancel()
        self.assertTrue(navigation.gta_snapshot()['enabled'])
        navigation.submit_gta_mode(dict(id='off', enabled=False))
        navigation.update(None)
        self.assertFalse(navigation.gta_snapshot()['enabled'])

    def test_failure_does_not_report_false_success_or_retry_forever(self):
        node = SimpleNamespace(driver=Mock(), autopilot=False)
        node.driver.set_gta_mode.side_effect = RuntimeError('TM unavailable')
        navigation = LincolnNavigation(node)
        navigation.submit_gta_mode(dict(id='on', enabled=True))
        with self.assertLogs(level='ERROR'):
            navigation.update(None)
        self.assertFalse(navigation.gta_snapshot()['enabled'])
        self.assertTrue(navigation.gta_snapshot()['error'])
        navigation.submit_gta_mode(dict(id='on', enabled=True))
        navigation.update(None)
        node.driver.set_gta_mode.assert_called_once()


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

    def test_gta_requests_wait_for_owner_ack_and_isolate_vehicles(self):
        first = dict(sumo_id='lincoln0', world_id=1, actor_id=2, state='synced',
                     navigation_supported=True, gta_supported=True)
        second = dict(first, sumo_id='lincoln1', actor_id=3)
        service = BridgeNavigation(SimpleNamespace(records={'1:2': first, '1:3': second}))
        world = SimpleNamespace(world=SimpleNamespace(id=1))
        sumo = SimpleNamespace(network=self.network)
        command = dict(action='lincoln.gta.set', sumo_id='lincoln0', owner_key='lincoln:1:2',
                       network_file=str(self.path), enabled=True)
        response = service.handle_gta_mode(command, world, sumo)
        self.assertTrue(response['pending'])
        self.assertFalse(response['enabled'])
        request = first['gta_request']
        BridgeNavigation.receive_gta_status(first, dict(enabled=True, request_id='stale'))
        self.assertNotIn('gta_status', first)
        BridgeNavigation.receive_gta_status(first, dict(enabled=True, request_id=request['id']))
        response = service.handle_gta_mode(dict(command, action='lincoln.gta.status'), world, sumo)
        self.assertTrue(response['enabled'])
        self.assertFalse(response['pending'])
        self.assertNotIn('gta_request', second)
        response = service.handle_gta_mode(dict(command, enabled=False), world, sumo)
        self.assertTrue(response['enabled'])  # 关闭确认回来之前仍显示实际开启。
        self.assertTrue(response['pending'])
        for change in (dict(enabled='false'), dict(owner_key='lincoln:1:99')):
            with self.assertRaises(ValueError):
                service.handle_gta_mode(dict(command, **change), world, sumo)
        first['gta_supported'] = False
        with self.assertRaisesRegex(ValueError, 'Restart'):
            service.handle_gta_mode(command, world, sumo)

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
        self.junction_id = 2
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

    def test_lane_change_cannot_skip_entry_lane_into_junction(self):
        adjacent, following = Waypoint(0, lane=-2), Waypoint(12, lane=-2)
        following.is_junction = True
        following.road_id = 1161
        self.assertFalse(lane_change_finishes_before_junction(adjacent, following))

    def test_lane_change_leaves_clearance_before_junction(self):
        adjacent, following, ahead = Waypoint(0, lane=-2), Waypoint(12, lane=-2), Waypoint(24, lane=-2)
        following.following = [ahead]
        self.assertTrue(lane_change_finishes_before_junction(adjacent, following))
        ahead.is_junction = True
        self.assertFalse(lane_change_finishes_before_junction(adjacent, following))

    def test_tm_path_uses_exit_road_without_mutating_green_route(self):
        start, inside, exit_point, lookahead = [Waypoint(x) for x in (0,4,8,12)]
        inside.is_junction = True
        inside.road_id = 1161
        exit_point.road_id = lookahead.road_id = 23
        exit_point.following = [lookahead]
        route = [start, inside, exit_point]
        points = path_with_lookahead(route)
        self.assertEqual([point.x for point in points], [0,12])
        self.assertEqual([point.road_id for point in route], [1,1161,23])

    def test_destination_inside_junction_can_use_an_exit_beyond_destination(self):
        goal, after = Waypoint(0), Waypoint(4)
        goal.is_junction = True
        goal.following = [after]
        self.assertEqual([point.x for point in path_with_lookahead([goal])], [4])
        goal.following = []
        with self.assertRaisesRegex(ValueError, 'exit guide'):
            path_with_lookahead([goal])

    def test_navigation_cannot_start_until_display_preparation_completes(self):
        entered,release=threading.Event(),threading.Event()
        node=SimpleNamespace(driver=Mock(supports_junction_local_control=False),autopilot=False,
                             active=None,car=Mock(),navigation_view=Mock())
        navigation=LincolnNavigation(node);navigation.road_map=self.road_map
        def prepare(*args):entered.set();release.wait(2);return True
        node.navigation_view.prepare_navigation.side_effect=prepare
        with patch('modules.lincoln.driving.navigation.plan_route',return_value=(self.points,self.points[-1])):
            try:
                navigation.submit(dict(id='ready',target=dict(x=40,y=0,z=0)))
                self.assertTrue(entered.wait(1))
                navigation.update(None)
                self.assertTrue(navigation.hold_brake)
                self.assertFalse(node.autopilot)
                self.assertTrue(navigation.snapshot()['display_preparing'])
                node.driver.set_route.assert_not_called()
                release.set()
                deadline=time.monotonic()+1
                while navigation.result is None and time.monotonic()<deadline:time.sleep(.01)
                navigation.consume_plan()
                self.assertTrue(node.autopilot)
                self.assertFalse(navigation.hold_brake)
                node.driver.set_route.assert_called_once()
            finally:release.set();navigation.close()

    def test_display_failure_holds_brakes_without_starting_tm(self):
        node=SimpleNamespace(driver=Mock(supports_junction_local_control=False),autopilot=False,
                             active=None,car=Mock(),navigation_view=Mock())
        node.navigation_view.prepare_navigation.side_effect=RuntimeError('RViz failed')
        navigation=LincolnNavigation(node);navigation.road_map=self.road_map
        with patch('modules.lincoln.driving.navigation.plan_route',return_value=(self.points,self.points[-1])):
            try:
                navigation.submit(dict(id='failed-view',target=dict(x=40,y=0,z=0)))
                deadline=time.monotonic()+1
                while navigation.result is None and time.monotonic()<deadline:time.sleep(.01)
                navigation.update(None)
                self.assertTrue(navigation.hold_brake)
                self.assertFalse(node.autopilot)
                self.assertEqual(navigation.snapshot()['state'],'error')
                node.driver.set_route.assert_not_called()
            finally:navigation.close()

    def test_late_plan_after_cancel_never_takes_control(self):
        node = SimpleNamespace(driver=Mock(), autopilot=False)
        navigation = LincolnNavigation(node)
        navigation.status = dict(id='task', state='planning')
        navigation.result = ('task', ([carla.Location(x=40)], self.points[-1], self.points, '', None))
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
        snapshot = SimpleNamespace(find=lambda _: actor, timestamp=SimpleNamespace(elapsed_seconds=10.))
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

    def active_navigation(self, x, speed, lane=-1):
        actor = SimpleNamespace(get_transform=lambda: carla.Transform(carla.Location(x=x)),
                                get_velocity=lambda: carla.Vector3D(x=speed))
        node = SimpleNamespace(driver=Mock(), autopilot=True,
                               car=SimpleNamespace(id=2, get_speed_limit=lambda: 90.))
        navigation = LincolnNavigation(node)
        navigation.status = dict(id='task', state='navigating')
        navigation.goal = self.points[-1]
        navigation.road_map = SimpleNamespace(get_waypoint=lambda *a, **kw: Waypoint(x, lane))
        navigation.prepare_route_tracking(self.points)
        return navigation, node, SimpleNamespace(find=lambda _: actor, timestamp=SimpleNamespace(elapsed_seconds=10.)), actor

    def test_approach_caps_speed_without_toggling_autopilot(self):
        navigation, node, snapshot, _ = self.active_navigation(25., 12.)
        self.assertIsNone(navigation.update(snapshot))
        node.driver.disable.assert_not_called()
        node.driver.enable.assert_not_called()
        self.assertLess(node.driver.set_navigation_speed.call_args.args[0], 40.)

    def test_destination_braking_never_reenables_tm_even_after_overshoot(self):
        navigation, node, snapshot, actor = self.active_navigation(37., 10.)
        self.assertEqual(navigation.update(snapshot).brake, 1.)
        actor.get_transform = lambda: carla.Transform(carla.Location(x=46))
        actor.get_velocity = lambda: carla.Vector3D()
        self.assertEqual(navigation.update(snapshot).brake, 1.)
        self.assertEqual(navigation.snapshot()['state'], 'error')
        self.assertFalse(node.autopilot)
        self.assertTrue(navigation.hold_brake)
        node.driver.disable.assert_called_once()
        node.driver.enable.assert_not_called()

    def test_wrong_destination_lane_stops_instead_of_driving_past(self):
        navigation, node, snapshot, _ = self.active_navigation(38., 2., lane=-2)
        self.assertEqual(navigation.update(snapshot).brake, 1.)
        self.assertIn('lane missed', navigation.stopping)
        node.driver.enable.assert_not_called()

    def test_passed_destination_stops_even_without_previous_arrival_sample(self):
        navigation, _, snapshot, _ = self.active_navigation(47., 8.)
        self.assertEqual(navigation.update(snapshot).brake, 1.)
        self.assertEqual(navigation.stopping, 'Destination passed')

    def test_persistent_deviation_stops_and_does_not_report_arrival(self):
        navigation, node, snapshot, actor = self.active_navigation(10., 8.)
        actor.get_transform = lambda: carla.Transform(carla.Location(x=10, y=5))
        self.assertIsNone(navigation.update(snapshot))
        snapshot.timestamp.elapsed_seconds = 11.
        self.assertEqual(navigation.update(snapshot).brake, 1.)
        self.assertEqual(navigation.stopping, 'Route deviation')
        node.driver.enable.assert_not_called()

    def test_upcoming_lane_change_reduces_target_speed(self):
        navigation, node, _, _ = self.active_navigation(0., 8.)
        navigation.route[3].lane_id = -2
        self.assertEqual(navigation.planned_speed(0, 1000, 8), 80.)

    def test_normal_navigation_allows_twenty_percent_above_road_limit(self):
        navigation, node, _, _ = self.active_navigation(0.,0.)
        node.car.get_speed_limit = lambda:150.
        self.assertEqual(navigation.planned_speed(0,1000.,0.),120.)
        node.car.get_speed_limit = lambda:50.
        self.assertEqual(navigation.planned_speed(0,1000.,0.),60.)

    def test_junction_uses_eighty_kmh_cap(self):
        navigation, node, _, _ = self.active_navigation(0.,8.)
        node.car.get_speed_limit = lambda:150.
        navigation.route[2].is_junction = True
        self.assertEqual(navigation.planned_speed(0,1000.,8.),80.)

    def test_tm_lane_change_accepts_only_planned_lanes_on_the_same_road(self):
        navigation, _, _, _ = self.active_navigation(0.,5.)
        for point in navigation.route[3:]:
            point.lane_id = -2
        current = Waypoint(10.,lane=-2)
        current.transform.location.y = 3.5
        position = carla.Location(x=10.,y=3.5)
        self.assertEqual(navigation.tm_lane_change_deviation(position,current,4.),0.)
        current.lane_id = -3
        self.assertEqual(navigation.tm_lane_change_deviation(position,current,4.),4.)
        current.lane_id,current.road_id = -2,2
        self.assertEqual(navigation.tm_lane_change_deviation(position,current,4.),4.)

    def test_tm_exit_guides_avoid_overlapping_junction_edge(self):
        points = [Waypoint(x) for x in (0,4,8,12,24)]
        points[1].is_junction = True
        points[1].road_id = 100
        for point in points[2:]:point.road_id=3
        self.assertEqual([p.x for p in path_with_lookahead(points)],[0,24])


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


@unittest.skipUnless(importlib.util.find_spec('geometry_msgs'), '需要加载 ROS 2 环境以复用绿色轨迹采样')
class JunctionControlTests(unittest.TestCase):
    def setUp(self):
        from modules.lincoln.driving.junction_local_controller import JunctionLocalController
        class Actors(list):
            def filter(self, pattern):
                return [actor for actor in self if actor.type_id.startswith('traffic.traffic_light')]
        self.actors = Actors()
        wheels = [SimpleNamespace(position=carla.Vector3D(x=x,y=y),max_steer_angle=70.)
                  for x,y in ((150,80),(150,-80),(-150,80),(-150,-80))]
        physics = SimpleNamespace(wheels=wheels,steering_curve=[carla.Vector2D(x=0,y=1),carla.Vector2D(x=20,y=.8)])
        self.vehicle = SimpleNamespace(id=1,bounding_box=carla.BoundingBox(carla.Location(),carla.Vector3D(x=2.5,y=1,z=1)),
                                       get_physics_control=lambda:physics,get_control=lambda:carla.VehicleControl())
        self.world = SimpleNamespace(get_actors=lambda:self.actors)
        self.route = [Waypoint(x) for x in range(0,121,4)]
        for point in self.route:
            point.lane_width = 3.5
            point.is_junction = 40 <= point.s <= 60
            point.road_id = 2 if point.is_junction else (3 if point.s>60 else 1)
        self.node = SimpleNamespace(car=self.vehicle,world=self.world)
        with patch('modules.lincoln.driving.junction_local_controller.JunctionBranchClassifier') as classifier:
            classifier.return_value.requires_local_control.return_value = True
            self.controller = JunctionLocalController(self.node,self.route,None,[p.s for p in self.route])

    def sample(self, x, speed=0., now=10., other=None):
        ego = SimpleNamespace(get_transform=lambda:carla.Transform(carla.Location(x=x)),
                              get_velocity=lambda:carla.Vector3D(x=speed))
        return SimpleNamespace(find=lambda actor_id:ego if actor_id==1 else other,
                               timestamp=SimpleNamespace(elapsed_seconds=now))

    def test_takeover_precedes_junction_and_return_requires_stable_correct_exit(self):
        local = self.controller
        self.assertFalse(local.should_take_over(0.,0.))
        self.assertTrue(local.should_take_over(15.,0.))
        local.activate(76.)
        transform = carla.Transform(carla.Location(x=76))
        current = self.route[19]
        self.assertFalse(local.can_return_to_tm(76.,transform,current,10.))
        self.assertTrue(local.can_return_to_tm(76.,transform,current,10.6))
        wrong_lane = Waypoint(76.,lane=-2)
        wrong_lane.road_id = 3
        self.assertFalse(local.can_return_to_tm(76.,transform,wrong_lane,11.))

    def test_gta_mode_bypasses_lights_and_restores_stopping(self):
        local = self.controller
        local.signal_stops = [(32., SimpleNamespace(get_state=lambda:carla.TrafficLightState.Red))]
        local.activate(29.)
        local.ignore_traffic_lights = True
        self.assertGreater(local.run_step(self.sample(29.),20.).throttle,0.)
        self.assertEqual(local.wait_reason, '')
        local.ignore_traffic_lights = False
        self.assertEqual(local.run_step(self.sample(29.,now=10.05),20.).brake,1.)
        self.assertEqual(local.wait_reason, 'traffic_light')

    def test_red_and_yellow_stop_then_green_releases(self):
        local = self.controller
        light = SimpleNamespace(get_state=lambda:carla.TrafficLightState.Red)
        local.signal_stops = [(32.,light)]
        local.activate(29.)
        control = local.run_step(self.sample(29.),20.)
        self.assertEqual(control.brake,1.)
        self.assertEqual(local.wait_reason,'traffic_light')
        light.get_state = lambda:carla.TrafficLightState.Yellow
        self.assertEqual(local.run_step(self.sample(29.,now=10.05),20.).brake,1.)
        light.get_state = lambda:carla.TrafficLightState.Green
        self.assertGreater(local.run_step(self.sample(29.,now=10.1),20.).throttle,0.)

    def test_nearby_junctions_keep_local_control_without_tm_chatter(self):
        local = self.controller
        local.windows.append(dict(start=70.,entry=96.,exit=110.,exit_index=28))
        local.activate(76.)
        self.assertFalse(local.can_return_to_tm(76.,carla.Transform(carla.Location(x=76)),self.route[19],10.))
        self.assertEqual(local.window_index,1)
        self.assertTrue(local.active)

    def test_destination_inside_junction_keeps_local_control_for_arrival_logic(self):
        local = self.controller
        local.windows[0]['exit'] = local.route_lengths[-1]
        local.activate(118.)
        self.assertFalse(local.can_return_to_tm(118.,carla.Transform(carla.Location(x=118)),self.route[-1],10.))

    def test_blocking_vehicle_stops_but_adjacent_lane_and_overpass_do_not(self):
        local = self.controller
        obstacle = SimpleNamespace(id=2,type_id='vehicle.test',
                                    bounding_box=carla.BoundingBox(carla.Location(),carla.Vector3D(x=2.5,y=1,z=1)))
        self.actors.append(obstacle)
        local.ignore_traffic_lights = True  # GTA Mode 仍须避障。
        position = carla.Location(x=35)
        state = SimpleNamespace(get_transform=lambda:carla.Transform(position))
        local.activate(31.)
        self.assertEqual(local.run_step(self.sample(31.,other=state),20.).brake,1.)
        self.assertEqual(local.wait_reason,'obstacle')
        position.y = 3.5
        self.assertGreater(local.run_step(self.sample(31.,now=10.1,other=state),20.).throttle,0.)
        position.y, position.z = 0., 6.
        self.assertGreater(local.run_step(self.sample(31.,now=10.2,other=state),20.).throttle,0.)

    def test_local_control_and_tm_are_never_written_in_the_same_update(self):
        local = self.controller
        node = SimpleNamespace(car=self.vehicle,driver=Mock(),navigation_view=Mock())
        navigation = LincolnNavigation(node)
        navigation.prepare_route_tracking(self.route)
        navigation.local_controller = local
        navigation.route_progress = 15.
        snapshot = self.sample(15.)
        control = navigation.update_junction_control(snapshot,carla.Transform(carla.Location(x=15)),self.route[4],3,20.)
        self.assertIsInstance(control,carla.VehicleControl)
        node.driver.disable.assert_called_once()
        node.driver.set_route.assert_not_called()
        navigation.cancel()
        self.assertIsNone(navigation.local_controller)

    def test_handoff_uploads_only_remaining_route_then_releases_local_control(self):
        node = SimpleNamespace(car=self.vehicle,driver=Mock(),navigation_view=Mock())
        navigation = LincolnNavigation(node)
        navigation.prepare_route_tracking(self.route)
        navigation.route_progress = 76.
        local = self.controller
        local.activate(76.)
        local.exit_stable_since = 9.
        navigation.local_controller = local
        snapshot = self.sample(76.,now=10.)
        control = navigation.update_junction_control(snapshot,carla.Transform(carla.Location(x=76)),self.route[19],19,20.)
        self.assertIsNone(control)
        self.assertFalse(local.active)
        uploaded = node.driver.set_route.call_args.args[0]
        self.assertGreaterEqual(min(p.x for p in uploaded),80.)
        self.assertEqual(navigation.snapshot()['controller'],'tm')

    def test_ui_distinguishes_local_waiting_and_tm(self):
        from modules.lincoln.ui.driving_panel import navigation_control_display
        text,color = navigation_control_display(dict(state='navigating',controller='local',control_detail='obstacle'),True)
        self.assertIn('路口局部控制',text)
        self.assertIn('障碍物',text)
        tm_text,tm_color = navigation_control_display(dict(state='navigating',controller='tm'),True)
        self.assertIn('TM',tm_text)
        self.assertNotEqual(color,tm_color)

    def test_straight_guidance_stays_inside_tm(self):
        node = SimpleNamespace(driver=Mock(supports_junction_local_control=True))
        navigation = LincolnNavigation(node)
        navigation.prepare_route_tracking(self.route)
        self.route[16].transform.rotation.yaw = 0.
        navigation.prepare_tm_turns()
        self.assertEqual(navigation.tm_turns[0]['action'],'Straight')
        navigation.route_progress = 25.
        navigation.update_tm_turn_guidance(6,self.route[6])
        node.driver.set_turn_guidance.assert_called_once_with(['Straight'])
        node.driver.disable.assert_not_called()
        navigation.route_progress = 76.
        navigation.update_tm_turn_guidance(19,self.route[19])
        self.assertEqual(node.driver.set_turn_guidance.call_args.args[0],[])
        node.driver.disable.assert_not_called()

    def test_selected_exit_does_not_receive_ordinary_tm_turn_commands(self):
        node = SimpleNamespace(driver=Mock(supports_junction_local_control=True))
        navigation = LincolnNavigation(node)
        navigation.prepare_route_tracking(self.route)
        navigation.local_controller = self.controller
        navigation.prepare_tm_turns()
        self.assertEqual(navigation.tm_turns,[])
        navigation.update_tm_turn_guidance(6,self.route[6])
        node.driver.set_turn_guidance.assert_not_called()

    def test_straight_route_has_no_local_window_or_vehicle_physics_setup(self):
        from modules.lincoln.driving.junction_local_controller import JunctionLocalController
        self.vehicle.get_physics_control = Mock(side_effect=AssertionError('unneeded physics setup'))
        with patch('modules.lincoln.driving.junction_local_controller.JunctionBranchClassifier') as classifier:
            classifier.return_value.requires_local_control.return_value = False
            local = JunctionLocalController(self.node,self.route,None,[p.s for p in self.route])
        self.assertFalse(local.should_take_over(1000.,30.))
        self.vehicle.get_physics_control.assert_not_called()

    def test_speed_integral_can_compensate_for_cruising_resistance_and_resets_at_stop(self):
        local = self.controller
        for _ in range(160):
            throttle,brake = local.longitudinal_control(7.,8.,.05)
        self.assertGreater(throttle,.7)
        self.assertEqual(brake,0.)
        self.assertEqual(local.longitudinal_control(1.,0.,.05),(0.,1.))
        self.assertEqual(local.integral,0.)


class JunctionBranchClassificationTests(unittest.TestCase):
    def setUp(self):
        from modules.lincoln.driving.junction_local_controller import JunctionBranchClassifier
        self.classifier_type = JunctionBranchClassifier
        self.incoming = Waypoint(0)
        self.entry = Waypoint(4)
        self.entry.road_id, self.entry.junction_id, self.entry.is_junction = 100,99,True
        self.main = Waypoint(4)
        self.main.road_id,self.main.junction_id,self.main.is_junction = 200,99,True
        self.branch_probe,self.main_probe = Waypoint(22),Waypoint(22)
        self.entry.following = [self.branch_probe]
        self.main.following = [self.main_probe]
        self.entry.get_junction = lambda:SimpleNamespace(get_waypoints=lambda _: [(self.entry,self.branch_probe),(self.main,self.main_probe)])
        self.main.get_junction = self.entry.get_junction

    def classifier(self, selected_lanes=1, alternate=True):
        def road(identity,count,links='',junction='-1'):
            lanes=''.join(f'<lane id="-{i}" type="driving"/>' for i in range(1,count+1))
            return f'<road id="{identity}" junction="{junction}"><link>{links}</link><lanes><laneSection><right>{lanes}</right></laneSection></lanes></road>'
        def connector(identity,outgoing):
            return road(identity,1,f'<predecessor elementType="road" elementId="1" contactPoint="end"/>'
                        f'<successor elementType="road" elementId="{outgoing}" contactPoint="start"/>','99')
        connections='<connection incomingRoad="1" connectingRoad="100"><laneLink from="-1" to="-1"/></connection>'
        if alternate:
            connections+='<connection incomingRoad="1" connectingRoad="200"><laneLink from="-1" to="-1"/></connection>'
        xml='<OpenDRIVE>'+road(1,3)+road(3,selected_lanes)+road(4,3)+connector(100,3)+connector(200,4)+f'<junction id="99">{connections}</junction></OpenDRIVE>'
        return self.classifier_type(SimpleNamespace(to_opendrive=lambda:xml, get_waypoint_xodr=lambda *args:self.incoming))

    def test_ordinary_left_right_and_uturn_are_local_but_straight_is_tm(self):
        classifier = self.classifier(selected_lanes=3)
        for yaw in (-90., 90., 180.):
            self.branch_probe.transform.rotation.yaw = yaw
            self.assertTrue(classifier.requires_local_control(self.entry,self.incoming,self.branch_probe))
        self.branch_probe.transform.rotation.yaw = 0.
        self.assertFalse(classifier.requires_local_control(self.entry,self.incoming,self.branch_probe))

    def test_equal_width_diverging_choice_is_local_but_mainline_is_tm(self):
        self.branch_probe.transform.rotation.yaw = 20.
        classifier = self.classifier(selected_lanes=3)
        self.assertTrue(classifier.requires_local_control(self.entry,self.incoming,self.branch_probe))
        self.assertFalse(classifier.requires_local_control(self.main,self.incoming,self.main_probe))

    def test_curved_mainline_is_not_taken_over_for_a_straighter_narrow_exit(self):
        self.main_probe.transform.rotation.yaw = -27.
        classifier = self.classifier()
        self.assertFalse(classifier.requires_local_control(self.main,self.incoming,self.main_probe))
        self.assertTrue(classifier.requires_local_control(self.entry,self.incoming,self.branch_probe))

    def test_real_classifier_builds_windows_for_turn_but_not_straight(self):
        from modules.lincoln.driving.junction_local_controller import JunctionLocalController
        local = JunctionLocalController.__new__(JunctionLocalController)
        self.branch_probe.road_id = 3
        local.route = [self.incoming,self.entry,self.branch_probe]
        local.route_lengths = [0.,4.,22.]
        local.branch_classifier = self.classifier(selected_lanes=3)
        self.branch_probe.transform.rotation.yaw = 90.
        self.assertEqual(len(local.find_junction_windows()),1)
        self.branch_probe.transform.rotation.yaw = 0.
        self.assertEqual(local.find_junction_windows(),[])

    def test_starting_inside_turn_uses_true_incoming_heading(self):
        self.entry.transform.rotation.yaw = 80.
        self.branch_probe.transform.rotation.yaw = 90.
        self.assertTrue(self.classifier(selected_lanes=3).requires_local_control(self.entry))

    def test_selected_narrow_exit_is_local_but_main_continuation_remains_tm(self):
        classifier=self.classifier()
        self.assertTrue(classifier.requires_exit_selection(self.entry,self.incoming))
        self.assertFalse(classifier.requires_exit_selection(self.main,self.incoming))

    def test_ninety_degree_turn_is_not_a_narrow_gentle_exit(self):
        self.branch_probe.transform.rotation.yaw=90.
        self.assertFalse(self.classifier().requires_exit_selection(self.entry,self.incoming))

    def test_equal_width_fork_and_single_connection_do_not_trigger_exit_control(self):
        self.assertFalse(self.classifier(selected_lanes=3).requires_exit_selection(self.entry,self.incoming))
        self.assertFalse(self.classifier(alternate=False).requires_exit_selection(self.entry,self.incoming))

    def test_starting_inside_selected_exit_can_infer_incoming_road(self):
        self.assertTrue(self.classifier().requires_exit_selection(self.entry))


if __name__ == '__main__':
    unittest.main()
