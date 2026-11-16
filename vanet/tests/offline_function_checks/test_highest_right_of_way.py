"""最高路权允许跨标线和逆向车道连接，但不穿越实体隔离。"""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import carla
from vanet.lincoln.emergency_route_planning.priority_lane_access import PriorityWaypoint,priority_key
from vanet.lincoln.emergency_route_planning.priority_route_planner import plan_priority_route
from vanet.lincoln.emergency_driving_control.control_configuration import load_control_configuration,load_highest_right_of_way
from vanet.lincoln.emergency_driving_control.trajectory_controller import EmergencyTrajectoryController
from vanet.tests.offline_function_checks import test_lincoln_service_commands as service_fixtures


class Lane:
    road_id=1;section_id=0;lane_width=3.5;is_junction=False;lane_change=carla.LaneChange.NONE
    lane_type=carla.LaneType.Driving
    def __init__(self,x,lane_id=-1):
        self.x=x;self.s=x if lane_id<0 else 100.-x;self.lane_id=lane_id
        self.transform=carla.Transform(carla.Location(x=x,y=0. if lane_id<0 else 3.5),carla.Rotation(yaw=0. if lane_id<0 else 180.))
    def next(self,distance):
        x=self.x+(distance if self.lane_id<0 else -distance)
        return [Lane(x,self.lane_id)] if 0.<=x<=24. else []
    def previous(self,distance):
        x=self.x+(-distance if self.lane_id<0 else distance)
        return [Lane(x,self.lane_id)] if 0.<=x<=24. else []
    def get_right_lane(self):return None
    def get_left_lane(self):return Lane(self.x,-self.lane_id)


class HighestRightOfWayTests(unittest.TestCase):
    def test_default_policy_has_no_fixed_speed_ceiling(self):
        self.assertTrue(load_highest_right_of_way())
        maximum,_=load_control_configuration()
        self.assertIsNone(maximum)

    def test_solid_marking_does_not_block_opposite_lane_access(self):
        world=Mock();world.cast_ray.return_value=[]
        adjacent=list(PriorityWaypoint(Lane(0.)).adjacent(world))
        self.assertEqual(len(adjacent),1)
        self.assertEqual(adjacent[0].direction,-1)
        following=adjacent[0].next(12.)[0]
        self.assertEqual(following.x,12.)
        self.assertGreater(following.transform.get_forward_vector().x,.99)

    def test_physical_barrier_still_blocks_crossing(self):
        world=Mock();world.cast_ray.return_value=[SimpleNamespace(label='Walls')]
        self.assertEqual(list(PriorityWaypoint(Lane(0.)).adjacent(world)),[])

    def test_route_can_use_reverse_lane_and_preserves_destination_heading(self):
        world=Mock();world.cast_ray.return_value=[]
        road_map=Mock();road_map.get_waypoint.side_effect=[Lane(0.),Lane(20.,1)]
        route,goal=plan_priority_route(road_map,world,carla.Location(),carla.Location(x=20.,y=3.5),heading=0.)
        self.assertTrue(any(point.direction<0 for point in route))
        self.assertLess(goal.transform.location.distance(carla.Location(x=20.,y=3.5)),.01)
        self.assertGreater(goal.transform.get_forward_vector().x,.99)
        self.assertNotEqual(priority_key(PriorityWaypoint(Lane(0.),1)),priority_key(PriorityWaypoint(Lane(0.),-1)))

    def test_reverse_pose_matches_actual_start_heading(self):
        world=Mock();world.cast_ray.return_value=[]
        road_map=Mock();road_map.get_waypoint.side_effect=[Lane(20.),Lane(0.)]
        route,goal=plan_priority_route(road_map,world,carla.Location(x=20.),carla.Location(),heading=180.)
        self.assertEqual(route[0].direction,-1)
        self.assertLess(goal.transform.get_forward_vector().x,-.99)

    def test_all_groups_use_trajectory_control_without_starting_tm(self):
        for group in ('normal','emergency','cooperative'):
            with self.subTest(group=group):
                node,service=service_fixtures.ServiceCommandTests().make_service()
                service.highest_right_of_way=True
                points=[Lane(0.),Lane(20.)]
                try:
                    with patch('vanet.lincoln.emergency_state_management.emergency_service.plan_priority_route',return_value=(points,points[-1])) as planner,patch('vanet.lincoln.emergency_state_management.emergency_service.EmergencyTrajectoryController') as factory:
                        factory.return_value.run_step.return_value=carla.VehicleControl(throttle=.5)
                        service.execute(dict(id='prepare',action='prepare',group=group,task_id='priority',destination=dict(x=20.,y=0.,z=0.)))
                        service.worker.join(1.)
                        service.process_commands()
                        self.assertEqual(service.task.state,'prepared')
                        service.execute(dict(id='start',action='start'))
                        self.assertIs(service.controller,factory.return_value)
                        self.assertEqual(service.task.emergency,group!='normal')
                        node.driver.enable.assert_not_called()
                        self.assertIsNone(factory.call_args.args[3])
                        planner.assert_called_once()
                finally:service.close();node.navigation.close()

    def test_unlimited_controller_keeps_curve_and_collision_checks(self):
        controller=EmergencyTrajectoryController.__new__(EmergencyTrajectoryController)
        controller.vehicle=SimpleNamespace(id=7);controller.maximum_speed=math.inf
        controller.last_time=None;controller.progress=0.;controller.distances=[0.,2000.]
        controller.lateral_control=Mock(return_value=0.)
        controller.curve_speed_limit=Mock(side_effect=lambda target,speed:target)
        controller.obstacle_speed_limit=Mock(side_effect=lambda snapshot,target,speed,now:target)
        controller.longitudinal_control=Mock(return_value=(1.,0.))
        ego=Mock();ego.get_transform.return_value=carla.Transform();ego.get_velocity.return_value=carla.Vector3D()
        snapshot=Mock(timestamp=SimpleNamespace(elapsed_seconds=1.));snapshot.find.return_value=ego
        controller.run_step(snapshot)
        self.assertGreater(controller.target_speed,120.)
        controller.curve_speed_limit.assert_called_once()
        controller.obstacle_speed_limit.assert_called_once()

    def test_reverse_sampling_does_not_restore_legal_lane_heading(self):
        from modules.lincoln.visualization.navigation_route import sample_route
        road_map=Mock()
        road_map.get_waypoint_xodr.side_effect=lambda road,lane,s:Lane(s)
        samples=sample_route([PriorityWaypoint(Lane(20.),-1),PriorityWaypoint(Lane(0.),-1)],road_map)
        self.assertTrue(all(sample.forward[0]<-.99 for sample in samples))

    def test_driving_panel_names_the_actual_priority_controller(self):
        from modules.lincoln.ui.driving_panel import navigation_control_display
        label,_=navigation_control_display(dict(controller='highest_priority',state='running'),True)
        self.assertIn('最高路权',label)
