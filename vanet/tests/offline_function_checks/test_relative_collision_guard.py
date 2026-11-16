"""前车运动、真实碰撞边界与制动恢复的回归检查。"""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import carla
from vanet.lincoln.emergency_driving_control.collision_guard import collision_speed_limit
from vanet.lincoln.emergency_driving_control.guard_actuation import GuardActuation
from vanet.lincoln.emergency_driving_control.control_configuration import DEFAULT_COLLISION_GUARD
from vanet.lincoln.emergency_driving_control.trajectory_controller import EmergencyTrajectoryController
from vanet.lincoln.emergency_route_planning.borrowing_planner import BorrowingPlanner


class RelativeCollisionGuardTests(unittest.TestCase):
    def controller(self):
        return SimpleNamespace(samples=[SimpleNamespace(center=(float(x),0.,0.),forward=(1.,0.,0.)) for x in range(1001)],
            distances=list(range(1001)),progress=0.,front=2.5,half_width=1.,wait_reason='',
            collision_configuration=dict(DEFAULT_COLLISION_GUARD))

    def obstacle(self,x=80.,y=0.,z=0.,speed=14.,kind='vehicle.npc'):
        return dict(id=10,kind=kind,center=(x,y,z),velocity=(speed,0.,0.),yaw=0.,length=2.5,width=1.,height=1.)

    def test_moving_npc_does_not_cap_speed_before_or_at_radio_range(self):
        for distance in (80.,60.,50.,45.):
            with self.subTest(distance=distance):
                controller=self.controller()
                target=200./3.6
                self.assertEqual(collision_speed_limit(controller,[self.obstacle(x=distance)],target,55.8/3.6),target)
                self.assertEqual(controller.collision_minimum_brake,0.)

    def test_highway_speed_can_continue_when_relative_gap_is_safe(self):
        controller=self.controller()
        self.assertEqual(collision_speed_limit(controller,[self.obstacle()],200./3.6,120./3.6),200./3.6)

    def test_near_collision_brakes_immediately(self):
        controller=self.controller()
        target=collision_speed_limit(controller,[self.obstacle(x=20.)],200./3.6,28.)
        self.assertLess(target,28.)
        self.assertEqual(controller.collision_minimum_brake,1.)
        self.assertEqual(controller.wait_reason,'collision_guard')

    def test_stationary_and_oncoming_vehicles_require_earlier_braking(self):
        for speed in (0.,-10.):
            with self.subTest(speed=speed):
                controller=self.controller()
                self.assertLess(collision_speed_limit(controller,[self.obstacle(x=40.,speed=speed)],200./3.6,28.),28.)

    def test_other_lane_and_overpass_do_not_block_the_route(self):
        for obstacle in (self.obstacle(x=10.,y=3.5),self.obstacle(x=10.,z=8.)):
            controller=self.controller()
            self.assertEqual(collision_speed_limit(controller,[obstacle],200./3.6,28.),200./3.6)

    def test_pedestrian_and_static_facilities_keep_conservative_checks(self):
        for kind in ('walker.pedestrian','static.prop.barrier'):
            controller=self.controller()
            self.assertLess(collision_speed_limit(controller,[self.obstacle(x=40.,kind=kind,speed=0.)],200./3.6,28.),200./3.6)
            self.assertEqual(controller.wait_reason,'obstacle')

    def test_boundary_noise_does_not_toggle_guard(self):
        controller=self.controller()
        for x in [37.]+[38.,37.]*10:
            collision_speed_limit(controller,[self.obstacle(x=x)],200./3.6,28.)
            self.assertIsNotNone(controller.collision_guard_state)
        collision_speed_limit(controller,[self.obstacle(x=80.)],200./3.6,28.)
        self.assertIsNone(controller.collision_guard_state)

    def test_kinematic_proxy_motion_is_estimated_from_consecutive_frames(self):
        actor=SimpleNamespace(id=10,type_id='vehicle.npc',bounding_box=carla.BoundingBox(carla.Location(),carla.Vector3D(2.5,1.,1.)))
        node=SimpleNamespace(car=SimpleNamespace(id=7),world=SimpleNamespace(get_actors=lambda:[actor]))
        planner=BorrowingPlanner(node,None)
        for now,x in ((1.,20.),(1.05,20.7)):
            state=SimpleNamespace(get_transform=lambda:carla.Transform(carla.Location(x=x)),get_velocity=lambda:carla.Vector3D())
            snapshot=SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=now),find=lambda actor_id:state)
            obstacles=planner.obstacles(snapshot)
        self.assertAlmostEqual(obstacles[0]['velocity'][0],14.,places=4)

    def test_recovery_is_smooth_but_a_new_threat_is_immediate(self):
        guard=GuardActuation(dict(DEFAULT_COLLISION_GUARD))
        self.assertEqual(guard.apply(1.,0.,1.,True,.05),(0.,1.))
        previous=(0.,1.)
        for _ in range(20):
            output=guard.apply(1.,0.,0.,False,.05)
            self.assertLessEqual(previous[1]-output[1],.1+1e-9)
            self.assertLessEqual(output[0]-previous[0],.075+1e-9)
            previous=output
        self.assertGreater(output[0],0.)
        self.assertEqual(guard.apply(1.,0.,1.,True,.05),(0.,1.))

    def test_delayed_npc_lane_change_remains_collision_free_in_closed_loop(self):
        # Deterministic plant, not a calibration of CARLA's tire/brake model.
        controller=EmergencyTrajectoryController.__new__(EmergencyTrajectoryController)
        controller.__dict__.update(self.controller().__dict__)
        controller.vehicle=SimpleNamespace(id=7);controller.maximum_speed=math.inf;controller.last_time=None
        controller.integral=controller.previous_error=0.
        controller.curve_speed_limit=lambda target,speed:target
        def lateral(transform,speed,dt):controller.progress=transform.location.x;return 0.
        controller.lateral_control=lateral
        x=0.;speed=28.1;leader_x=30.;minimum_gap=100.;braked=False
        for frame in range(240):
            now=frame*.05
            controller.collision_obstacles=[self.obstacle(x=leader_x,y=0. if now<3. else 3.5)]
            ego=SimpleNamespace(get_velocity=lambda:carla.Vector3D(x=speed),get_transform=lambda:carla.Transform(carla.Location(x=x)))
            snapshot=SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=now),find=lambda actor_id:ego)
            control=controller.run_step(snapshot)
            braked=braked or control.brake>0.
            next_speed=max(0.,speed+(3.*control.throttle-6.*control.brake)*.05)
            x+=(speed+next_speed)*.025;speed=next_speed;leader_x+=14.*.05
            if now<3.:minimum_gap=min(minimum_gap,leader_x-x-5.)
        self.assertTrue(braked);self.assertGreater(minimum_gap,1.)
        self.assertGreater(speed,28.1)
        self.assertIsNone(controller.collision_guard_state)

    def test_accelerates_above_eighty_before_reaching_radio_range(self):
        controller=EmergencyTrajectoryController.__new__(EmergencyTrajectoryController)
        controller.__dict__.update(self.controller().__dict__)
        controller.vehicle=SimpleNamespace(id=7);controller.maximum_speed=math.inf;controller.last_time=None
        controller.integral=controller.previous_error=0.
        controller.curve_speed_limit=lambda target,speed:target
        def lateral(transform,speed,dt):controller.progress=transform.location.x;return 0.
        controller.lateral_control=lateral
        x=0.;speed=55.8/3.6;leader_x=80.;entered=False
        for frame in range(200):
            controller.collision_obstacles=[self.obstacle(x=leader_x)]
            ego=SimpleNamespace(get_velocity=lambda:carla.Vector3D(x=speed),get_transform=lambda:carla.Transform(carla.Location(x=x)))
            snapshot=SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=frame*.05),find=lambda actor_id:ego)
            control=controller.run_step(snapshot)
            self.assertLessEqual(control.brake,1e-9)
            next_speed=speed+3.*control.throttle*.05
            x+=(speed+next_speed)*.025;speed=next_speed;leader_x+=14.*.05
            if leader_x-x<=50.:
                entered=True
                self.assertGreater(speed*3.6,80.)
                break
        self.assertTrue(entered)
