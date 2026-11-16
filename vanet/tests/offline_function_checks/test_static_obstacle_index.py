"""静态缓存生命周期与空间筛选必须保持完整算法的碰撞判断。"""
import copy
import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import carla
from vanet.lincoln.emergency_driving_control.collision_guard import collision_speed_limit
from vanet.lincoln.emergency_driving_control.control_configuration import DEFAULT_COLLISION_GUARD
from vanet.lincoln.emergency_route_planning.borrowing_geometry import trajectory_clear
from vanet.npc.stationary_traffic_obstacles.carla_obstacle_cache import CarlaObstacleCache
from vanet.npc.stationary_traffic_obstacles.spatial_index import ObstacleSet, StaticObstacleIndex


def obstacle(identifier, x, y, z=0., yaw=0., velocity=(0.,0.,0.)):
    return dict(id=identifier,kind='vehicle.audi.a2',center=(x,y,z),velocity=velocity,
                yaw=yaw,length=2.,width=1.,height=1.)


class StaticObstacleIndexTests(unittest.TestCase):
    def test_guard_and_borrowing_match_complete_checks_in_curves_and_multiple_layers(self):
        rng = random.Random(739)
        for trial in range(24):
            curvature = (trial % 3 - 1) * .003
            samples = [SimpleNamespace(center=(i*.5, curvature*(i*.5)**2, 0.),
                forward=(math.cos(math.atan(2.*curvature*i*.5)), math.sin(math.atan(2.*curvature*i*.5)),0.))
                for i in range(401)]
            distances = [0.]
            for a,b in zip(samples,samples[1:]):
                distances.append(distances[-1]+math.dist(a.center,b.center))
            static = [obstacle(i,rng.uniform(-30.,240.),rng.uniform(-80.,80.),
                z=rng.choice([0.,0.,8.]),yaw=rng.uniform(-math.pi,math.pi)) for i in range(90)]
            # Cover the braking boundary, neighbouring lanes, and a latched guard.
            static.append(obstacle(1000,trial*4.,0.,yaw=.15))
            dynamic = [obstacle(2000,100.,-20.,velocity=(-10.,4.,0.))]
            indexed = ObstacleSet(dynamic,StaticObstacleIndex(static))
            controller = SimpleNamespace(collision_configuration=dict(DEFAULT_COLLISION_GUARD),
                collision_active_actors={1000},samples=samples,distances=distances,progress=trial*.7,
                front=2.5,half_width=1.,wait_reason='')
            naive = copy.deepcopy(controller)
            speed = (trial % 8)*8.
            expected = collision_speed_limit(naive,dynamic+static,90.,speed)
            actual = collision_speed_limit(controller,indexed,90.,speed)
            self.assertAlmostEqual(actual,expected)
            self.assertEqual(controller.collision_active_actors,naive.collision_active_actors)
            self.assertEqual(controller.collision_guard_state,naive.collision_guard_state)
            self.assertEqual(controller.collision_minimum_brake,naive.collision_minimum_brake)
            self.assertEqual(trajectory_clear(samples,indexed,max(2.,speed),1.,2.5),
                             trajectory_clear(samples,dynamic+static,max(2.,speed),1.,2.5))

    def test_far_obstacles_produce_no_narrow_phase_candidates(self):
        index = StaticObstacleIndex([obstacle(i,i*6.,100.) for i in range(400)])
        samples = [SimpleNamespace(center=(i*.5,0.,0.)) for i in range(401)]
        controller = SimpleNamespace(samples=samples,distances=[i*.5 for i in range(401)],progress=0.,
            collision_configuration=dict(DEFAULT_COLLISION_GUARD),front=2.5,half_width=1.)
        self.assertEqual(list(ObstacleSet([],index).collision_candidates(controller,40.,0)),[])
        self.assertEqual(list(index.near((0.,0.,0.),5.)),[])

    def test_cache_refreshes_on_actor_lifecycle_and_world_change(self):
        static = SimpleNamespace(id=1,type_id='vehicle.audi.a2',
            attributes={'role_name':'vanet_obstacle_highway_41_3_001'},
            bounding_box=carla.BoundingBox(carla.Location(),carla.Vector3D(2.,1.,1.)))
        moving = SimpleNamespace(id=2,type_id='vehicle.audi.a2',attributes={},bounding_box=static.bounding_box)
        class Snapshot:
            def __init__(self, now, present):
                self.timestamp=SimpleNamespace(elapsed_seconds=now)
                self.present=present
            def __iter__(self):
                return iter(SimpleNamespace(id=value) for value in self.present)
            def find(self, identifier):
                if identifier not in self.present:return None
                x=20. if identifier==1 else self.timestamp.elapsed_seconds*10.
                return SimpleNamespace(get_transform=lambda:carla.Transform(carla.Location(x=x)),
                                       get_velocity=lambda:carla.Vector3D())
        world = SimpleNamespace(id=3,get_actors=Mock(return_value=[static,moving]))
        cache = CarlaObstacleCache(SimpleNamespace(car=SimpleNamespace(id=7),world=world))
        first=cache.collect(Snapshot(1.,[1,2]))
        second=cache.collect(Snapshot(1.05,[1,2]))
        self.assertIs(first.static_index,second.static_index)
        self.assertAlmostEqual(second.dynamic[0]['velocity'][0],10.)
        self.assertEqual(world.get_actors.call_count,1)
        world.get_actors.return_value=[moving]
        result=cache.collect(Snapshot(1.1,[2]))
        self.assertEqual(len(result.static_index.obstacles),0)
        self.assertEqual(world.get_actors.call_count,2)
        world.id=4
        cache.collect(Snapshot(.05,[2]))
        self.assertEqual(world.get_actors.call_count,3)


if __name__=='__main__':
    unittest.main()
