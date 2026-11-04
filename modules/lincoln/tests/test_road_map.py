"""无标线道路、共享边界和分段间隙的 RViz 地图回归。"""
from types import SimpleNamespace
import unittest
import carla

try:
    from modules.lincoln.visualization.road_map import road_markers
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def waypoint(station, lane=-1, road=51, y=1.75, yaw=0., z=5., marking='NONE'):
    no_mark = SimpleNamespace(type='NONE', width=0., color='White')
    return SimpleNamespace(road_id=road, section_id=0, lane_id=lane, s=station, lane_width=3.5,
        lane_type='Driving', transform=carla.Transform(carla.Location(station,y,z),carla.Rotation(yaw=yaw)),
        left_lane_marking=no_mark,
        right_lane_marking=SimpleNamespace(type=marking, width=.12 if marking!='NONE' else 0., color='White'),
        get_left_lane=lambda:None, get_right_lane=lambda:None)


def generate(points):
    return road_markers(SimpleNamespace(name='unmarked_map',generate_waypoints=lambda distance:points))


@unittest.skipUnless(ROS_AVAILABLE,'需要 ROS 2 消息类型')
class RoadMapTests(unittest.TestCase):
    def test_unmarked_two_way_road_has_outer_edges_without_invented_center_marking(self):
        points=[]
        for station in (0.,.5,1.):
            right=waypoint(station)
            left=waypoint(station,lane=1,y=-1.75,yaw=180.)
            right.get_left_lane=lambda:left
            left.get_left_lane=lambda:right
            points.extend((right,left))
        message=generate(points)
        self.assertEqual(len(message.markers),1)
        marker=message.markers[0]
        self.assertEqual(marker.ns,'carla_road_geometry')
        self.assertEqual(len(marker.points),4*6)
        self.assertTrue(all(abs(point.y)>3.4 for point in marker.points))
        self.assertTrue(all(abs(point.z-5.03)<1e-6 for point in marker.points))
        self.assertLess(marker.color.r,1.)

    def test_existing_paint_stays_in_original_namespace_and_style(self):
        marker=generate([waypoint(0.,marking='Solid'),waypoint(.5,marking='Solid')]).markers[0]
        self.assertEqual(marker.ns,'carla_map')
        self.assertEqual(marker.color.r,1.)
        self.assertEqual(len(marker.points),6)

    def test_disconnected_samples_and_single_waypoint_are_not_joined(self):
        self.assertEqual(generate([waypoint(0.),waypoint(2.)]).markers,[])
        self.assertEqual(generate([waypoint(0.)]).markers,[])

    def test_unmarked_junction_connector_is_visible_and_triangles_face_up(self):
        points=[waypoint(0.),waypoint(.5,y=1.8)]
        for point in points:point.is_junction=True
        marker=generate(points).markers[0]
        self.assertEqual(marker.ns,'carla_road_geometry')
        for index in range(0,len(marker.points),3):
            a,b,c=marker.points[index:index+3]
            self.assertGreater((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x),0.)
        self.assertEqual(len(marker.colors),len(marker.points)//3)


if __name__=='__main__':unittest.main()
