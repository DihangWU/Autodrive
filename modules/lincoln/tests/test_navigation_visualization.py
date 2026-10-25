"""导航 RViz 的宽度、高度、箭头、无传感器模型与窗口生命周期检查。"""
from pathlib import Path
import math
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import carla

try:
    import yaml
    from visualization_msgs.msg import Marker, MarkerArray
    from modules.lincoln.visualization.navigation_route import navigation_route_markers, tm_path_markers
    from modules.lincoln.visualization.navigation_rviz import NavigationRviz
    from modules.lincoln.visualization import rviz_launcher
    from modules.lincoln.visualization.vehicle_markers import VehicleMarkersMixin
    from modules.lincoln.runtime.paths import NAV_RVIZ_TEMPLATE_FILE, RVIZ_TEMPLATE_FILE
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def waypoint(x, y=0., z=0., yaw=0., width=3.5, road=1, lane=-1):
    return SimpleNamespace(transform=carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)),
                           road_id=road, lane_id=lane, s=x, lane_width=width)


@unittest.skipUnless(ROS_AVAILABLE, '需要 source /opt/ros/jazzy/setup.bash')
class GeometryTests(unittest.TestCase):
    def test_lane_width_and_carla_ros_height_and_alpha(self):
        ribbon, arrows = navigation_route_markers([waypoint(0, 10, 5, width=4), waypoint(30, 10, 5, width=4)]).markers
        self.assertAlmostEqual(max(p.y for p in ribbon.points)-min(p.y for p in ribbon.points), 4)
        self.assertTrue(all(abs(p.z-5.012)<1e-6 for p in ribbon.points))
        self.assertGreater(ribbon.color.g, ribbon.color.r)
        self.assertGreater(ribbon.color.a, 0)
        self.assertLess(ribbon.color.a, 1)
        self.assertEqual(len(ribbon.colors), len(ribbon.points)//3)
        self.assertTrue(all(color.a == 1. for color in ribbon.colors))
        self.assertTrue(all(-12 < p.y < -8 for p in arrows.points))
        self.assertTrue(all(0 <= p.x <= 30 for p in arrows.points))

    def test_variable_width_not_fixed_to_default(self):
        ribbon = navigation_route_markers([waypoint(0, width=2), waypoint(10, width=5)]).markers[0]
        self.assertAlmostEqual(abs(ribbon.points[0].y), 1)
        self.assertAlmostEqual(max(abs(p.y) for p in ribbon.points), 2.5)

    def test_curved_and_lane_change_ribbon_faces_point_up(self):
        route = [waypoint(0, yaw=0), waypoint(10, 10, yaw=90), waypoint(0, 20, yaw=180)]
        ribbon = navigation_route_markers(route).markers[0]
        for index in range(0, len(ribbon.points), 3):
            a,b,c=ribbon.points[index:index+3]
            self.assertGreaterEqual((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x), 0)
        changed = navigation_route_markers([waypoint(0), waypoint(12, 3.5, lane=-2)]).markers[0]
        self.assertGreater(max(abs(p.y) for p in changed.points), 3.5)

    def test_grade_does_not_flatten_to_zero(self):
        ribbon = navigation_route_markers([waypoint(0,z=5),waypoint(20,z=12)]).markers[0]
        self.assertGreater(min(p.z for p in ribbon.points), 4.9)
        self.assertGreater(max(p.z for p in ribbon.points), 12)

    def test_template_only_vehicle_lanes_and_route(self):
        config=yaml.safe_load(NAV_RVIZ_TEMPLATE_FILE.read_text())
        displays=config['Visualization Manager']['Displays']
        self.assertEqual(len(displays), 4)
        self.assertTrue(all(d['Class']=='rviz_default_plugins/MarkerArray' for d in displays))
        topics=[d['Topic']['Value'] for d in displays]
        self.assertTrue(any(t.endswith('/navigation/vehicle') for t in topics))
        self.assertTrue(any(t.endswith('/navigation/tm_path') for t in topics))
        self.assertFalse(any('points' in t or 'image' in t or 'mosaic' in t for t in topics))
        self.assertEqual(config['Visualization Manager']['Views'],
                         yaml.safe_load(RVIZ_TEMPLATE_FILE.read_text())['Visualization Manager']['Views'])

    def test_tm_preview_is_red_and_only_connects_real_action_points(self):
        marker = tm_path_markers(carla.Location(x=1, y=2), [carla.Location(x=5, y=3, z=7),
                                                           carla.Location(x=9, y=4, z=8)]).markers[0]
        self.assertEqual(marker.type, Marker.LINE_STRIP)
        self.assertEqual([(p.x,p.y) for p in marker.points], [(1.,-2.),(5.,-3.),(9.,-4.)])
        self.assertGreater(marker.color.r, marker.color.g)
        self.assertGreater(marker.points[1].z, 7.)
        self.assertGreater(marker.lifetime.nanosec, 0)
        self.assertEqual(tm_path_markers(carla.Location(), []).markers[0].action, Marker.DELETE)

    def test_namespace_adapts_wheels_and_lidar_display_frame(self):
        node=SimpleNamespace(uid='lincoln_1234567890',sensors=[],args=SimpleNamespace(no_rviz=True),
                             model=SimpleNamespace(header=SimpleNamespace(frame_id='lincoln_1234567890/lidar_display_base_link')),
                             frame=lambda:'lincoln_1234567890/base_link')
        with tempfile.TemporaryDirectory() as root, patch.object(rviz_launcher,'RUNTIME_DIR',Path(root)), \
                patch.object(rviz_launcher.subprocess,'Popen') as launch:
            rviz_launcher.launch_rviz_template(node,NAV_RVIZ_TEMPLATE_FILE,'lincoln_nav.rviz','nav.log',ros_node_name=node.uid+'_navigation_rviz')
            saved=yaml.safe_load((Path(root)/node.uid/'lincoln_nav.rviz').read_text())
            self.assertIn(node.uid+'_wheels',saved['Visualization Manager']['Displays'][0]['Namespaces'])
            self.assertEqual(saved['Visualization Manager']['Views']['Current']['Target Frame'],node.model.header.frame_id)
            self.assertIn('__node:='+node.uid+'_navigation_rviz',launch.call_args.args[0])


@unittest.skipUnless(ROS_AVAILABLE, '需要 ROS 2 消息类型')
class ViewLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.publishers={}
        def publisher(_type,name,*args):
            result=Mock();result.get_subscription_count.return_value=1
            self.publishers[name]=result
            return result
        self.node=SimpleNamespace(uid='lincoln_1234567890',roads=MarkerArray(markers=[Marker()]),
                                  create_publisher=publisher,map_pub=Mock(),model=Marker(),
                                  wheel_markers=[Marker() for _ in range(4)],mounts=[Marker() for _ in range(31)])
        self.node.map_pub.get_subscription_count.return_value=1
        self.view=NavigationRviz(self.node)

    def tearDown(self):
        self.view.close()

    def wait_until(self,condition):
        deadline=time.monotonic()+2
        while not condition() and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(condition())

    def test_first_start_waits_for_map_generation_and_rviz_subscribers(self):
        entered,release,finished=threading.Event(),threading.Event(),threading.Event()
        self.node.roads=MarkerArray()
        self.view.roads_ready=False
        self.view.vehicle_pub.get_subscription_count.return_value=0
        process=Mock();process.poll.return_value=None
        result=[]
        def generate(*args):entered.set();release.wait(2);return MarkerArray(markers=[Marker()])
        def prepare():
            result.append(self.view.prepare_navigation([waypoint(0),waypoint(20)],None,lambda:False))
            finished.set()
        with patch('modules.lincoln.visualization.navigation_rviz.road_markers',side_effect=generate), \
                patch('modules.lincoln.visualization.navigation_rviz.launch_rviz_template',return_value=process), \
                patch('modules.lincoln.visualization.navigation_rviz.SUBSCRIBER_SETTLE_SECONDS',0.):
            thread=threading.Thread(target=prepare);thread.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertFalse(finished.is_set())
                release.set()
                self.wait_until(lambda:self.view.prepared_generation>0)
                self.assertFalse(finished.is_set())
                self.view.vehicle_pub.get_subscription_count.return_value=1
                self.assertTrue(finished.wait(1))
                self.assertEqual(result,[True])
                # 已有窗口复用，不重新经历首次订阅等待。
                self.view.vehicle_pub.get_subscription_count.return_value=0
                self.assertTrue(self.view.prepare_navigation([waypoint(0),waypoint(30)],None,lambda:False))
            finally:
                release.set();self.view.stopped.set();thread.join(2)

    def test_rviz_failure_and_timeout_do_not_report_ready(self):
        process=Mock();process.poll.return_value=1
        with patch('modules.lincoln.visualization.navigation_rviz.launch_rviz_template',return_value=process):
            with self.assertRaisesRegex(RuntimeError,'exited'):
                self.view.prepare_navigation([waypoint(0),waypoint(20)],None,lambda:False)
        process.poll.return_value=None
        self.view.vehicle_pub.get_subscription_count.return_value=0
        with patch('modules.lincoln.visualization.navigation_rviz.launch_rviz_template',return_value=process), \
                patch('modules.lincoln.visualization.navigation_rviz.PREPARATION_TIMEOUT',.1):
            with self.assertRaisesRegex(RuntimeError,'timed out'):
                self.view.prepare_navigation([waypoint(0),waypoint(20)],None,lambda:False)
        self.assertFalse(self.view.prepare_navigation([waypoint(0),waypoint(20)],None,lambda:True))

    def test_reuses_body_and_wheels_without_mounts(self):
        self.view.publish_vehicle()
        message=self.view.vehicle_pub.publish.call_args.args[0]
        self.assertEqual(len(message.markers),5)
        self.assertIs(message.markers[0],self.node.model)
        self.assertIs(message.markers[1],self.node.wheel_markers[0])

    def test_reuses_one_window_and_closes_it_on_cleanup(self):
        process=Mock();process.poll.return_value=None
        with patch('modules.lincoln.visualization.navigation_rviz.launch_rviz_template',return_value=process) as launch:
            route=[waypoint(0),waypoint(20)]
            self.view.show_route(route,None)
            self.wait_until(lambda:launch.call_count==1)
            self.view.show_route(route,None)
            self.wait_until(lambda:self.view.route_pub.publish.call_count==2)
            self.assertEqual(launch.call_count,1)
            self.view.close()
            process.send_signal.assert_called_once()

    def test_cancellation_during_mesh_generation_does_not_open_window(self):
        entered,release=threading.Event(),threading.Event()
        def generate(*args):entered.set();release.wait(2);return MarkerArray()
        with patch('modules.lincoln.visualization.navigation_rviz.navigation_route_markers',side_effect=generate), \
                patch('modules.lincoln.visualization.navigation_rviz.launch_rviz_template') as launch:
            self.view.show_route([waypoint(0),waypoint(20)],None)
            self.assertTrue(entered.wait(1))
            self.view.clear_route();release.set()
            time.sleep(.05)
            launch.assert_not_called()
            self.assertEqual(self.view.route_pub.publish.call_args.args[0].markers[0].action,Marker.DELETEALL)

    def test_navigation_subscriber_alone_enables_real_wheel_capture(self):
        node=SimpleNamespace(markers=Mock(),navigation_view=self.view,car=Mock(),world=Mock(),
                             wheel_bone_indices=[0,1,2,3],wheel_pose_frames={},wheel_read_failed=False)
        node.markers.get_subscription_count.return_value=0
        poses=[carla.Transform() for _ in range(4)]
        node.car.get_bone_relative_transforms.return_value=poses
        node.world.get_snapshot.return_value=SimpleNamespace(frame=7)
        VehicleMarkersMixin.capture_wheel_poses(node,SimpleNamespace(frame=7))
        self.assertIn(7,node.wheel_pose_frames)
        node.car.get_bone_relative_transforms.assert_called_once()


if __name__=='__main__':unittest.main()
