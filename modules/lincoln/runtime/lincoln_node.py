"""Lincoln 组件组装：车辆、传感器、ROS 2、控制和可视化。"""
import json
import threading
import uuid
import carla
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from modules.lincoln.runtime.paths import GEOMETRY_CONFIG_FILE, SENSOR_CONFIG_FILE
from modules.lincoln.vehicle.spawn_vehicle import SpawnVehicleMixin
from modules.lincoln.vehicle.vehicle_cleanup import VehicleCleanupMixin
from modules.lincoln.sensors.sensor_streams import SensorStreamsMixin
from modules.lincoln.visualization.vehicle_markers import VehicleMarkersMixin
from modules.lincoln.visualization.spectator import SpectatorMixin
from modules.lincoln.driving.manual_driving import ManualDrivingMixin
from modules.lincoln.driving.controller import load_driving_controller


CARLA_CLIENT_WORKERS = 2


class LincolnNode(SpawnVehicleMixin, SensorStreamsMixin, VehicleMarkersMixin,
                  SpectatorMixin, ManualDrivingMixin, VehicleCleanupMixin, Node):
    def __init__(self, args):
        self.uid = 'lincoln_' + uuid.uuid4().hex[:10]
        super().__init__(self.uid)
        self.args = args
        self.actors = []
        self.sensors = []
        self.sensor_display_frames = {}
        self.sensor_display_frame = -1
        self.latest_images = {}
        self.last_frame = -1
        self.last_mosaic = 0.
        self.follow = True
        self.free_follow = False
        self.camera_vehicle_position = None
        self.camera_pose = None
        self.camera_time = None
        self.spectator = None
        self.camera_lock = threading.RLock()
        self.control_lock = threading.RLock()
        self.free_pose = None
        self.camera_control_enabled = False
        from modules.lincoln.visualization.camera_ownership import CameraOwnership
        self.camera_owner = CameraOwnership(args.host, args.port, self.uid)
        self.active = None
        self.cruise_speed = None
        self.cruise_integral = 0.
        self.cruise_time = None
        self.autopilot = False
        self.tm_port = args.tm_port
        self.reverse = False
        self.stop_requested = False
        self.rviz = None
        self.driver = None
        self.geometry = json.loads(GEOMETRY_CONFIG_FILE.read_text())
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.markers = self.create_publisher(MarkerArray, f'/{self.uid}/markers', 2)
        self.map_pub = self.create_publisher(MarkerArray, f'/{self.uid}/roads',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mosaic = self.create_publisher(Image, f'/{self.uid}/cameras/mosaic', qos_profile_sensor_data)
        self.client = carla.Client(args.host, args.port, worker_threads=CARLA_CLIENT_WORKERS)
        self.client.set_timeout(args.timeout)
        self.world = self.client.get_world()
        self.follow = self.camera_owner.claim()
        self.camera_control_enabled = self.follow
        self.dt = self.world.get_settings().fixed_delta_seconds or .05
        try:
            self.spawn_car()
            self.driver = load_driving_controller(args.driver, self.client, self.world, self.car, args)
            specs = json.loads(SENSOR_CONFIG_FILE.read_text())
            if args.sensors == 'none':
                specs = []
            elif args.sensors == 'lidar':
                specs = [s for s in specs if s['kind'] == 'lidar']
            self.synchronize_radar_display = (self.world.get_settings().synchronous_mode
                                              and any(s['kind'] == 'lidar' for s in specs))
            for spec in specs:
                self.spawn_sensor(spec)
            self.make_markers()
            self.make_roads()
            self.map_pub.publish(self.roads)
            self.timer = self.create_timer(.02, self.update)
        except BaseException:
            self.cleanup()
            raise
        print(f'VEHICLE {self.uid} CARLA_ID={self.car.id} MAP={self.world.get_map().name}', flush=True)
        print(f'Sensors={len(self.sensors)}; ROS namespace=/{self.uid}; SUMO id is assigned by the bridge.', flush=True)
        if self.synchronize_radar_display:
            count = sum('display_pub' in entry for entry in self.sensors)
            print(f'RViz 雷达同帧显示：{count} 路，等待完整物理帧；原始话题保留配置频率。', flush=True)

