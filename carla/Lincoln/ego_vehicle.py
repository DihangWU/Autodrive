"""One physical Lincoln, native sensors, ROS2 visualization and mouse driving.

The SUMO bridge owns simulation ticks. This client never calls world.tick().
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json
import math
import queue
import random
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
import uuid

import carla
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

PROJECT_ROOT = Path(__file__).resolve().parents[2]

# Match Demo's camera order; optional extra rows remain explicitly empty.
CAMERA_ROWS = (('roof_sensor_rig/standard', 'roof_sensor_rig/tele'),
               ('surround_camera_rig/rear', 'roof_sensor_rig/wide'),
               ('surround_camera_rig/b_left', 'surround_camera_rig/b_right'),
               ('surround_camera_rig/left', 'surround_camera_rig/right'))


def camera_mosaic(images, specs, rows):
    tile_height, tile_width, title_height = 392, 640, 32
    canvas = np.zeros((rows * tile_height, 2 * tile_width, 3), dtype=np.uint8)
    for row in range(rows):
        for col in range(2):
            tile = canvas[row * tile_height:(row + 1) * tile_height,
                          col * tile_width:(col + 1) * tile_width]
            name = CAMERA_ROWS[row][col] if row < len(CAMERA_ROWS) else None
            title = 'UNUSED - no camera' if name is None else name.split('/')[-1].upper()
            if name in specs:
                title += f' | {math.degrees(specs[name]["fov"]):.0f} deg'
            if name in images:
                tile[title_height:] = images[name]
            elif name is not None:
                title += ' | waiting for data'
            cv2.putText(tile, title, (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    return canvas


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def set_rotation(message, angles):
    message.x, message.y, message.z, message.w = quaternion(*angles)


def local_pose(spec, geometry):
    pose = list(spec['pose'])
    cx, cy, cz = geometry['center']
    pose[0] += cx
    pose[1] += cy
    pose[2] += cz - geometry['height'] / 2
    return pose


def camera_info(spec, header):
    info = CameraInfo(header=header, width=spec['width'], height=spec['height'])
    focal = spec['width'] / (2 * math.tan(spec['fov'] / 2))
    cx, cy = spec['width'] / 2, spec['height'] / 2
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    info.k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
    return info


def sensor_points(data, kind):
    raw = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    if kind == 'lidar':
        result = raw.copy()
        result[:, 1] *= -1
    else:
        # CARLA radar raw fields: velocity, azimuth, altitude, depth.
        velocity, azimuth, altitude, depth = raw.T
        horizontal = depth * np.cos(altitude)
        result = np.column_stack((horizontal * np.cos(azimuth),
                                  -horizontal * np.sin(azimuth),
                                  depth * np.sin(altitude), velocity))
    return np.asarray(result, dtype='<f4')


class Lincoln(Node):
    def __init__(self, args):
        self.uid = 'lincoln_' + uuid.uuid4().hex[:10]
        super().__init__(self.uid)
        self.args = args
        self.actors = []
        self.sensors = []
        self.latest_images = {}
        self.last_frame = -1
        self.last_mosaic = 0.
        self.follow = True
        self.camera_pose = None
        self.camera_time = None
        self.spectator = None
        self.camera_lock = threading.RLock()
        self.free_pose = None
        self.camera_control_enabled = False
        from camera_ownership import CameraOwnership
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
        self.geometry = json.loads((PROJECT_ROOT / 'carla/Lincoln/config/vehicle_geometry.json').read_text())
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.markers = self.create_publisher(MarkerArray, f'/{self.uid}/markers', 2)
        self.map_pub = self.create_publisher(MarkerArray, f'/{self.uid}/roads',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mosaic = self.create_publisher(Image, f'/{self.uid}/cameras/mosaic', qos_profile_sensor_data)
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(30)
        self.world = self.client.get_world()
        self.follow = self.camera_owner.claim()
        self.camera_control_enabled = self.follow
        self.dt = self.world.get_settings().fixed_delta_seconds or .05
        try:
            self.spawn_car()
            specs = json.loads((PROJECT_ROOT / 'carla/Lincoln/config/sensors.json').read_text())
            if args.sensors == 'lidar':
                specs = [s for s in specs if s['kind'] == 'lidar']
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

    def spawn_car(self):
        bp = self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        bp.set_attribute('role_name', self.uid)
        bp.set_attribute('color', '30,90,230')
        points = self.world.get_map().get_spawn_points()
        if self.args.pose:
            x, y, z, yaw = self.args.pose
            points = [carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))]
        elif self.args.spawn_index is not None:
            if not 0 <= self.args.spawn_index < len(points):
                raise ValueError(f'spawn-index must be between 0 and {len(points) - 1}')
            points = [points[self.args.spawn_index]]
        else:
            occupied = [a.get_location() for a in self.world.get_actors().filter('vehicle.*')]
            points = [p for p in points if all(p.location.distance(v) > 7 for v in occupied)]
            random.shuffle(points)
        for transform in points:
            self.car = self.world.try_spawn_actor(bp, transform)
            if self.car:
                self.actors.append(self.car)
                # Newly spawned vehicles are manual by default. Calling
                # set_autopilot(False) would unnecessarily create/contact a TM.
                self.car.apply_control(carla.VehicleControl(brake=1.))
                # spawn RPC can complete before the client receives the next
                # actor snapshot. is_alive is temporarily false in that gap.
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    snapshot = self.world.get_snapshot()
                    if snapshot.find(self.car.id) is not None and self.car.is_alive:
                        return
                    time.sleep(.05)
                self.car.destroy()
                raise RuntimeError('Spawned vehicle never appeared in CARLA snapshots; check that the SUMO bridge is advancing frames.')
        raise RuntimeError('No free spawn point. Choose another --spawn-index or --pose.')

    def frame(self, name='base_link'):
        return self.uid + '/' + name

    def spawn_sensor(self, spec):
        kind = spec['kind']
        bp = self.world.get_blueprint_library().find({'camera': 'sensor.camera.rgb',
            'lidar': 'sensor.lidar.ray_cast', 'radar': 'sensor.other.radar'}[kind])
        rate = min(spec['rate'], 1 / self.dt)
        period = max(1, round(1 / rate / self.dt))
        bp.set_attribute('sensor_tick', str(max(0., period * self.dt - 1e-6)))
        if kind == 'camera':
            for key, value in [('image_size_x', spec['width']), ('image_size_y', spec['height']),
                               ('fov', math.degrees(spec['fov'])), ('motion_blur_intensity', 0),
                               ('lens_k', 0), ('lens_kcube', 0)]:
                bp.set_attribute(key, str(value))
        else:
            bp.set_attribute('range', str(spec['range']))
            bp.set_attribute('horizontal_fov', str(math.degrees(spec['hmax'] - spec['hmin'])))
            bp.set_attribute('points_per_second', str(round(spec['samples'] * spec['channels'] / self.dt)))
            if kind == 'lidar':
                for key, value in [('channels', spec['channels']), ('rotation_frequency', 1 / self.dt),
                                   ('upper_fov', math.degrees(spec['vmax'])),
                                   ('lower_fov', math.degrees(spec['vmin'])),
                                   ('dropoff_general_rate', 0), ('dropoff_zero_intensity', 0)]:
                    bp.set_attribute(key, str(value))
            else:
                bp.set_attribute('vertical_fov', str(math.degrees(spec['vmax'] - spec['vmin'])))
        pose = local_pose(spec, self.geometry)
        x, y, z, roll, pitch, yaw = pose
        transform = carla.Transform(carla.Location(x=x, y=-y, z=z),
            carla.Rotation(roll=math.degrees(roll), pitch=-math.degrees(pitch), yaw=-math.degrees(yaw)))
        actor = self.world.spawn_actor(bp, transform, attach_to=self.car)
        self.actors.append(actor)
        entry = dict(spec=spec, actor=actor, pose=pose, queue=queue.Queue(maxsize=1), received=0, published=0)
        topic = f'/{self.uid}/{spec["name"]}'
        entry['pub'] = self.create_publisher(Image if kind == 'camera' else PointCloud2,
            topic + ('/image' if kind == 'camera' else '/points'), qos_profile_sensor_data)
        if kind == 'camera':
            entry['info'] = self.create_publisher(CameraInfo, topic + '/camera_info', qos_profile_sensor_data)
        self.sensors.append(entry)
        def receive(data):
            entry['received'] += 1
            try:
                entry['queue'].put_nowait(data)
            except queue.Full:
                try:
                    entry['queue'].get_nowait()
                except queue.Empty:
                    pass
                try:
                    entry['queue'].put_nowait(data)
                except queue.Full:
                    pass
        actor.listen(receive)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = self.frame(), self.frame(spec['name'])
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pose[:3]
        set_rotation(tf.transform.rotation, pose[3:])
        transforms = [tf]
        if kind == 'camera':
            optical = TransformStamped()
            optical.header.frame_id = tf.child_frame_id
            optical.child_frame_id = tf.child_frame_id + '/optical'
            set_rotation(optical.transform.rotation, (-math.pi / 2, 0, -math.pi / 2))
            transforms.append(optical)
        self.static_tf.sendTransform(transforms)

    def make_markers(self):
        self.model = Marker()
        self.model.header.frame_id = self.frame()
        self.model.ns, self.model.id = self.uid, 0
        self.model.type, self.model.action = Marker.MESH_RESOURCE, Marker.ADD
        self.model.mesh_resource = (PROJECT_ROOT / 'carla/Lincoln/assets/lincoln.dae').as_uri()
        self.model.mesh_use_embedded_materials = True
        cx, cy, cz = self.geometry['center']
        self.model.pose.position.x, self.model.pose.position.y = cx, cy
        self.model.pose.position.z = cz - self.geometry['height'] / 2
        self.model.pose.orientation.w = 1.
        self.model.scale.x = self.model.scale.y = self.model.scale.z = 1.
        self.model.color.a = 1.
        self.mounts = []
        for i, entry in enumerate(self.sensors, 1):
            m = Marker()
            m.header.frame_id = self.frame(entry['spec']['name'])
            m.ns, m.id, m.type, m.action = self.uid, i, Marker.SPHERE, Marker.ADD
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = .08
            m.color.r, m.color.g, m.color.b, m.color.a = (1., .6, .1, 1.)
            self.mounts.append(m)

    def make_roads(self):
        from map_visualization import road_markers
        self.roads = road_markers(self.world.get_map())

    def update(self):
        if not self.car.is_alive:
            self.stop_requested = True
            return
        snapshot = self.world.get_snapshot()
        if snapshot.frame != self.last_frame:
            self.last_frame = snapshot.frame
            actor = snapshot.find(self.car.id)
            if actor:
                transform = actor.get_transform()
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'map', self.frame()
                tf.header.stamp = stamp(snapshot.timestamp.elapsed_seconds)
                loc, rot = transform.location, transform.rotation
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = loc.x, -loc.y, loc.z
                set_rotation(tf.transform.rotation, (math.radians(rot.roll), -math.radians(rot.pitch), -math.radians(rot.yaw)))
                self.tf.sendTransform(tf)
                self.markers.publish(MarkerArray(markers=[self.model] + self.mounts))
        for entry in self.sensors:
            try:
                data = entry['queue'].get_nowait()
            except queue.Empty:
                continue
            spec = entry['spec']
            entry['published'] += 1
            frame = self.frame(spec['name'])
            if spec['kind'] == 'camera':
                rgba = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(data.height, data.width, 4)
                rgb = rgba[:, :, [2, 1, 0]].copy()
                msg = Image(height=data.height, width=data.width, encoding='rgb8', step=data.width * 3, data=rgb.tobytes())
                msg.header.frame_id, msg.header.stamp = frame + '/optical', stamp(data.timestamp)
                entry['pub'].publish(msg)
                entry['info'].publish(camera_info(spec, msg.header))
                self.latest_images[spec['name']] = rgb
            else:
                points = sensor_points(data, spec['kind'])
                msg = PointCloud2(height=1, width=len(points), is_bigendian=False, point_step=16,
                    row_step=len(points) * 16, is_dense=False, data=points.tobytes())
                msg.header.frame_id, msg.header.stamp = frame, stamp(data.timestamp)
                msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z', 'intensity' if spec['kind'] == 'lidar' else 'velocity'))]
                entry['pub'].publish(msg)
        if self.latest_images and time.monotonic() - self.last_mosaic > .1:
            specs = {e['spec']['name']: e['spec'] for e in self.sensors if e['spec']['kind'] == 'camera'}
            canvas = camera_mosaic(self.latest_images, specs, self.args.camera_rows)
            msg = Image(height=canvas.shape[0], width=canvas.shape[1], encoding='rgb8',
                        step=canvas.shape[1] * 3, data=canvas.tobytes())
            msg.header.stamp = stamp(snapshot.timestamp.elapsed_seconds)
            self.mosaic.publish(msg)
            self.last_mosaic = time.monotonic()

    def set_camera_mode(self, follow, acquire=True, replace=True):
        with self.camera_lock:
            if acquire:
                acquire = self.camera_owner.claim(replace=replace)
            self.camera_control_enabled = acquire
            self.follow = follow
            self.camera_pose = None
            self.camera_time = None
            if self.spectator is None:
                self.spectator = self.world.get_spectator()
            if not follow:
                t = self.spectator.get_transform()
                self.free_pose = (t.location.x, t.location.y, t.location.z, t.rotation.pitch, t.rotation.yaw)
            else:
                self.update_camera()

    def update_camera(self, snapshot=None):
        with self.camera_lock:
            with self.camera_owner.access() as owned:
                if not owned:
                    self.camera_control_enabled = False
                    self.follow = False
                    return
                if self.camera_control_enabled:
                    self._update_camera(snapshot)

    def _update_camera(self, snapshot=None):
        # Match the simulation cadence, rather than the unrelated GUI cadence.
        if not self.follow:
            self.camera_pose = None
            self.camera_time = None
            if self.free_pose is not None and self.camera_control_enabled:
                x, y, z, pitch, yaw = self.free_pose
                self.spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                            carla.Rotation(pitch=pitch, yaw=yaw)))
            return
        actor = (snapshot or self.world.get_snapshot()).find(self.car.id)
        if actor is None:
            return
        now = time.monotonic()
        transform = actor.get_transform()
        loc, rot = transform.location, transform.rotation
        yaw = math.radians(rot.yaw)
        target = (loc.x - 8 * math.cos(yaw), loc.y - 8 * math.sin(yaw), loc.z + 4, rot.yaw)
        if self.camera_pose is None:
            self.camera_pose = target
        else:
            dt = max(0., min(now - self.camera_time, .2))
            alpha = 1. - math.exp(-dt / .10)
            old = self.camera_pose
            yaw_delta = (target[3] - old[3] + 180.) % 360. - 180.
            heading = old[3] + alpha * yaw_delta
            # Keep a fixed vehicle-relative offset; position smoothing causes
            # the car to drift/jump relative to the camera during acceleration.
            heading_rad = math.radians(heading)
            self.camera_pose = (loc.x - 8 * math.cos(heading_rad),
                                loc.y - 8 * math.sin(heading_rad), loc.z + 4, heading)
        self.camera_time = now
        if self.spectator is None:
            self.spectator = self.world.get_spectator()
        x, y, z, heading = self.camera_pose
        self.spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                   carla.Rotation(pitch=-15, yaw=heading)))

    def set_autopilot(self, enabled):
        if enabled == self.autopilot:
            return
        if enabled:
            manager = self.client.get_trafficmanager(self.tm_port)
            manager.set_synchronous_mode(self.world.get_settings().synchronous_mode)
        self.car.set_autopilot(enabled, self.tm_port)
        self.autopilot = enabled
        self.active = None
        self.cruise_speed = None
        self.reverse = False

    def drive(self):
        if self.autopilot:
            return  # Only Traffic Manager may control the car in this mode.
        active = self.active or ''
        if 'go' in active or 'brake' in active:
            self.cruise_speed = None
        throttle = .45 if 'go' in active else 0.
        if self.cruise_speed is not None:
            velocity = self.car.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            now = self.world.get_snapshot().timestamp.elapsed_seconds
            dt = 0. if self.cruise_time is None else max(0., min(now - self.cruise_time, .2))
            self.cruise_time = now
            error = self.cruise_speed - speed
            self.cruise_integral = max(0., min(.85, self.cruise_integral + .08 * error * dt))
            throttle = max(0., min(.85, self.cruise_integral + .25 * error))
        control = carla.VehicleControl(throttle=throttle,
            steer=-.45 if 'left' in active else (.45 if 'right' in active else 0.),
            brake=1. if 'brake' in active else 0., reverse=self.reverse)
        if self.car.is_alive:
            self.car.apply_control(control)

    def cleanup(self):
        self.camera_owner.release()
        if self.autopilot and self.car.is_alive:
            self.set_autopilot(False)
        if self.sensors:
            try:
                runtime = Path(__file__).resolve().parent / 'runtime'
                runtime.mkdir(parents=True, exist_ok=True)
                record = dict(event='SENSOR_COUNTS', timestamp=time.time(), vehicle=self.uid,
                              counts={e['spec']['name']: [e['received'], e['published']]
                                      for e in self.sensors})
                with (runtime / f'{self.uid}.log').open('a', encoding='utf-8') as log:
                    log.write(json.dumps(record, ensure_ascii=False) + '\n')
            except OSError as error:
                self.get_logger().warning(f'Cannot write sensor statistics log: {error}')
        if self.rviz and self.rviz.poll() is None:
            self.rviz.send_signal(signal.SIGINT)
            try:
                self.rviz.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.rviz.kill()
                self.rviz.wait()
        for actor in reversed(self.actors):
            try:
                if actor.type_id.startswith('sensor.') and actor.is_alive:
                    actor.stop()
            except RuntimeError:
                pass
        # is_alive comes from the client's local snapshot and can briefly be
        # false even though the server still owns the actor. Destroy by ID.
        if self.actors:
            try:
                responses = self.client.apply_batch_sync(
                    [carla.command.DestroyActor(actor.id) for actor in reversed(self.actors)], False)
                for response in responses:
                    if response.error:
                        print(f'CARLA actor cleanup: {response.error}', flush=True)
            except RuntimeError as error:
                print(f'CARLA actor cleanup failed: {error}', flush=True)
        self.actors.clear()


def rviz_config(node):
    rear = {'Class': 'rviz_default_plugins/ThirdPersonFollower', 'Name': '车后跟随',
            'Target Frame': node.frame(), 'Distance': 15., 'Pitch': .4, 'Yaw': 3.14,
            'Focal Point': {'X': 0., 'Y': 0., 'Z': 1.}, 'Near Clip Distance': .05}
    displays = [dict(Class='rviz_default_plugins/Grid', Name='Grid', Enabled=True, **{'Cell Size': 5}),
        dict(Class='rviz_default_plugins/MarkerArray', Name='Lincoln and mounts', Enabled=True,
             Topic=dict(Value=f'/{node.uid}/markers', **{'Reliability Policy': 'Reliable'})),
        dict(Class='rviz_default_plugins/MarkerArray', Name='CARLA road surface and lane markings', Enabled=True,
             Topic=dict(Value=f'/{node.uid}/roads', **{'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local'})),
        dict(Class='rviz_default_plugins/Image', Name=f'8 cameras - 2 columns x {node.args.camera_rows} rows',
             Enabled=True, Topic=dict(Value=f'/{node.uid}/cameras/mosaic', **{'Reliability Policy': 'Best Effort'}))]
    rig_groups = {}
    ultrasonic_groups = {}
    for entry in node.sensors:
        spec = entry['spec']
        rig, sensor_name = spec['name'].split('/', 1)
        if rig not in rig_groups:
            rig_groups[rig] = dict(Class='rviz_common/Group', Name=rig,
                                   Enabled=True, Displays=[])
            displays.append(rig_groups[rig])
        if spec['kind'] == 'camera':
            # The mosaic stays visible; individual image docks are opt-in.
            display = dict(Class='rviz_default_plugins/Image', Name=sensor_name, Enabled=False,
                           Topic=dict(Value=f'/{node.uid}/{spec["name"]}/image',
                                      **{'Reliability Policy': 'Best Effort'}))
        else:
            display = dict(Class='rviz_default_plugins/PointCloud2', Name=sensor_name, Enabled=True,
                Topic=dict(Value=f'/{node.uid}/{spec["name"]}/points', **{'Reliability Policy': 'Best Effort'}),
                **{'Size (m)': .04, 'Style': 'Points', 'Color Transformer': 'AxisColor', 'Axis': 'Z', 'Decay Time': 0})
        parent = rig_groups[rig]
        if rig == 'ultrasonic_rig':
            direction = ('front' if sensor_name.startswith('front_') else
                         'rear' if sensor_name.startswith('rear_') else
                         'left' if sensor_name.startswith('side_left_') else
                         'right' if sensor_name.startswith('side_right_') else None)
            if direction is None:
                raise ValueError(f'Unknown ultrasonic direction: {sensor_name}')
            if direction not in ultrasonic_groups:
                ultrasonic_groups[direction] = dict(Class='rviz_common/Group', Name=direction,
                                                     Enabled=True, Displays=[])
                parent['Displays'].append(ultrasonic_groups[direction])
            parent = ultrasonic_groups[direction]
        parent['Displays'].append(display)
    return {'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays',
                        'Property Tree Widget': {'Expanded': []}},
                       {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {'Global Options': {'Fixed Frame': 'map', 'Frame Rate': 20},
            'Tools': [{'Class': 'rviz_default_plugins/MoveCamera'},
                      {'Class': 'rviz_default_plugins/Interact'},
                      {'Class': 'rviz_default_plugins/Select'}],
            'Displays': displays, 'Views': {'Current': rear,
                                          'Saved': []}}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm-port', type=int, default=8000, help='Shared CARLA Traffic Manager port.')
    position = parser.add_mutually_exclusive_group()
    position.add_argument('--spawn-index', type=int)
    position.add_argument('--pose', type=float, nargs=4, metavar=('X', 'Y', 'Z', 'YAW'))
    parser.add_argument('--sensors', choices=['all', 'lidar'], default='all')
    parser.add_argument('--camera-rows', type=int, choices=[4, 6], default=4,
                        help='Two-column camera mosaic; six rows reserves two empty rows.')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--rviz-config', type=Path, default=PROJECT_ROOT / 'carla/Lincoln/rviz/lincoln.rviz',
                        help='Saved RViz layout; vehicle topic/TF namespaces are adapted at startup.')
    parser.add_argument('--headless', action='store_true', help='No mouse panel (for verification).')
    parser.add_argument('--duration', type=float, default=0, help='Exit after this many real seconds; 0 keeps running.')
    args = parser.parse_args()
    rclpy.init()
    node = None
    executor = None
    worker = None
    camera_worker = None
    free_camera = None
    camera_stop = threading.Event()
    started = time.monotonic()
    try:
        node = Lincoln(args)
        if not args.no_rviz:
            import tempfile
            import yaml
            config = Path(tempfile.gettempdir()) / (node.uid + '.rviz')
            template = args.rviz_config.expanduser()
            if not template.is_absolute():
                template = PROJECT_ROOT / template
            template = template.resolve()
            saved = yaml.safe_load(template.read_text())
            if not isinstance(saved, dict) or 'Visualization Manager' not in saved:
                raise ValueError(f'Invalid RViz configuration: {template}')
            def adapt_namespace(value):
                if isinstance(value, str):
                    return re.sub(r'(?<![A-Za-z0-9_])lincoln_[0-9a-f]{10}(?=/|$)', node.uid, value)
                if isinstance(value, list):
                    return [adapt_namespace(item) for item in value]
                if isinstance(value, dict):
                    adapted = {adapt_namespace(key): adapt_namespace(item) for key, item in value.items()}
                    topic = adapted.get('Topic')
                    if isinstance(topic, dict):
                        name = topic.get('Value', '')
                        if isinstance(name, str) and name.startswith(f'/{node.uid}/') and name.endswith(('/points', '/image', '/camera_info', '/cameras/mosaic')):
                            topic['Reliability Policy'] = 'Best Effort'
                    return adapted
                return value
            config.write_text(yaml.safe_dump(adapt_namespace(saved), sort_keys=False, allow_unicode=True))
            print(f'RVIZ_CONFIG template={template} instance={config}', flush=True)
            node.rviz = subprocess.Popen(['rviz2', '-d', str(config)])
        def done():
            return node.stop_requested or not rclpy.ok() or (args.duration > 0 and time.monotonic() - started >= args.duration)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        def publish_sensors():
            try:
                executor.spin()
            except Exception as error:
                print(f'Sensor worker stopped: {error}', flush=True)
                node.stop_requested = True
        worker = threading.Thread(target=publish_sensors, name='lincoln-sensors')
        worker.start()
        def follow_camera():
            try:
                while not camera_stop.is_set():
                    try:
                        snapshot = node.world.wait_for_tick(.5)
                    except RuntimeError:
                        continue
                    if not camera_stop.is_set():
                        node.update_camera(snapshot)
            except Exception as error:
                print(f'Camera worker stopped: {error}', flush=True)
                node.stop_requested = True
        camera_worker = threading.Thread(target=follow_camera, name='lincoln-camera')
        camera_worker.start()
        if args.headless:
            while not done():
                node.drive()
                time.sleep(.03)
        else:
            window = tk.Tk()
            window.title('Lincoln mouse drive — ' + node.uid)
            tk.Label(window, text=f'{node.uid}\n鼠标按住驾驶；W 油门 / S 刹车 / A D 转向 / Q 切换前进倒车'
                                  '\n油门右键：定速／取消；非定速时松开或失焦收油滑行').pack(pady=8)
            pressed_keys = set()
            pending_releases = {}
            def stop_driving(event=None):
                pressed_keys.clear()
                for callback in pending_releases.values():
                    window.after_cancel(callback)
                pending_releases.clear()
                node.active = None
                node.drive()
            grid = tk.Frame(window)
            grid.pack()
            def activate(action):
                node.set_autopilot(False)
                stop_driving()
                node.active = action
                node.drive()
            def toggle_cruise(event=None):
                node.set_autopilot(False)
                if node.cruise_speed is not None:
                    node.cruise_speed = None
                else:
                    velocity = node.car.get_velocity()
                    node.cruise_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                    node.cruise_integral = node.car.get_control().throttle
                    node.cruise_time = None
                stop_driving()
                return 'break'
            for i, (label, action) in enumerate([('油门', 'go'), ('左转', 'left'),
                                               ('刹车', 'brake'), ('右转', 'right')]):
                button = tk.Button(grid, text=label, width=12, height=3)
                if i == 0:
                    button.grid(row=0, column=1, padx=4, pady=4)
                else:
                    button.grid(row=1, column=i - 1, padx=4, pady=4)
                button.bind('<ButtonPress-1>',
                            lambda event, a=action: activate(a))
                if action == 'go':
                    button.bind('<ButtonPress-3>', toggle_cruise)
            speed_input = tk.Frame(grid)
            speed_input.grid(row=0, column=2, padx=4, pady=4)
            tk.Label(speed_input, text='目标速度 km/h').pack()
            target_entry = tk.Entry(speed_input, width=12)
            target_entry.pack()
            target_hint = tk.Label(speed_input, text='输入后按 Enter')
            target_hint.pack()
            def set_target_speed(event=None):
                try:
                    kmh = float(target_entry.get().strip())
                    if not math.isfinite(kmh) or kmh < 0:
                        raise ValueError
                except ValueError:
                    target_hint.config(text='请输入非负数字', fg='red')
                    return 'break'
                node.set_autopilot(False)
                node.cruise_speed = kmh / 3.6
                node.cruise_integral = node.car.get_control().throttle
                node.cruise_time = None
                stop_driving()
                target_hint.config(text=f'目标 {kmh:g} km/h', fg='black')
                window.focus_set()
                return 'break'
            target_entry.bind('<Return>', set_target_speed)
            target_entry.bind('<KP_Enter>', set_target_speed)
            def toggle_autopilot():
                stop_driving()
                node.set_autopilot(not node.autopilot)
            autopilot_button = tk.Button(grid, text='开启自动驾驶', width=12, height=3,
                                         command=toggle_autopilot)
            autopilot_button.grid(row=0, column=0, padx=4, pady=4)
            def release(event=None):
                stop_driving()
            window.bind_all('<ButtonRelease-1>', release)
            def focus_out(event=None):
                # Moving focus between buttons must not cancel a command.
                window.after_idle(lambda: stop_driving() if window.focus_get() is None else None)
            window.bind('<FocusOut>', focus_out)
            window.bind('<Leave>', lambda event: release() if event.widget == window else None)
            def reverse():
                node.set_autopilot(False)
                node.cruise_speed = None
                node.active = None
                velocity = node.car.get_velocity()
                if math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) > .5:
                    return
                node.reverse = not node.reverse
                gear.config(text='当前：倒车' if node.reverse else '当前：前进')
            gear = tk.Button(window, text='当前：前进', command=reverse)
            gear.pack(pady=5)
            def keyboard_drive():
                if pressed_keys & {'w', 'a', 's', 'd'}:
                    node.set_autopilot(False)
                parts = []
                if 's' in pressed_keys:
                    parts.append('brake')
                elif 'w' in pressed_keys:
                    parts.append('go')
                if ('a' in pressed_keys) != ('d' in pressed_keys):
                    parts.append('left' if 'a' in pressed_keys else 'right')
                node.active = '-'.join(parts) or None
                node.drive()
            def key_press(event):
                key = event.keysym.lower()
                if key not in ('w', 'a', 's', 'd', 'q'):
                    return
                callback = pending_releases.pop(key, None)
                if callback is not None:
                    window.after_cancel(callback)
                if key not in pressed_keys:
                    pressed_keys.add(key)
                    if key == 'q':
                        reverse()
                    keyboard_drive()
                return 'break'
            def key_release(event):
                key = event.keysym.lower()
                if key not in pressed_keys:
                    return
                def finish():
                    pending_releases.pop(key, None)
                    pressed_keys.discard(key)
                    keyboard_drive()
                # X11 auto-repeat can emit release/press pairs for a held key.
                if key not in pending_releases:
                    pending_releases[key] = window.after(20, finish)
                return 'break'
            window.bind('<KeyPress>', key_press)
            window.bind('<KeyRelease>', key_release)
            from free_camera_controls import FreeCameraControls
            def toggle_camera_mode():
                stop_driving()
                node.set_camera_mode(not node.follow)
                free_camera.mode_changed()
                camera_button.config(text='释放视角' if node.follow else '跟随本车')
            camera_button = tk.Button(window, text='释放视角' if node.follow else '跟随本车',
                                      command=toggle_camera_mode)
            camera_button.pack()
            node.set_camera_mode(node.follow, acquire=node.follow, replace=False)
            free_camera = FreeCameraControls(window, node)
            free_camera.mode_changed()
            speed = tk.Label(window)
            speed.pack(pady=8)
            def close():
                free_camera.clear()
                node.stop_requested = True
            window.protocol('WM_DELETE_WINDOW', close)
            signal.signal(signal.SIGINT, lambda *_: close())
            def poll():
                if done():
                    window.destroy()
                    return
                node.drive()
                free_camera.update()
                camera_button.config(text="释放视角" if node.follow else "跟随本车")
                autopilot_button.config(text='关闭自动驾驶' if node.autopilot else '开启自动驾驶')
                velocity = node.car.get_velocity()
                control = node.car.get_control()
                speed.config(text=f'{math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)*3.6:.1f} km/h'
                             + (f'  定速目标：{node.cruise_speed*3.6:.1f} km/h' if node.cruise_speed is not None else '')
                             + f'\n指令：{node.active or "收油／滑行"}  油门：{control.throttle:.2f}  刹车：{control.brake:.2f}')
                window.after(30, poll)
            window.after(30, poll)
            window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if free_camera:
            free_camera.close()
        camera_stop.set()
        if camera_worker:
            camera_worker.join()
        if executor:
            executor.shutdown()
        if worker:
            worker.join()
        if node:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
