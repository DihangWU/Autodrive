"""原生传感器、静态 TF 与 ROS 2 发布，保持旧版传感器参数。"""
import math
import queue
import time
import carla
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
from modules.lincoln.sensors.ros_messages import (camera_mosaic, stamp, set_rotation,
                                                 local_pose, camera_info, sensor_points)


class SensorStreamsMixin:
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

