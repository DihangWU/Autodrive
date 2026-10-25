"""原生传感器与 ROS 2 发布：同步模式采集单帧完整 LiDAR，异步模式拼包。"""
import math
import queue
import time
import carla
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from modules.lincoln.sensors.ros_messages import (camera_mosaic, stamp, set_rotation,
                                                 local_pose, camera_info, sensor_points)

DISPLAY_FRAME_BUFFER = 4  # 有界缓存；不等待、不混用缺失的传感器帧。


def put_latest_sensor_packet(packets, data):
    """缓存满时丢弃最旧一包，原生传感器回调不等待 ROS 发布线程。"""
    try:
        packets.put_nowait(data)
    except queue.Full:
        try:
            packets.get_nowait()
        except queue.Empty:
            pass
        try:
            packets.put_nowait(data)
        except queue.Full:
            pass


def point_cloud_message(data, points, frame, kind):
    """原始数据和显示数据共用编码，始终保留采集时刻与传感器坐标系。"""
    msg = PointCloud2(height=1, width=len(points), is_bigendian=False, point_step=16,
                      row_step=len(points) * 16, is_dense=False, data=points.tobytes())
    msg.header.frame_id, msg.header.stamp = frame, stamp(data.timestamp)
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity' if kind == 'lidar' else 'velocity'))]
    return msg

def collect_complete_lidar_scan(entry, data):
    """同步模式直接取单帧完整扫描；异步模式才拼包，按配置频率输出。"""
    if entry['scan_started'] is None or data.timestamp < entry['scan_started']:
        entry['scan_started'] = data.timestamp
        entry['scan_chunks'].clear()
        entry['last_published'] = None
        return None  # 第一包的起始时刻未知，从它结束后开始收集完整一轮。
    entry['scan_chunks'].append(data)
    if data.timestamp - entry['scan_started'] < entry['scan_period'] - 1e-6:
        return None
    chunks = list(entry['scan_chunks'])
    entry['scan_chunks'].clear()
    entry['scan_started'] = data.timestamp
    if (entry['last_published'] is not None
            and data.timestamp - entry['last_published'] < entry['publish_period'] - 1e-6):
        return None  # 例如每帧20Hz完整采集，ROS仍按配置10Hz输出。
    entry['single_frame_scan'] = len(chunks) == 1
    if len(chunks) == 1:
        points = sensor_points(data, 'lidar')  # 同一个物理帧，没有半帧交界或运动拼接。
    else:
        reference_inverse = np.asarray(data.transform.get_inverse_matrix(), dtype=np.float32)
        parts = []
        for chunk in chunks:
            points = np.frombuffer(chunk.raw_data, dtype=np.float32).reshape(-1, 4).copy()
            relative = reference_inverse @ np.asarray(chunk.transform.get_matrix(), dtype=np.float32)
            points[:, :3] = np.einsum('ij,nj->ni', relative[:3, :3], points[:, :3]) + relative[:3, 3]
            points[:, 1] *= -1
            parts.append(points)
        points = np.concatenate(parts)
    entry['last_published'] = data.timestamp
    return data, points


class SensorStreamsMixin:
    def frame(self, name='base_link'):
        return self.uid + '/' + name

    def spawn_sensor(self, spec):
        kind = spec['kind']
        bp = self.world.get_blueprint_library().find({'camera': 'sensor.camera.rgb',
            'lidar': 'sensor.lidar.ray_cast', 'radar': 'sensor.other.radar'}[kind])
        rate = min(spec['rate'], 1 / self.dt)
        period = max(1, round(1 / rate / self.dt))
        # LiDAR 转速匹配初始化步长，使同步模式每个物理帧扫描完整视野。
        every_frame = kind == 'lidar' or (kind == 'radar' and self.synchronize_radar_display)
        bp.set_attribute('sensor_tick', '0' if every_frame else str(max(0., period * self.dt - 1e-6)))
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
        if kind == 'lidar':
            entry.update(scan_started=None, scan_chunks=[], scan_period=self.dt,
                         publish_period=1 / rate, last_published=None,
                         sensor_mount_inverse=np.asarray(transform.get_inverse_matrix()))
        topic = f'/{self.uid}/{spec["name"]}'
        entry['pub'] = self.create_publisher(Image if kind == 'camera' else PointCloud2,
            topic + ('/image' if kind == 'camera' else '/points'), qos_profile_sensor_data)
        if self.synchronize_radar_display and kind != 'camera':
            entry['display_queue'] = queue.Queue(maxsize=DISPLAY_FRAME_BUFFER)
            entry['display_pub'] = self.create_publisher(PointCloud2, topic + '/display_points', qos_profile_sensor_data)
        if kind == 'camera':
            entry['info'] = self.create_publisher(CameraInfo, topic + '/camera_info', qos_profile_sensor_data)
        self.sensors.append(entry)
        def receive(data):
            entry['received'] += 1
            if kind == 'lidar':
                data = collect_complete_lidar_scan(entry, data)
                if data is None:
                    return
            if 'display_queue' in entry:
                if kind != 'lidar' or entry['single_frame_scan']:
                    put_latest_sensor_packet(entry['display_queue'], data)
                if kind == 'radar' and data.frame % period:
                    return  # 对外原始话题仍按配置的10/20Hz输出。
            put_latest_sensor_packet(entry['queue'], data)
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
        for entry in self.sensors:
            try:
                data = entry['queue'].get_nowait()
            except queue.Empty:
                continue
            spec = entry['spec']
            entry['published'] += 1
            frame = self.frame(spec['name'])
            if spec['kind'] == 'lidar':
                data, points = data
            if spec['kind'] == 'camera':
                rgba = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(data.height, data.width, 4)
                rgb = rgba[:, :, [2, 1, 0]].copy()
                msg = Image(height=data.height, width=data.width, encoding='rgb8', step=data.width * 3, data=rgb.tobytes())
                msg.header.frame_id, msg.header.stamp = frame + '/optical', stamp(data.timestamp)
                entry['pub'].publish(msg)
                entry['info'].publish(camera_info(spec, msg.header))
                self.latest_images[spec['name']] = rgb
            else:
                if spec['kind'] != 'lidar':
                    points = sensor_points(data, spec['kind'])
                msg = point_cloud_message(data, points, frame, spec['kind'])
                entry['pub'].publish(msg)
                if spec['kind'] == 'lidar' and not self.synchronize_radar_display:
                    self.publish_lidar_aligned_vehicle(entry, data)
        if self.synchronize_radar_display:
            self.publish_synchronized_radar_display()
        if self.latest_images and time.monotonic() - self.last_mosaic > .1:
            specs = {e['spec']['name']: e['spec'] for e in self.sensors if e['spec']['kind'] == 'camera'}
            canvas = camera_mosaic(self.latest_images, specs, self.args.camera_rows)
            msg = Image(height=canvas.shape[0], width=canvas.shape[1], encoding='rgb8',
                        step=canvas.shape[1] * 3, data=canvas.tobytes())
            msg.header.stamp = stamp(snapshot.timestamp.elapsed_seconds)
            self.mosaic.publish(msg)
            self.last_mosaic = time.monotonic()

    def publish_synchronized_radar_display(self):
        """只发布全部雷达到齐的最新物理帧，随后更新同一时刻的车模。"""
        entries = [entry for entry in self.sensors if 'display_queue' in entry]
        frames = self.sensor_display_frames
        for entry in entries:
            for _ in range(DISPLAY_FRAME_BUFFER):
                try:
                    packet = entry['display_queue'].get_nowait()
                except queue.Empty:
                    break
                data = packet[0] if entry['spec']['kind'] == 'lidar' else packet
                if data.frame <= self.sensor_display_frame:
                    continue
                frames.setdefault(data.frame, {})[entry['spec']['name']] = packet
        complete = [frame for frame, packets in frames.items() if len(packets) == len(entries)]
        if complete:
            frame = max(complete)
            packets = frames[frame]
            reference = packets[self.display_lidar['spec']['name']][0]
            reference_stamp = stamp(reference.timestamp)
            messages = []
            for entry in entries:
                spec = entry['spec']
                packet = packets[spec['name']]
                if spec['kind'] == 'lidar':
                    data, points = packet
                else:
                    data, points = packet, sensor_points(packet, spec['kind'])
                if stamp(data.timestamp) != reference_stamp:
                    break  # 不改写时间戳，也不把不同采样时刻冒充为同帧。
                messages.append((entry['display_pub'], point_cloud_message(data, points, self.frame(spec['name']), spec['kind'])))
            if len(messages) == len(entries):
                for publisher, message in messages:
                    publisher.publish(message)
                self.publish_lidar_aligned_vehicle(self.display_lidar, reference)
                self.sensor_display_frame = frame
            for old in [key for key in frames if key <= frame]:
                del frames[old]
        for old in sorted(frames)[:-DISPLAY_FRAME_BUFFER]:
            del frames[old]

