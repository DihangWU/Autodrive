"""RViz 车辆模型、安装点和道路标记。"""
import math
import logging
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from modules.lincoln.sensors.ros_messages import stamp, set_rotation
from modules.lincoln.visualization.wheel_meshes import prepare_wheel_meshes

WHEEL_BONES = ('Wheel_Front_Left', 'Wheel_Front_Right', 'Wheel_Rear_Left', 'Wheel_Rear_Right')
WHEEL_POSE_FRAME_BUFFER = 8


class VehicleMarkersMixin:
    def publish_vehicle_transform(self, snapshot):
        """与 CARLA 新帧同步发布，不等待相机和点云转换。"""
        actor = snapshot.find(self.car.id)
        if actor is None or snapshot.frame == self.last_frame:
            return
        self.last_frame = snapshot.frame
        transform = actor.get_transform()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', self.frame()
        tf.header.stamp = stamp(snapshot.timestamp.elapsed_seconds)
        loc, rot = transform.location, transform.rotation
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = loc.x, -loc.y, loc.z
        set_rotation(tf.transform.rotation, (math.radians(rot.roll), -math.radians(rot.pitch), -math.radians(rot.yaw)))
        self.tf.sendTransform(tf)
        self.capture_wheel_poses(snapshot)
        if self.display_lidar is None:
            self.publish_vehicle_markers(snapshot.frame, tf.header.stamp)

    def capture_wheel_poses(self, snapshot):
        """一次读取四轮骨骼，只有世界仍处于该帧时才登记为同帧数据。"""
        navigation_view = getattr(self, 'navigation_view', None)
        if self.markers.get_subscription_count() == 0 and not (
                navigation_view is not None and navigation_view.has_vehicle_subscribers()):
            return
        try:
            poses = self.car.get_bone_relative_transforms()
            if self.world.get_snapshot().frame != snapshot.frame:
                return
            self.wheel_pose_frames[snapshot.frame] = tuple(poses[i] for i in self.wheel_bone_indices)
            for old in sorted(self.wheel_pose_frames)[:-WHEEL_POSE_FRAME_BUFFER]:
                del self.wheel_pose_frames[old]
            self.wheel_read_failed = False
        except RuntimeError as error:
            if not self.wheel_read_failed:
                logging.warning('RViz 车轮骨骼读取失败，暂时保留上次姿态：%s', error)
                self.wheel_read_failed = True

    def publish_vehicle_markers(self, frame, timestamp):
        """车身、安装点和四轮共用显示时刻；缺少同帧骨骼时保留旧轮姿态。"""
        poses = self.wheel_pose_frames.get(frame)
        if poses is not None:
            for marker, transform in zip(self.wheel_markers, poses):
                loc, rot = transform.location, transform.rotation
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = loc.x, -loc.y, loc.z
                set_rotation(marker.pose.orientation, (math.radians(rot.roll),
                             -math.radians(rot.pitch), -math.radians(rot.yaw)))
        markers = [self.model] + self.mounts + self.wheel_markers
        for marker in markers:
            marker.header.stamp = timestamp
        self.markers.publish(MarkerArray(markers=markers))
        navigation_view = getattr(self, 'navigation_view', None)
        if navigation_view is not None:
            navigation_view.publish_vehicle()

    def publish_lidar_aligned_vehicle(self, entry, data):
        """显示位姿取自本次点云的采集位姿；真实车辆 TF 仍独立更新。"""
        if entry is not self.display_lidar:
            return
        # CARLA 提供采集时刻的传感器世界位姿，乘安装位姿的逆得到车辆原点。
        vehicle_pose = np.asarray(data.transform.get_matrix()) @ entry['sensor_mount_inverse']
        # CARLA 的 Y 轴与 ROS 相反，同时转换平移和旋转。
        axes = np.array([1., -1., 1.])
        rotation = vehicle_pose[:3, :3] * axes[:, None] * axes[None, :]
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', self.model.header.frame_id
        tf.header.stamp = stamp(data.timestamp)
        position = vehicle_pose[:3, 3] * axes
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, position)
        set_rotation(tf.transform.rotation, (
            math.atan2(rotation[2, 1], rotation[2, 2]),
            math.asin(float(np.clip(-rotation[2, 0], -1., 1.))),
            math.atan2(rotation[1, 0], rotation[0, 0])))
        self.tf.sendTransform(tf)
        self.publish_vehicle_markers(data.frame, tf.header.stamp)

    def make_markers(self):
        self.display_lidar = next((entry for entry in self.sensors if entry['spec']['kind'] == 'lidar'), None)
        display_frame = self.frame('lidar_display_base_link') if self.display_lidar is not None else self.frame()
        self.model = Marker()
        self.model.header.frame_id = display_frame
        self.model.ns, self.model.id = self.uid, 0
        self.model.frame_locked = True
        self.model.type, self.model.action = Marker.MESH_RESOURCE, Marker.ADD
        mesh_folder, meshes = prepare_wheel_meshes(self.geometry)
        self.model.mesh_resource = (mesh_folder / meshes['body']).as_uri()
        self.model.mesh_use_embedded_materials = True
        cx, cy, cz = self.geometry['center']
        self.model.pose.position.x, self.model.pose.position.y = cx, cy
        self.model.pose.position.z = cz - self.geometry['height'] / 2
        self.model.pose.orientation.w = 1.
        self.model.scale.x = self.model.scale.y = self.model.scale.z = 1.
        self.model.color.a = 1.
        self.wheel_markers = []
        self.wheel_pose_frames = {}
        self.wheel_read_failed = False
        bone_names = self.car.get_bone_names()
        self.wheel_bone_indices = [bone_names.index(name) for name in WHEEL_BONES]
        for index, wheel in enumerate(meshes['wheels']):
            marker = Marker()
            marker.header.frame_id = display_frame
            marker.ns, marker.id = self.uid + '_wheels', index
            marker.frame_locked = True
            marker.type, marker.action = Marker.MESH_RESOURCE, Marker.ADD
            marker.mesh_resource = (mesh_folder / wheel['file']).as_uri()
            marker.mesh_use_embedded_materials = True
            offset = np.array([cx, cy, cz - self.geometry['height'] / 2]) + wheel['pivot']
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, offset)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = 1.
            marker.color.a = 1.
            self.wheel_markers.append(marker)
        self.mounts = []
        for i, entry in enumerate(self.sensors, 1):
            m = Marker()
            m.header.frame_id = display_frame
            m.pose.position.x, m.pose.position.y, m.pose.position.z = entry['pose'][:3]
            m.ns, m.id, m.type, m.action = self.uid, i, Marker.SPHERE, Marker.ADD
            m.frame_locked = True
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = .08
            m.color.r, m.color.g, m.color.b, m.color.a = (1., .6, .1, 1.)
            self.mounts.append(m)

    def make_roads(self):
        if self.args.no_rviz:
            self.roads = MarkerArray()
            return
        from modules.lincoln.visualization.road_map import road_markers
        self.roads = road_markers(self.world.get_map())

