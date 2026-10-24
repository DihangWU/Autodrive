"""RViz 车辆模型、安装点和道路标记。"""
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from modules.lincoln.sensors.ros_messages import stamp, set_rotation
from modules.lincoln.runtime.paths import VEHICLE_MESH_FILE


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
        if self.display_lidar is None:
            self.markers.publish(MarkerArray(markers=[self.model] + self.mounts))

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
        for marker in [self.model] + self.mounts:
            marker.header.stamp = tf.header.stamp
        self.markers.publish(MarkerArray(markers=[self.model] + self.mounts))

    def make_markers(self):
        self.display_lidar = next((entry for entry in self.sensors if entry['spec']['kind'] == 'lidar'), None)
        display_frame = self.frame('lidar_display_base_link') if self.display_lidar is not None else self.frame()
        self.model = Marker()
        self.model.header.frame_id = display_frame
        self.model.ns, self.model.id = self.uid, 0
        self.model.frame_locked = True
        self.model.type, self.model.action = Marker.MESH_RESOURCE, Marker.ADD
        self.model.mesh_resource = VEHICLE_MESH_FILE.as_uri()
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

