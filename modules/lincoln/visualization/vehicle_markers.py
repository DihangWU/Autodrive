"""RViz 车辆模型、安装点和道路标记。"""
from visualization_msgs.msg import Marker, MarkerArray
from modules.lincoln.runtime.paths import VEHICLE_MESH_FILE


class VehicleMarkersMixin:
    def make_markers(self):
        self.model = Marker()
        self.model.header.frame_id = self.frame()
        self.model.ns, self.model.id = self.uid, 0
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
            m.header.frame_id = self.frame(entry['spec']['name'])
            m.ns, m.id, m.type, m.action = self.uid, i, Marker.SPHERE, Marker.ADD
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

