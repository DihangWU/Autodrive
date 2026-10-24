"""仅将 CARLA 拥有的 Lincoln 投影为 SUMO 车辆，不控制 CARLA。"""
import logging
import math
import xml.etree.ElementTree as ET

TYPE_ID = 'autodrive_lincoln_proxy'
ROUTE_ID = 'autodrive_lincoln_route'
OWNER_PARAMETER = 'autodrive.lincoln.owner'
OWNER_PREFIX = 'lincoln:'


def find_passenger_edge(network_file):
    """从本次转换的实际路网选取可供轿车使用的普通道路。"""
    for edge in ET.parse(network_file).getroot().findall('edge'):
        if edge.get('function') == 'internal':
            continue
        for lane in edge.findall('lane'):
            allowed = lane.get('allow', '').split()
            forbidden = lane.get('disallow', '').split()
            if (not allowed or 'passenger' in allowed or 'all' in allowed) and not {'passenger', 'all'} & set(forbidden):
                return edge.get('id')
    raise ValueError('SUMO 路网没有可用于 Lincoln 的轿车车道。')


def carla_front_to_sumo(actor, transform, offset):
    """SUMO 使用车头中心，CARLA 使用 actor 原点；同时处理包围盒偏移。"""
    import carla
    box = actor.bounding_box
    front = transform.transform(carla.Location(x=box.location.x + box.extent.x,
                                               y=box.location.y, z=box.location.z))
    return front.x + offset[0], -front.y + offset[1], (transform.rotation.yaw + 90) % 360


class LincolnSumoProjection:
    def __init__(self, adapter):
        self.connection = adapter.connection
        self.network = adapter.network
        self.ready = False

    def prepare_route_and_type(self):
        if self.ready:
            return
        c = self.connection
        if ROUTE_ID not in c.route.getIDList():
            c.route.add(ROUTE_ID, [find_passenger_edge(self.network['network_file'])])
        if TYPE_ID not in c.vehicletype.getIDList():
            c.vehicletype.copy('DEFAULT_VEHTYPE', TYPE_ID)
            c.vehicletype.setVehicleClass(TYPE_ID, 'passenger')
            c.vehicletype.setShapeClass(TYPE_ID, 'passenger/sedan')
        self.ready = True

    def cleanup_previous_proxies(self):
        """仅清理带本功能标记的旧投影，不能碰普通 SUMO 车流。"""
        c = self.connection
        for name in c.vehicle.getIDList():
            if c.vehicle.getParameter(name, OWNER_PARAMETER).startswith(OWNER_PREFIX):
                c.vehicle.remove(name)

    def create_proxy(self, name, actor, owner_key):
        self.prepare_route_and_type()
        c = self.connection
        if name in c.vehicle.getIDList():
            raise ValueError(f'SUMO 已存在同名车辆 {name}，拒绝覆盖。')
        c.vehicle.add(name, ROUTE_ID, typeID=TYPE_ID, depart='now', departSpeed='0')
        try:
            # 先标记归属，即使后续设置失败也能在下一次接入时找回并清理。
            c.vehicle.setParameter(name, OWNER_PARAMETER, OWNER_PREFIX + owner_key)
            c.vehicle.setLength(name, actor.bounding_box.extent.x * 2)
            c.vehicle.setWidth(name, actor.bounding_box.extent.y * 2)
            c.vehicle.setHeight(name, actor.bounding_box.extent.z * 2)
            color = actor.attributes.get('color', '30,90,230').split(',')
            c.vehicle.setColor(name, [int(value) for value in color[:3]] + [255])
            c.vehicle.setSpeedMode(name, 0)
            c.vehicle.setLaneChangeMode(name, 0)
        except Exception:
            try:
                c.vehicle.remove(name)
            except Exception:
                logging.exception('Lincoln 半成品投影未能回滚：%s', name)
            raise

    def update_proxy(self, name, actor, snapshot):
        """在 SUMO 本帧步进前提交 CARLA 最新姿态；位置由 CARLA 决定。"""
        x, y, angle = carla_front_to_sumo(actor, snapshot.get_transform(), self.network['offset'])
        velocity = snapshot.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        self.connection.vehicle.setSpeed(name, speed)
        self.connection.vehicle.moveToXY(name, '', -1, x, y, angle=angle, keepRoute=2)

    def delete_proxy(self, name):
        if not self.connection.vehicle.getParameter(name, OWNER_PARAMETER).startswith(OWNER_PREFIX):
            raise ValueError(f'拒绝删除不属于 Lincoln 投影的 SUMO 车辆：{name}')
        self.connection.vehicle.remove(name)
