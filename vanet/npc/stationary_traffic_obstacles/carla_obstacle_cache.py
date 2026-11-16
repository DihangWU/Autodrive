"""缓存固定堵车车的包围盒；生命周期改变时重建，运动车辆继续逐帧采样。"""
import logging
import math
import carla

from .spatial_index import ObstacleSet, StaticObstacleIndex

LOG = logging.getLogger('vanet.lincoln.obstacles')
STATIONARY_ROLE_PREFIX = 'vanet_obstacle_highway_'


def obstacle_geometry(actor, state):
    transform, box = state.get_transform(), actor.bounding_box
    center = transform.transform(carla.Location(x=box.location.x, y=box.location.y, z=box.location.z))
    velocity = state.get_velocity()
    return dict(id=actor.id, kind=actor.type_id, center=(center.x, center.y, center.z),
                velocity=(velocity.x, velocity.y, velocity.z),
                yaw=math.radians(transform.rotation.yaw + box.rotation.yaw),
                length=box.extent.x, width=box.extent.y, height=box.extent.z)


class CarlaObstacleCache:
    def __init__(self, node):
        self.node = node
        self.identity = None
        self.dynamic_actors = ()
        self.static_index = StaticObstacleIndex()
        self.previous = {}

    def collect(self, snapshot):
        world = self.node.world
        try:
            identity = (getattr(world, 'id', None), frozenset(state.id for state in snapshot))
        except TypeError:
            identity = None  # Simple snapshots in offline tests have no actor iterator.
        if identity is None or identity != self.identity:
            static, dynamic = [], []
            for actor in world.get_actors():
                if actor.id == self.node.car.id or not actor.type_id.startswith(('vehicle.', 'walker.', 'static.prop.')):
                    continue
                state = snapshot.find(actor.id)
                if state is None:
                    continue
                role = getattr(actor, 'attributes', {}).get('role_name', '')
                if actor.type_id.startswith('vehicle.') and role.startswith(STATIONARY_ROLE_PREFIX):
                    obstacle = obstacle_geometry(actor, state)
                    obstacle['velocity'] = (0., 0., 0.)
                    static.append(obstacle)
                else:
                    dynamic.append(actor)
            self.dynamic_actors = tuple(dynamic)
            self.static_index = StaticObstacleIndex(static)
            if self.identity is not None and identity is not None and self.identity[0] != identity[0]:
                self.previous = {}
            self.identity = identity
            LOG.info('OBSTACLE_CACHE_REBUILT static=%s dynamic=%s', len(static), len(dynamic))
        now = snapshot.timestamp.elapsed_seconds
        dynamic, current = [], {}
        for actor in self.dynamic_actors:
            state = snapshot.find(actor.id)
            if state is None:
                continue
            obstacle = obstacle_geometry(actor, state)
            point, motion = obstacle['center'], obstacle['velocity']
            # Moving SUMO proxies also have physics disabled: retain position-delta
            # velocity estimation. They must never be cached as stationary cars.
            if actor.id in self.previous:
                then, previous = self.previous[actor.id]
                if now > then and sum(v * v for v in motion) < .01:
                    obstacle['velocity'] = tuple((point[i] - previous[i]) / (now - then) for i in range(3))
            current[actor.id] = (now, point)
            dynamic.append(obstacle)
        self.previous = current
        return ObstacleSet(dynamic, self.static_index)
