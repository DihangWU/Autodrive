"""一批 SUMO 随机车流的创建、路线维护和删除；不推进任何仿真时钟。

由 CoBridge 在帧边界调用，复用它已经连接的 SUMO，业务代码留在 NPC 模块。
"""
import random

from modules.npcvehicules.traffic.routes import RandomRoutes
from modules.npcvehicules.traffic.vehicle_types import install_vehicle_type

OWNER_PARAMETER = 'autodrive.npc.batch'
SPAWN_EXTRA_SPACE = 5.0


class NpcTrafficBatch:
    def __init__(self, connection, network, vehicle_types, batch_id, count, seed):
        self.connection = connection
        self.types = vehicle_types
        self.batch_id, self.count = batch_id, count
        self.rng = random.Random(seed)
        self.routes = RandomRoutes(network, self.rng)
        self.created = {}
        self.existing_types = set(connection.vehicletype.getIDList())
        self.next_index = 0
        for spec in self.types.values():
            self.routes.starting_edges(spec['vClass'], float(spec['length']) + SPAWN_EXTRA_SPACE)

    def spawn_next_vehicles(self, limit):
        """分帧加入，避免一次创建大量车辆时长时间挡住共享时钟。"""
        for _ in range(min(limit, self.count - self.next_index)):
            spec = self.rng.choice(list(self.types.values()))
            install_vehicle_type(self.connection, spec, self.existing_types)
            name = f'npc_{self.batch_id}_{self.next_index}'
            route = self.routes.create(spec['vClass'], float(spec['length']) + SPAWN_EXTRA_SPACE)
            self.connection.route.add(name, route)
            self.connection.vehicle.add(name, name, typeID=spec['id'],
                depart='now', departLane='random', departPos='random_free', departSpeed='0')
            # 立即记录，后续设置失败时仍可清理已加入的车辆。
            self.created[name] = dict(spec=spec, route=route)
            self.next_index += 1
            self.connection.vehicle.setParameter(name, OWNER_PARAMETER, self.batch_id)
            if spec['colors']:
                color = [int(v) for v in self.rng.choice(spec['colors']).split(',')]
                self.connection.vehicle.setColor(name, color + [255])
            else:
                color = [80, 100, 140]
            self.created[name]['color'] = color + [255]

    def maintain_routes(self, states, constants):
        for name, record in self.created.items():
            state = states.get(name)
            if state:
                self.routes.continue_vehicle(self.connection, name, record,
                    state[str(constants.VAR_ROUTE_INDEX)], state[str(constants.VAR_ROAD_ID)])

    def remove_vehicles(self, subscribed=None):
        """同时清理已上路和仍等待插入的本批车辆；保留其他批次和 Lincoln。"""
        c = self.connection
        active = set(c.vehicle.getIDList())
        present = active | set(c.simulation.getPendingVehicles())
        for name in self.created.keys() & present:
            if c.vehicle.getParameter(name, OWNER_PARAMETER) not in ('', self.batch_id):
                raise RuntimeError(f'NPC 归属不匹配，拒绝删除：{name}')
            if subscribed is not None and name in active and name in subscribed:
                c.vehicle.unsubscribe(name)
                subscribed.discard(name)
            c.vehicle.remove(name)
