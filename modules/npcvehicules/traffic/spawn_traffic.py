"""一批 SUMO 随机车流的创建、路线维护和删除；不推进任何仿真时钟。

由 CoBridge 在帧边界调用，复用它已经连接的 SUMO，业务代码留在 NPC 模块。
"""
import random

from modules.npcvehicules.traffic.vehicle_types import install_vehicle_type
from foundation.cobridge.npcvehicules.spawn_npc_sumo import (
    allowed_starting_edges, choose_vehicle_and_route, spawn_vehicle, update_vehicle_routes)

OWNER_PARAMETER = 'autodrive.npc.batch'


class NpcTrafficBatch:
    def __init__(self, connection, network, vehicle_types, batch_id, count, seed):
        self.connection = connection
        self.types = vehicle_types
        self.batch_id, self.count = batch_id, count
        self.rng = random.Random(seed)
        self.network = network
        self.created = {}
        self.existing_types = set(connection.vehicletype.getIDList())
        self.next_index = 0
        for spec in self.types.values():
            if not allowed_starting_edges(network, spec['vClass'], float(spec['length'])):
                raise ValueError(f"当前地图没有适合 {spec['id']} 出生的道路")

    def spawn_next_vehicles(self, limit):
        """分帧加入，避免一次创建大量车辆时长时间挡住共享时钟。"""
        for _ in range(min(limit, self.count - self.next_index)):
            spec, route = choose_vehicle_and_route(self.network, self.types, self.rng)
            install_vehicle_type(self.connection, spec, self.existing_types)
            name = f'npc_{self.batch_id}_{self.next_index}'
            spawn_vehicle(self.connection, name, route, spec['id'])
            # 立即记录，后续设置失败时仍可清理已加入的车辆。
            self.created[name] = dict(spec=spec)
            self.next_index += 1
            self.connection.vehicle.setParameter(name, OWNER_PARAMETER, self.batch_id)
            if spec['colors']:
                color = [int(v) for v in self.rng.choice(spec['colors']).split(',')]
                self.connection.vehicle.setColor(name, color + [255])
            else:
                color = [80, 100, 140]
            self.created[name]['color'] = color + [255]

    def maintain_routes(self, states, constants):
        update_vehicle_routes(self.connection, self.network, self.created, states, constants, self.rng)

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
