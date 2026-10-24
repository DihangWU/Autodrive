"""桥进程内的 Lincoln 登记与同步；CARLA 车辆始终属于模块进程。"""
import logging
import re

from foundation.cobridge.lincoln.vehicle_projection import LincolnSumoProjection
from foundation.cobridge.lincoln.traffic_manager_clock import LincolnTrafficManagerClock

SUMO_ID_PATTERN = r'lincoln[0-9]+'
BLUEPRINT_ID = 'vehicle.lincoln.mkz_2020'


class LincolnVehicleRegistry:
    def __init__(self):
        self.records = {}
        self.traffic_managers = LincolnTrafficManagerClock()
        self.projection = None
        self.sumo_adapter = None
        self.retired = set()

    def register_vehicle(self, command, owner):
        self.validate_tm_port(command.get('tm_port'))
        actor_id, world_id, role = command['actor_id'], command['world_id'], command['role_name']
        if not isinstance(actor_id, int) or actor_id <= 0 or not isinstance(world_id, int):
            raise ValueError('CARLA actor/world ID 无效。')
        if not isinstance(role, str) or not re.fullmatch(r'lincoln_[0-9a-f]{10}', role):
            raise ValueError('Lincoln role_name 无效。')
        key = f'{world_id}:{actor_id}'
        record = self.records.get(key)
        if record is None:
            preferred = command.get('sumo_id')
            used = {item['sumo_id'] for item in self.records.values()} | self.retired
            if preferred and re.fullmatch(SUMO_ID_PATTERN, preferred) and preferred not in used:
                name = preferred
            else:
                number = 0
                while f'lincoln{number}' in used:
                    number += 1
                name = f'lincoln{number}'
            record = dict(actor_id=actor_id, world_id=world_id, role_name=role,
                          sumo_id=name, state='waiting', detail='等待匹配的 CARLA 和 SUMO',
                          actor=None, projected=False, owner=owner)
            self.records[key] = record
        elif record['role_name'] != role:
            raise ValueError('车辆登记身份不匹配。')
        record['owner'] = owner
        record['tm_port'] = command.get('tm_port')
        return self.vehicle_status(record)

    def handle_request(self, command, owner):
        action = command['action']
        if action == 'lincoln.register':
            return self.register_vehicle(command, owner)
        key = f"{command['world_id']}:{command['actor_id']}"
        record = self.records.get(key)
        if record is None or record['owner'] is not owner:
            raise ValueError('该连接没有登记这辆 Lincoln。')
        if action == 'lincoln.status':
            self.validate_tm_port(command.get('tm_port'))
            record['tm_port'] = command.get('tm_port')
            return self.vehicle_status(record)
        if action == 'lincoln.unregister':
            self.retire_vehicle(key)
            return dict(removed=True)
        raise ValueError(f'未知 Lincoln 请求：{action}')

    @staticmethod
    def vehicle_status(record):
        return {key: record[key] for key in ('actor_id', 'world_id', 'sumo_id', 'state', 'detail')}

    @staticmethod
    def validate_tm_port(port):
        if port is not None and (not isinstance(port, int) or not 1 <= port <= 65535):
            raise ValueError('Traffic Manager 端口无效。')

    def prepare_traffic_managers(self, carla):
        self.traffic_managers.attach_registered_managers(carla, self.records.values())

    def snapshot(self):
        return [self.vehicle_status(record) for record in self.records.values()]

    def retire_vehicle(self, key):
        record = self.records.pop(key)
        if record['projected']:
            self.retired.add(record['sumo_id'])

    def disconnected(self, owner):
        for key, record in list(self.records.items()):
            if record['owner'] is owner:
                self.retire_vehicle(key)

    def attach_sumo(self, sumo):
        if self.sumo_adapter is sumo:
            return
        self.projection = LincolnSumoProjection(sumo)
        self.projection.cleanup_previous_proxies()
        self.sumo_adapter = sumo
        self.retired.clear()
        for record in self.records.values():
            record['projected'] = False

    def cleanup_retired_proxies(self):
        if self.projection is None or not self.retired:
            return
        present = set(self.projection.connection.vehicle.getIDList())
        for name in list(self.retired):
            if name in present:
                self.projection.delete_proxy(name)
            self.retired.discard(name)

    def synchronize_one_vehicle(self, key, record, carla, snapshot):
        if record['world_id'] != carla.world.id:
            raise ValueError('Lincoln 所在的 CARLA 世界与桥当前世界不同。')
        actor = record['actor']
        if actor is None:
            actor = carla.world.get_actor(record['actor_id'])
            if actor is None:
                raise ValueError('CARLA 车辆已经删除。')
            if actor.type_id != BLUEPRINT_ID or actor.attributes.get('role_name') != record['role_name']:
                raise ValueError('CARLA 车辆类型或身份不匹配。')
            record['actor'] = actor
        actor_snapshot = snapshot.find(record['actor_id'])
        if actor_snapshot is None:
            raise ValueError('CARLA 当前帧中没有这辆车辆。')
        if not record['projected']:
            self.projection.create_proxy(record['sumo_id'], actor, key)
            record['projected'] = True
        self.projection.update_proxy(record['sumo_id'], actor, actor_snapshot)
        record['state'], record['detail'] = 'synced', 'CARLA → SUMO'

    def synchronize_before_sumo_step(self, carla, sumo):
        """由唯一桥主循环调用；本方法从不额外推进任何仿真器。"""
        if carla is not None:
            self.traffic_managers.advance_controllers()
        if sumo is None:
            self.sumo_adapter = None
            self.projection = None
            for record in self.records.values():
                record['state'], record['detail'] = 'waiting', '等待 SUMO 接入'
            return
        self.attach_sumo(sumo)
        self.cleanup_retired_proxies()
        if carla is None:
            for record in self.records.values():
                record['state'], record['detail'] = 'waiting', '等待 CARLA 重新接入'
                if record['projected']:
                    self.projection.connection.vehicle.setSpeed(record['sumo_id'], 0)
            return
        if not self.records:
            return
        snapshot = carla.world.get_snapshot()
        present = set(self.projection.connection.vehicle.getIDList()) if self.records else set()
        for key, record in list(self.records.items()):
            try:
                if record['projected'] and record['sumo_id'] not in present:
                    record['projected'] = False
                self.synchronize_one_vehicle(key, record, carla, snapshot)
            except ValueError as error:
                record['state'], record['detail'] = 'error', str(error)
                self.retire_vehicle(key)
            except Exception as error:
                message = str(error)
                if record['detail'] != message:
                    logging.warning('Lincoln %s 同步等待：%s', record['sumo_id'], message)
                record['state'], record['detail'] = 'waiting', message
        self.cleanup_retired_proxies()

    def close(self, sumo):
        """正常退出桥时仅删除 SUMO 投影，绝不删除模块拥有的 CARLA 车辆。"""
        self.traffic_managers.release()
        for key in list(self.records):
            self.retire_vehicle(key)
        if sumo is not None:
            self.attach_sumo(sumo)
            self.cleanup_retired_proxies()
