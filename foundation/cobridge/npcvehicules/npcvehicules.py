"""NPC 桥端会话：登记归属、调用随机车流模块、同步代理并清理。

只在现有 CoBridge 帧边界运行，不启动 SUMO/CARLA，不额外推进时钟。
"""
import json
import logging
from pathlib import Path
import uuid

from foundation.cobridge.npcvehicules.official_tools import load_official_tools
from foundation.cobridge.npcvehicules.vehicle_sync import NpcCarlaProjection
from modules.npcvehicules.traffic.spawn_traffic import NpcTrafficBatch
from modules.npcvehicules.traffic.vehicle_types import read_vehicle_types

CONFIG_FILE = Path(__file__).resolve().parent / 'config.json'


class NpcVehicleRegistry:
    def __init__(self):
        self.batches = {}

    def start_batch(self, command, owner, carla, sumo):
        """先验证两端和请求，再登记一批；车辆由随后各帧逐步创建。"""
        if carla is None or sumo is None:
            raise RuntimeError('请先让 CARLA 和 SUMO 接入 CoBridge，再添加 NPC。')
        if carla.fingerprint != sumo.network['xodr_sha256']:
            raise RuntimeError('CARLA 与 SUMO 地图不匹配。')
        settings = json.loads(CONFIG_FILE.read_text())
        count, seed, pattern = command['count'], command.get('seed', 0), command.get('filter', 'vehicle.*')
        capacity = settings['max_vehicles'] - sum(r['traffic'].count for r in self.batches.values())
        if (type(count) is not int or not 1 <= count <= capacity or type(seed) is not int
                or not isinstance(pattern, str) or not pattern or len(pattern) > 128):
            raise ValueError(f'NPC 数量或参数无效；当前还可申请 {capacity} 辆。')
        if type(settings['spawn_per_step']) is not int or settings['spawn_per_step'] < 1:
            raise ValueError('spawn_per_step 必须为正整数。')
        tools = load_official_tools()
        blueprints = carla.world.get_blueprint_library()
        types = read_vehicle_types(blueprints, pattern)
        network = tools.sumolib.net.readNet(str(sumo.network['network_file']))
        name = uuid.uuid4().hex[:12]
        traffic = NpcTrafficBatch(sumo.connection, network, types, name, count, seed)
        projection = NpcCarlaProjection(carla, sumo, tools, settings['sync_vehicle_lights'])
        self.batches[name] = dict(owner=owner, traffic=traffic, projection=projection,
            carla=carla, sumo=sumo, spawn_per_step=settings['spawn_per_step'],
            state='starting', detail='', closing=False, active=0)
        return dict(batch_id=name, count=count)

    def handle_request(self, command, owner, carla, sumo):
        if command['action'] == 'npc.start':
            return self.start_batch(command, owner, carla, sumo)
        name = command['batch_id']
        record = self.batches.get(name)
        if record is None or record['owner'] is not owner:
            raise ValueError('当前连接没有登记这批 NPC。')
        if command['action'] == 'npc.status':
            return self.batch_status(name, record)
        if command['action'] == 'npc.stop':
            record['closing'] = True
            return dict(stopping=True)
        raise ValueError(f"未知 NPC 请求：{command['action']}")

    @staticmethod
    def batch_status(name, record):
        traffic, projection = record['traffic'], record['projection']
        detail = record['detail'] or next(iter(projection.errors.values()), '')
        return dict(batch_id=name, state=record['state'], requested=traffic.count,
            added=traffic.next_index, active=record['active'], carla=len(projection.actors),
            pending=traffic.next_index - len(projection.subscribed),
            finished=max(0, len(projection.subscribed) - record['active']), detail=detail)

    def snapshot(self):
        return [self.batch_status(name, r) for name, r in self.batches.items()]

    def disconnected(self, owner):
        """客户端正常退出或被杀死，都由桥在下一个帧边界收尾。"""
        for record in self.batches.values():
            if record['owner'] is owner:
                record['closing'] = True

    def remove_batch(self, name, carla, sumo):
        """先清理所有能够连接的本批实体，任何失败都保留记录供重试。"""
        record = self.batches[name]
        errors = []
        projection, traffic = record['projection'], record['traffic']
        if projection.actors:
            try:
                if carla is None:
                    raise RuntimeError('等待 CARLA 重新接入，以清理 NPC 实体。')
                if carla.world.id == record['carla'].world.id:
                    projection.carla = carla
                    projection.close()
                else:
                    # load_world 已销毁旧世界，不能把旧 ID 用到新世界。
                    projection.actors.clear()
            except Exception as error:
                errors.append(str(error))
        if traffic.created:
            try:
                if sumo is None:
                    raise RuntimeError('等待 SUMO 重新接入，以清理本批 NPC。')
                if sumo.network['xodr_sha256'] != record['sumo'].network['xodr_sha256']:
                    raise RuntimeError('等待原地图的 SUMO 接入，以清理本批 NPC。')
                traffic.connection = sumo.connection
                traffic.remove_vehicles(projection.subscribed)
                traffic.created.clear()
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('; '.join(errors))
        del self.batches[name]

    def synchronize(self, carla, sumo):
        """上一轮 SUMO 完成后取得的状态，在下一轮 CARLA tick 前提交。"""
        for name, record in list(self.batches.items()):
            if carla is not record['carla'] or sumo is not record['sumo']:
                record['state'], record['detail'] = 'error', '仿真器已断开或重新接入，请重新添加本批 NPC。'
                record['closing'] = True
            try:
                if record['closing']:
                    self.remove_batch(name, carla, sumo)
                    continue
                if record['state'] == 'error':
                    continue
                record['traffic'].spawn_next_vehicles(record['spawn_per_step'])
            except Exception as error:
                self.report_error(record, error)
        if not self.batches or carla is None or sumo is None:
            return
        try:
            present = set(sumo.connection.vehicle.getIDList())
            for record in self.batches.values():
                if record['state'] != 'error' and not record['closing']:
                    record['projection'].subscribe_new_vehicles(record['traffic'], present)
            # 全部 NPC 的动态变量只读一次；不逐车请求位置和角度。
            states = sumo.connection.vehicle.getAllSubscriptionResults()
            for record in self.batches.values():
                if record['state'] == 'error' or record['closing']:
                    continue
                traffic, projection = record['traffic'], record['projection']
                record['active'] = len(traffic.created.keys() & present)
                projection.update_vehicles(traffic, states, present)
                traffic.maintain_routes(states, projection.tools.constants)
                record['state'] = 'running' if traffic.next_index == traffic.count else 'starting'
        except Exception as error:
            for record in self.batches.values():
                self.report_error(record, error)

    @staticmethod
    def report_error(record, error):
        message = str(error)
        if record['detail'] != message:
            logging.exception('NPC 同步异常：%s', message)
        record['state'], record['detail'] = 'error', message

    def close(self, carla, sumo):
        errors = []
        for name in list(self.batches):
            try:
                self.remove_batch(name, carla, sumo)
            except Exception as error:
                errors.append(f'{name}: {error}')
        if errors:
            raise RuntimeError('; '.join(errors))
