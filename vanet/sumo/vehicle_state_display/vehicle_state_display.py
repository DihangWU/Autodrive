"""SUMO 车辆状态配色：保存原状态、按变化写入、关闭时恢复。"""
import logging

LOG = logging.getLogger(__name__)
NPC_COLORS = {
    'received': (75,160,255,255),
    'yielding': (255,165,0,255),
    'cleared': (65,200,100,255),
    'unable': (225,60,120,255),
    'recovering': (160,110,220,255),
}
EMERGENCY_COLOR = (255,65,35,255)
PARAMETER_NAMES = ('vanet.yielding','vanet.yield_reason','vanet.task_id',
                   'vanet.emergency','vanet.state')


class VehicleStateDisplay:
    def __init__(self, connection):
        self.connection = connection
        self.saved = {}

    def remember(self, vehicle_id):
        if vehicle_id not in self.saved:
            vehicle = self.connection.vehicle
            color = tuple(vehicle.getColor(vehicle_id))
            parameters = {key:vehicle.getParameter(vehicle_id,key) for key in PARAMETER_NAMES}
            self.saved[vehicle_id] = dict(color=color,parameters=parameters,
                                           applied_color=color,applied_parameters=dict(parameters))
        return self.saved[vehicle_id]

    def apply(self, vehicle_id, color, parameters):
        record = self.remember(vehicle_id)
        selected = tuple(color) if color is not None else record['color']
        if selected != record['applied_color']:
            self.connection.vehicle.setColor(vehicle_id,selected)
            record['applied_color'] = selected
        for key,value in parameters.items():
            value = str(value)
            if record['applied_parameters'].get(key) != value:
                self.connection.vehicle.setParameter(vehicle_id,key,value)
                record['applied_parameters'][key] = value

    def npc(self, vehicle_id, status, reason='', task_id=''):
        if status not in ('normal',*NPC_COLORS):
            raise ValueError(f'Unknown NPC display state: {status}')
        self.apply(vehicle_id,NPC_COLORS.get(status),{
            'vanet.yielding':status,'vanet.yield_reason':reason,'vanet.task_id':task_id or '',
            'vanet.state':status})

    def lincoln(self, vehicle_id, emergency, state=''):
        self.apply(vehicle_id,EMERGENCY_COLOR if emergency else None,{
            'vanet.emergency':'true' if emergency else 'false','vanet.state':state})

    def prune(self, present_ids):
        """车辆已删除时丢弃缓存，防止重用同名 ID 的下一辆车继承旧颜色。"""
        present = set(present_ids)
        for vehicle_id in self.saved.keys()-present:
            del self.saved[vehicle_id]

    def close(self, present_ids=None):
        """仅恢复仍存在且由本对象改动的显示字段；不操作驾驶状态。"""
        try:
            present = set(self.connection.vehicle.getIDList() if present_ids is None else present_ids)
        except Exception:
            LOG.exception('SUMO state display cleanup could not query remaining vehicles')
            return
        for vehicle_id,record in list(self.saved.items()):
            if vehicle_id not in present:
                del self.saved[vehicle_id]
                continue
            try:
                self.apply(vehicle_id,record['color'],record['parameters'])
            except Exception:
                LOG.exception('SUMO state display cleanup failed vehicle=%s',vehicle_id)
            else:
                del self.saved[vehicle_id]
