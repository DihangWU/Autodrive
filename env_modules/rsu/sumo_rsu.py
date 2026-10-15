"""通过 SUMO 管理端同步 RSU 图标，使用当前路网的实际偏移。"""

from pathlib import Path
from envinit.sumo.traci.client import process_identity, request


IMAGE_FILE = Path(__file__).resolve().parent / 'assets' / 'rsu.png'


class SumoRsu:
    def __init__(self):
        self.status = request({'action': 'status'})

    def add(self, session, rsus):
        offset_x, offset_y = self.status['offset']
        pois = [dict(id=rsu['id'], x=rsu['x'] + offset_x,
                     y=-rsu['y'] + offset_y, image=str(IMAGE_FILE)) for rsu in rsus]
        return request(dict(action='add_pois', session=session, owner=process_identity(), pois=pois))

    def remove(self, session):
        return request(dict(action='remove_pois', session=session))

    def check(self):
        status = request({'action': 'status'})
        if status['pid'] != self.status['pid']:
            raise RuntimeError('SUMO 管理端已更换，结束本次 RSU 会话。')
        return status
