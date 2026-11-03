"""只保存由无线接收入口提供的 NPC 信标，禁止读取交通全局状态。"""
import copy
import math


class BeaconCache:
    def __init__(self, lifetime=3.):
        self.lifetime = lifetime
        self.entries = {}

    def receive(self, message, now):
        state = message.get('state')
        if not isinstance(state, dict):
            return False
        try:
            if not all(math.isfinite(float(state[k])) for k in ('x','y','speed','heading')):
                return False
        except (KeyError, TypeError, ValueError):
            return False
        source = str(message['source'])
        value = copy.deepcopy(state)
        value['id'] = source
        self.entries[source] = dict(state=value, received_at=float(message['sent_at']))
        return True

    def fresh(self, now):
        self.entries = {name:item for name,item in self.entries.items()
                        if now-item['received_at'] <= self.lifetime}
        return {name:item['state'] for name,item in self.entries.items()}
