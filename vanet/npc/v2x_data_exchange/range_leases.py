"""实际收包建立任务；断联后可短时保持动作，但不能据此收发消息。"""
import copy
import logging
import math

LOG=logging.getLogger(__name__)


class RangeLeases:
    def __init__(self,enabled,lifetime,hold_seconds=0.,node_id=None):
        self.enabled=enabled;self.lifetime=lifetime
        self.peers=set();self.received={};self.departed_until={}
        self.hold_seconds=float(hold_seconds)
        if not math.isfinite(self.hold_seconds) or self.hold_seconds<0:raise ValueError('Invalid out-of-range yielding duration.')
        self.retained={};self.node_id=node_id

    def update_links(self,peers,now):
        self.peers=set(peers)
        if self.enabled:
            for source in self.received.keys()-self.peers:
                value=self.received.pop(source)
                self.departed_until[source]=value['until']
                if value['until']>now and self.hold_seconds>0:
                    self.retained[source]=dict(message=value['message'],until=now+self.hold_seconds)
                    LOG.info('NPC yield hold started node=%s source=%s sim_time=%.3f until=%.3f task=%s',
                             self.node_id,source,now,now+self.hold_seconds,value['message']['task_id'])

    def can_receive(self,source):
        return not self.enabled or source in self.peers

    def remember(self,message,now,new_task):
        if new_task:self.clear()
        if self.retained:
            LOG.info('NPC yield hold replaced by received message node=%s source=%s sim_time=%.3f',self.node_id,message['source'],now)
        self.retained.clear()
        self.departed_until.pop(message['source'],None)
        self.received[message['source']]=dict(message=copy.deepcopy(message),until=now+self.lifetime)

    def active(self,now):
        available=[value for source,value in self.received.items() if source in self.peers and value['until']>now]
        return max(available,key=lambda value:(value['message']['sent_at'],value['message']['sequence'])) if available else None

    def current(self,now):
        live=self.active(now)
        if live is not None:return live
        available=[value for value in self.retained.values() if value['until']>now+1e-8]
        return max(available,key=lambda value:(value['message']['sent_at'],value['message']['sequence'])) if available else None

    def holding(self,now):
        return self.active(now) is None and self.current(now) is not None

    def cancel(self,message):
        if message.get('scope','task')=='task':self.clear()
        else:
            self.received.pop(message['source'],None);self.departed_until.pop(message['source'],None)
            self.retained.pop(message['source'],None)

    def reason(self,now):
        if any(value['until']<=now+1e-8 for value in self.retained.values()):return 'out_of_range_yield_expired'
        return 'out_of_communication_range' if any(until>now for until in self.departed_until.values()) else 'lease_expired'

    def clear(self):
        self.received.clear();self.departed_until.clear();self.retained.clear()
