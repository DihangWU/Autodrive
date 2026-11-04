"""几何资格只撤销通信许可；许可内容只能由实际无线收包建立。"""
import copy


class RangeLeases:
    def __init__(self,enabled,lifetime):
        self.enabled=enabled;self.lifetime=lifetime
        self.peers=set();self.received={};self.departed_until={}

    def update_links(self,peers):
        self.peers=set(peers)
        if self.enabled:
            for source in self.received.keys()-self.peers:
                self.departed_until[source]=self.received.pop(source)['until']

    def can_receive(self,source):
        return not self.enabled or source in self.peers

    def remember(self,message,now,new_task):
        if new_task:self.clear()
        self.departed_until.pop(message['source'],None)
        self.received[message['source']]=dict(message=copy.deepcopy(message),until=now+self.lifetime)

    def current(self,now):
        available=[value for source,value in self.received.items() if source in self.peers and value['until']>now]
        return max(available,key=lambda value:(value['message']['sent_at'],value['message']['sequence'])) if available else None

    def cancel(self,message):
        if message.get('scope','task')=='task':self.clear()
        else:
            self.received.pop(message['source'],None);self.departed_until.pop(message['source'],None)

    def reason(self,now):
        return 'out_of_communication_range' if any(until>now for until in self.departed_until.values()) else 'lease_expired'

    def clear(self):
        self.received.clear();self.departed_until.clear()
