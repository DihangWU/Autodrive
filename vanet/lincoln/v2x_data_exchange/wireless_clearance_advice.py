"""合并真实 V2V 反馈和 RSU 汇总；路径资格只撤销旧反馈，不生成接收。"""
import copy
from .clearance_advice import ClearanceAdvice


class WirelessClearanceAdvice(ClearanceAdvice):
    def __init__(self,configuration=None,source='rsu0'):
        self.direct={}
        self.paths=None
        self.rsu_connected=False
        super().__init__(configuration,source)

    def reset(self):
        self.direct.clear()
        return super().reset()

    def set_paths(self,paths,rsu_connected):
        self.paths=dict(paths);self.rsu_connected=bool(rsu_connected)
        self.direct={name:advice for name,advice in self.direct.items() if self.paths.get(name)=='direct'}
        if not self.rsu_connected:self.message=None
        elif self.message:
            self.message['vehicles']={name:status for name,status in self.message['vehicles'].items() if self.paths.get(name)=='rsu'}
            self.message['conflict_points']={name:point for name,point in self.message['conflict_points'].items() if name in self.message['vehicles']}

    def receive(self,message,sim_time,task_id,route_version):
        if message.get('type')=='feedback':
            source=message.get('source')
            if not isinstance(source,str) or not source or source==self.source:return False
            if self.paths is not None and self.paths.get(source)!='direct':return False
            if message.get('status') not in ('received','yielding','cleared','unable','recovering','normal'):return False
            # Local conversion for the common advice evaluator only. This is not
            # a new radio packet, RSU reception or synthetic delivery event.
            normalized=copy.deepcopy(message)
            normalized.update(type='clearance',vehicles={source:message['status']},
                conflict_points={source:message.get('conflict_point') or message.get('position')})
            advice=self.direct.get(source) or ClearanceAdvice(self.configuration,source)
            if not advice.receive(normalized,sim_time,task_id,route_version):return False
            self.direct[source]=advice
            return True
        if self.paths is not None and not self.rsu_connected:return False
        if not super().receive(message,sim_time,task_id,route_version):return False
        if self.paths is not None:self.set_paths(self.paths,self.rsu_connected)
        return True

    def evaluate(self,*args):
        states=[]
        rsu=super().evaluate(*args)
        if rsu['active']:states.append(dict(rsu))
        for advice in self.direct.values():
            state=advice.evaluate(*args)
            if state['active']:
                state=dict(state);state['reason']='pending_direct_feedback';states.append(state)
        if not states:return self.state
        self.state=dict(min(states,key=lambda state:state['speed_limit_kmh']))
        self.state['vehicles']=sorted({name for state in states for name in state['vehicles']})
        if len({state['reason'] for state in states})>1:self.state['reason']='pending_wireless_feedback'
        return self.state
