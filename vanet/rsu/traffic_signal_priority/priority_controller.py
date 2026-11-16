"""RSU 实际收到紧急消息后预先放行路径，Lincoln 车尾出路口后逐个恢复。"""
import copy

from vanet.npc.route_conflict_assessment.route_geometry import project
from vanet.rsu.traffic_signal_priority.route_signals import RouteSignalIndex
from vanet.rsu.traffic_signal_priority.signal_program import save_program,restore_program


class TrafficSignalPriority:
    def __init__(self, config, event_sink):
        self.policy=config.get('rsu_traffic_signal_priority',{})
        self.emit=event_sink
        self.index=None
        self.network_file=None
        self.connection=None
        self.task=None
        self.targets=[]
        self.active={}
        self.passed=set()
        self.next_check=0.

    def receive(self, message, now, connection, network_file, position):
        # Called only after RsuCooperation accepts an actual in-range delivery.
        if not self.policy.get('enabled',True) or message.get('type')!='emergency' or message.get('source')!='lincoln0':return
        identity=(message['task_id'],message['route_version'])
        if self.task and identity==(self.task['task_id'],self.task['route_version']):return
        self.release_all(now,'route_replaced')
        self.connection=connection
        if self.index is None or self.network_file!=str(network_file):
            self.index=RouteSignalIndex(network_file);self.network_file=str(network_file)
        self.task=copy.deepcopy(message)
        self.targets=self.index.match(message['route'],self.policy.get('route_tolerance_metres',2.5))
        self.emit('rsu_signal_priority_received',now,task_id=message['task_id'],route_version=message['route_version'],
                  source=message['source'],controllers=[item['controller'] for item in self.targets])
        self.update(position or message.get('position'),now)

    def update(self, position, now):
        if not self.task or not position:return
        at=project(position,self.task['route'])
        if not at or at['distance']>self.policy.get('tracking_tolerance_metres',6.) or at['height']>2.:return
        rear_station=at['station']-float(position.get('length',4.9))/2
        for target in self.targets:
            name=target['controller']
            if name in self.passed:continue
            if rear_station>target['exit_station']+self.policy.get('passed_clearance_metres',1.):
                self.restore(name,now,'lincoln_cleared_junction');self.passed.add(name)
                continue
            if name not in self.active:
                saved=save_program(self.connection,name)
                # Store first so cancellation can restore even a partial write.
                self.active[name]=dict(target=target,saved=saved)
                self.connection.trafficlight.setRedYellowGreenState(name,target['green_state'])
                self.emit('rsu_signal_green',now,controller=name,task_id=self.task['task_id'],
                          route_version=self.task['route_version'],incoming=target['incoming'],outgoing=target['outgoing'],
                          green_indices=target['green_indices'],state=target['green_state'],original=saved,
                          distance_metres=round(target['entry_station']-at['station'],3))
            elif now>=self.next_check:
                if self.connection.trafficlight.getRedYellowGreenState(name)!=target['green_state']:
                    self.connection.trafficlight.setRedYellowGreenState(name,target['green_state'])
                    self.emit('rsu_signal_green_reapplied',now,controller=name,state=target['green_state'])
        if now>=self.next_check:self.next_check=now+self.policy.get('state_check_interval_seconds',.25)

    def restore(self, name, now, reason):
        if name not in self.active:return
        record=self.active[name]
        restore_program(self.connection,name,record['saved'])
        del self.active[name]
        self.emit('rsu_signal_restored',now,controller=name,reason=reason,original=record['saved'])

    def release_all(self, now, reason):
        # Invalidate authorization before restoring; a failed restoration stays
        # tracked in active and can be retried during reset/shutdown.
        self.task=None;self.targets=[];self.passed.clear()
        failures=[]
        for name in list(self.active):
            try:self.restore(name,now,reason)
            except Exception as error:failures.append(f'{name}: {error}')
        if failures:raise RuntimeError('RSU signal restoration failed: '+'; '.join(failures))

    def snapshot(self):
        return dict(task_id=self.task['task_id'] if self.task else None,
                    active=sorted(self.active),passed=sorted(self.passed),
                    route_controllers=[item['controller'] for item in self.targets])
