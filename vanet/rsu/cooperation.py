"""RSU 协同入口：无线信标筛选、定向通知、反馈汇总与有效任务回补。"""
import copy
import logging
from vanet.npc.route_conflict_assessment.route_geometry import conflict, project
from vanet.npc.v2x_data_exchange.message_guard import MessageGuard
from vanet.rsu.vehicle_beacon_tracking.beacon_cache import BeaconCache

LOG = logging.getLogger(__name__)


class RsuCooperation:
    def __init__(self, config):
        self.config = dict(config or {})
        self.node_id = self.config.get('rsu',{}).get('id', self.config.get('rsu_id','rsu0'))
        self.guard = MessageGuard(self.config.get('lease_seconds',10.))
        self.beacons = BeaconCache(self.config.get('beacon_lifetime',3.))
        self.task = None
        self.lease_until = -1.
        self.sequence = 0
        self.selected = {}
        self.feedback = {}
        self.pending = []
        self.next_summary = 0.
        self.last_time = None
        self.range_limited='communication_ranges' in self.config
        self.range_peers=set()
        self.relay_targets=None

    def set_relay_targets(self,targets,sim_time):
        self.relay_targets=set(targets)
        for name in list(self.beacons.entries):
            if name not in self.relay_targets:del self.beacons.entries[name]
        for name in list(self.selected):
            if name not in self.relay_targets:
                LOG.info('RSU releases relay target sim_time=%.3f target=%s reason=communication_path_changed',sim_time,name)
                del self.selected[name];self.feedback.pop(name,None)
        self.pending=[message for message in self.pending if message['recipient'] in self.relay_targets]

    def set_communication_links(self,peers,sim_time):
        self.range_peers=set(peers)
        if self.range_limited and self.task and self.task['source'] not in self.range_peers:
            self.cancel_selected(sim_time,'emergency_out_of_rsu_range',scope='selection')
            self.task=None

    def send(self, kind, now, recipient, **fields):
        self.sequence += 1
        task = self.task or {}
        payload = dict(type=kind,source=self.node_id,task_id=task.get('task_id',''),
                       route_version=task.get('route_version',0),sequence=self.sequence,sent_at=now)
        payload.update(fields)
        return dict(sender=self.node_id,recipient=recipient,payload=payload)

    def cancel_selected(self, now, reason, scope='task'):
        for name in self.selected:
            self.pending.append(self.send('cancel',now,name,scope=scope,reason=reason))
        self.selected.clear()
        self.feedback.clear()

    def receive(self, message, sim_time):
        if self.range_limited and message.get('source') not in self.range_peers:return False
        kind = message.get('type')
        if kind in ('beacon','feedback') and self.relay_targets is not None and message.get('source') not in self.relay_targets:return False
        if kind not in ('beacon','emergency','feedback','cancel','identity') or not self.guard.accept(message,sim_time):
            return False
        if kind == 'beacon':
            return self.beacons.receive(message,sim_time)
        if kind == 'emergency':
            if not isinstance(message.get('route'),list) or len(message['route']) < 2:
                return False
            if self.task and message['task_id'] != self.task['task_id']:
                self.cancel_selected(sim_time,'task_replaced')
            elif self.task and message['route_version'] > self.task['route_version']:
                self.feedback.clear()
                for item in self.selected.values():
                    item['next_notice'] = sim_time
                    item['attempts'] = item['backfill_attempts'] = 0
                    item['anchor'] = None
            self.task = copy.deepcopy(message)
            self.lease_until = sim_time+self.config.get('lease_seconds',10.)
            return True
        if kind == 'cancel':
            if self.task and message['task_id'] == self.task['task_id'] and message['route_version'] >= self.task['route_version']:
                self.cancel_selected(sim_time,message.get('reason','cancelled'))
                self.task = None
            return True
        if kind == 'feedback':
            if not self.task or message['task_id'] != self.task['task_id'] or message['route_version'] != self.task['route_version']:
                return False
            if message['source'] not in self.selected:
                return False
            self.feedback[message['source']] = dict(status=message.get('status','unknown'),
                                                    reason=message.get('reason',''),received_at=sim_time)
            item = self.selected[message['source']]
            if item['backfill_attempts']:
                LOG.info('RSU feedback resets backfill sim_time=%.3f target=%s task=%s previous_attempts=%s',
                         sim_time,message['source'],self.task['task_id'],item['backfill_attempts'])
            item['backfill_attempts'] = 0
        return True

    def step(self, sim_time):
        if self.last_time is not None and sim_time < self.last_time:
            raise ValueError('RSU simulation time moved backwards; reset cooperation instance.')
        self.last_time = sim_time
        messages, self.pending = self.pending, []
        states = self.beacons.fresh(sim_time)
        if self.range_limited:states={name:state for name,state in states.items() if name in self.range_peers}
        if self.relay_targets is not None:states={name:state for name,state in states.items() if name in self.relay_targets}
        if self.task is None:
            return messages
        if sim_time >= self.lease_until:
            self.cancel_selected(sim_time,'emergency_update_expired',scope='selection')
            messages.extend(self.pending);self.pending=[]
            self.task = None
            return messages
        wanted = {}
        for name,state in states.items():
            if name == self.task['source']:
                continue
            assessment = conflict(state,self.task,self.config)
            if assessment['relevant']:
                wanted[name] = assessment
            elif name in self.selected:
                # 让行成功的停车/换道车辆仍保持通知，直到 Lincoln 经过原冲突点。
                anchor = self.selected[name].get('anchor')
                projected = project(anchor,self.task['route']) if anchor else None
                emergency = project(self.task.get('position',self.task.get('emergency_position',self.task['route'][0])),self.task['route'])
                if projected and emergency and projected['distance'] < 2. and projected['station']+self.config.get('passed_clearance',8.) > emergency['station']:
                    wanted[name] = self.selected[name]['assessment']
        for name in list(self.selected):
            if name not in wanted:
                messages.append(self.send('cancel',sim_time,name,scope='selection',reason='no_longer_relevant'))
                del self.selected[name]
                self.feedback.pop(name,None)
        for name,assessment in wanted.items():
            item = self.selected.setdefault(name,dict(next_notice=sim_time,attempts=0,backfill_attempts=0,
                anchor=(assessment.get('prediction') or assessment).get('point'),assessment=assessment))
            if item['anchor'] is None:
                item['anchor'] = (assessment.get('prediction') or assessment).get('point')
                item['assessment'] = assessment
            if sim_time+1e-8 >= item['next_notice']:
                feedback = self.feedback.get(name)
                feedback_current = feedback is not None and sim_time-feedback['received_at'] <= self.config.get('feedback_lifetime',3.)
                limit = self.config.get('max_backfill_attempts',3)
                backfill = item['attempts'] > 0 and not feedback_current and item['backfill_attempts'] < limit
                if backfill:
                    item['backfill_attempts'] += 1
                if item['attempts'] == 0:
                    reason = 'initial_notice'
                elif feedback_current:
                    reason = 'feedback_current'
                elif not backfill:
                    reason = 'backfill_limit_reached'
                else:
                    reason = 'feedback_expired' if feedback else 'feedback_missing'
                fields = {key:copy.deepcopy(self.task[key]) for key in
                          ('route','position','emergency_position','speed','eta','destination') if key in self.task}
                fields.update(emergency_source=self.task['source'],conflict_kind=assessment['kind'],
                              lease_seconds=self.config.get('lease_seconds',10.),
                              backfill=backfill)
                messages.append(self.send('notice',sim_time,name,**fields))
                item['attempts'] += 1
                item['next_notice'] = sim_time+self.config.get('notice_interval',1.)
                LOG.info('RSU notice sim_time=%.3f target=%s task=%s route_version=%s notice_count=%s '
                         'backfill=%s backfill_attempts=%s max_backfill_attempts=%s reason=%s',
                         sim_time,name,self.task['task_id'],self.task['route_version'],item['attempts'],
                         backfill,item['backfill_attempts'],limit,reason)
        if self.relay_targets is not None and not self.selected:return messages
        if sim_time+1e-8 >= self.next_summary:
            lifetime = self.config.get('feedback_lifetime',3.)
            statuses = {name:(self.feedback[name]['status'] if name in self.feedback and
                         sim_time-self.feedback[name]['received_at'] <= lifetime else 'unknown') for name in self.selected}
            conflict_points = {name:copy.deepcopy(item.get('anchor') or item['assessment'].get('point'))
                               for name,item in self.selected.items()}
            messages.append(self.send('clearance',sim_time,self.task['source'],vehicles=statuses,
                                      conflict_points=conflict_points,
                                      all_cleared=bool(statuses) and all(s=='cleared' for s in statuses.values()),
                                      selected_count=len(statuses),known_vehicle_count=len(states)))
            self.next_summary = sim_time+self.config.get('summary_interval',1.)
        return messages

    def snapshot(self):
        return dict(id=self.node_id,beacon_count=len(self.beacons.entries),selected=list(self.selected),
                    feedback=copy.deepcopy(self.feedback),task_id=self.task.get('task_id') if self.task else None)
