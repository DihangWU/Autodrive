"""NPC 无线协同入口；receive 只能由 OMNeT++ 实际接收事件调用。"""
import copy
import logging
from vanet.npc.route_conflict_assessment.route_geometry import conflict, project
from vanet.npc.v2x_data_exchange.message_guard import MessageGuard
from vanet.npc.v2x_data_exchange.beacon_schedule import beacon_phase_seconds
from vanet.npc.yielding_action_control.sumo_actions import YieldingActions
from vanet.npc.v2x_data_exchange.range_leases import RangeLeases

LOG = logging.getLogger(__name__)


class NpcCooperation:
    def __init__(self, node_id, config):
        self.node_id = node_id
        self.config = dict(config or {})
        self.guard = MessageGuard(self.config.get('lease_seconds', 10.))
        self.actions = YieldingActions(node_id, self.config)
        self.task = None
        self.lease_until = -1.
        self.next_beacon = None
        self.next_feedback = 0.
        self.sequence = 0
        self.status, self.reason = 'normal', ''
        self.pending = []
        self.recovering = False
        self.last_time = None
        self.conflict_anchor = None
        self.range_leases=RangeLeases('communication_ranges' in self.config,self.config.get('lease_seconds',10.))

    def set_communication_links(self,peers,sim_time):
        self.range_leases.update_links(peers)

    def send(self, kind, now, recipient='broadcast', **fields):
        self.sequence += 1
        task = self.task or {}
        payload = dict(type=kind, source=self.node_id, task_id=task.get('task_id', ''),
                       route_version=task.get('route_version', 0), sequence=self.sequence, sent_at=now)
        payload.update(fields)
        return dict(sender=self.node_id, recipient=recipient, payload=payload)

    def receive(self, message, sim_time):
        if not self.range_leases.can_receive(message.get('source')):return False
        if message.get('type') not in ('emergency', 'notice', 'cancel') or not self.guard.accept(message, sim_time):
            return False
        kind = message['type']
        if kind == 'cancel':
            if self.task and message['task_id'] == self.task['task_id'] and message['route_version'] >= self.task['route_version']:
                self.range_leases.cancel(message)
                if not self.range_leases.enabled or self.range_leases.current(sim_time) is None:
                    self.recovering = True
                    self.reason = message.get('reason', 'task_cancelled')
            return True
        if not message.get('route') or len(message['route']) < 2:
            return False
        new_task = not self.task or (message['task_id'], message['route_version']) != (self.task['task_id'], self.task['route_version'])
        self.range_leases.remember(message,sim_time,new_task)
        self.task = copy.deepcopy(message)
        self.lease_until = sim_time+self.config.get('lease_seconds', 10.)
        if new_task or (self.recovering and self.reason in ('out_of_communication_range','lease_expired')):
            self.recovering = False
            self.conflict_anchor = None
            self.actions.action = None
            self.actions.restore_started=None
            self.status, self.reason = 'received', 'valid_wireless_message'
            self.next_feedback = sim_time
        return True

    def step(self, sim_time, own_state, connection):
        if self.last_time is not None and sim_time < self.last_time:
            raise ValueError('NPC simulation time moved backwards; reset the cooperation instance.')
        self.last_time = sim_time
        messages = []
        if self.next_beacon is None:
            seed = self.config.get('seed', 0)
            spread = self.config.get('beacon_phase_spread_seconds', 0.)
            phase = beacon_phase_seconds(self.node_id, seed, spread)
            self.next_beacon = sim_time + phase
            LOG.info('NPC beacon schedule node=%s seed=%s phase_algorithm=sha256-v1 start_time=%.9f '
                     'phase_spread_seconds=%.9f phase_seconds=%.9f first_due_time=%.9f interval_seconds=%.9f',
                     self.node_id, seed, sim_time, spread, phase, self.next_beacon,
                     self.config.get('beacon_interval', 1.))
        if sim_time+1e-8 >= self.next_beacon:
            state = {k: copy.deepcopy(v) for k, v in own_state.items() if k in
                     ('id','x','y','z','speed','heading','route','lane_id','width','length','in_junction')}
            messages.append(self.send('beacon', sim_time, recipient=self.config.get('rsu_id','rsu0'), state=state))
            self.next_beacon = sim_time+self.config.get('beacon_interval', 1.)
        if self.task is None:
            return messages
        previous = self.status, self.reason
        if self.range_leases.enabled:
            current=self.range_leases.current(sim_time)
            if current is None:self.recovering,self.reason=True,self.range_leases.reason(sim_time)
            else:self.task,self.lease_until=current['message'],current['until']
        if self.status == 'received':
            messages.append(self.send('feedback', sim_time, recipient=self.config.get('rsu_id','rsu0'), status='received', reason=self.reason))
        if sim_time >= self.lease_until:
            self.recovering, self.reason = True, 'lease_expired'
        assessment = conflict(own_state, self.task, self.config)
        if assessment['relevant'] and self.conflict_anchor is None:
            self.conflict_anchor = (assessment.get('prediction') or assessment).get('point')
        if not assessment['relevant'] and self.actions.saved is not None and self.conflict_anchor:
            anchor = project(self.conflict_anchor,self.task['route'])
            emergency = project(self.task.get('position',self.task.get('emergency_position',self.task['route'][0])),self.task['route'])
            if anchor and emergency and anchor['distance'] < 2. and anchor['station']+self.config.get('passed_clearance',8.) > emergency['station']:
                assessment['hold_clearance'] = True
        try:
            if self.recovering:
                self.status = 'recovering'
                if self.actions.restore(connection, own_state, sim_time):
                    self.status = 'normal'
            elif assessment.get('hold_clearance'):
                self.status,self.reason = 'cleared','holding_until_emergency_passes'
                if self.actions.action == 'wait_before_conflict':
                    connection.vehicle.setSpeed(self.node_id,0.)
            elif not assessment['relevant']:
                self.status, self.reason = 'cleared', 'outside_current_conflict'
                if self.actions.saved is not None:
                    self.recovering = True
            else:
                self.status, self.reason = self.actions.apply(connection, own_state, assessment)
        except Exception as error:
            self.status, self.reason = 'unable', f'sumo_action_failed: {error}'
            LOG.exception('NPC action failed node=%s task=%s', self.node_id, self.task['task_id'])
        if previous != (self.status,self.reason):
            LOG.info('NPC node=%s sim_time=%.3f status=%s reason=%s',self.node_id,sim_time,self.status,self.reason)
        if previous != (self.status,self.reason) or sim_time+1e-8 >= self.next_feedback:
            messages.append(self.send('feedback',sim_time,recipient=self.config.get('rsu_id','rsu0'),status=self.status,reason=self.reason))
            self.next_feedback = sim_time+self.config.get('feedback_interval',1.)
        if self.status == 'normal':
            self.task = None
            self.recovering = False
            self.conflict_anchor = None
            self.range_leases.clear()
        return messages

    def snapshot(self):
        return dict(id=self.node_id,status=self.status,reason=self.reason,lease_until=self.lease_until,
                    task_id=self.task.get('task_id') if self.task else None)
