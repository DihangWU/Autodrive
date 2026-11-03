"""交通指标与几何碰撞事件；分别保留 SUMO 的原生事件。"""
import math


def rectangles_overlap(a, b):
    if abs(a['z'] - b['z']) > (a.get('height', 1.5) + b.get('height', 1.5)) / 2:
        return False
    def corners(v):
        angle = math.radians(v['heading']); c, s = math.cos(angle), math.sin(angle)
        return [(v['x'] + u*c - w*s, v['y'] + u*s + w*c)
                for u, w in [(-v['length']/2,-v['width']/2),(-v['length']/2,v['width']/2),
                             (v['length']/2,v['width']/2),(v['length']/2,-v['width']/2)]]
    first, second = corners(a), corners(b)
    for points in (first, second):
        for p, q in zip(points, points[1:] + points[:1]):
            axis = (-(q[1]-p[1]), q[0]-p[0])
            left = [x*axis[0]+y*axis[1] for x,y in first]
            right = [x*axis[0]+y*axis[1] for x,y in second]
            if max(left) <= min(right) or max(right) <= min(left):
                return False
    return True


class Metrics:
    def __init__(self, log, config):
        self.log, self.config = log, config
        self.overlaps = set()
        self.speeds = {}
        self.braking = set()
        self.values = dict(arrived=False, elapsed_seconds=0., messages_sent=0, deliveries=0,
                           mean_delay_seconds=0., geometric_collision_events=0,
                           sumo_collision_events=0, hard_brake_events=0, teleports=0,
                           npc_waiting_vehicle_seconds=0., status='prepared',
                           npc_seen_count=0, npc_completed_trip_count=0, npc_incomplete_trip_count=0,
                           npc_active_count=0, npc_teleporting_count=0, npc_missing_count=0,
                           npc_completed_trip_seconds=0., npc_mean_completed_trip_seconds=None,
                           npc_observed_distance_metres=0., npc_observed_vehicle_seconds=0.,
                           npc_mean_speed_mps=None, npc_yield_received_count=0,
                           npc_yield_response_count=0, npc_yield_cleared_count=0,
                           npc_yield_unable_count=0, npc_yield_response_latency_count=0,
                           npc_yield_clearance_latency_count=0, npc_mean_yield_response_seconds=None,
                           npc_mean_yield_clearance_seconds=None)
        self.delay_sum = 0.
        self.previous_sumo_collisions = set()
        self.last_sample_time = None
        self.npc_trips = {}
        self.yield_records = {}
        self.active_yield_tasks = {}
        self.next_npc_report = 0.

    def delivery(self, message):
        delay = max(0., message['received_at'] - message['sent_at'])
        self.values['deliveries'] += 1
        self.delay_sum += delay
        self.values['mean_delay_seconds'] = self.delay_sum / self.values['deliveries']

    @staticmethod
    def simulation_events(sumo, method):
        """可选生命周期接口；旧离线 mock 没有具体返回值时按空事件处理。"""
        function = getattr(sumo.simulation, method, None)
        if function is None:
            return set()
        result = function()
        return set(result) if isinstance(result, (list, tuple, set, frozenset)) else set()

    def trip_summary(self, record, now):
        observed = record['observed_seconds']
        end = record['arrived_at'] if record['arrived_at'] is not None else now
        return dict(vehicle=record['id'], first_seen_at=record['first_seen_at'],
                    last_seen_at=record['last_seen_at'], arrived_at=record['arrived_at'],
                    last_disappeared_at=record['last_disappeared_at'], status=record['status'],
                    completed=record['arrived_at'] is not None,
                    elapsed_since_first_seen=max(0.,end-record['first_seen_at']),
                    trip_seconds=(end-record['first_seen_at'] if record['arrived_at'] is not None else None),
                    waiting_seconds=record['waiting_seconds'],
                    observed_seconds=observed, distance_metres=record['distance_metres'],
                    mean_observed_speed_mps=(record['distance_metres']/observed if observed else None),
                    teleport_count=record['teleport_count'], distance_source='speed_integral')

    def update_npc_trips(self, now, states, arrived, teleport_start, teleport_end):
        current = {state['id']:state for state in states if state.get('role') == 'npc'}
        for name,state in current.items():
            speed = max(0.,float(state['speed']))
            record = self.npc_trips.get(name)
            if record is None:
                record = dict(id=name,first_seen_at=now,last_seen_at=now,arrived_at=None,
                              last_disappeared_at=None,status='active',last_speed=speed,
                              observed_seconds=0.,distance_metres=0.,waiting_seconds=0.,teleport_count=0)
                self.npc_trips[name] = record
                self.log.emit('npc_trip_observed',now,vehicle=name,first_seen_at=now,
                              measurement='first observed traffic frame, not inferred departure')
            else:
                # 只有连续可见帧才积分；teleport/不可见间隔不计入里程、均速和等待。
                continuous = (record['status']=='active' and record['last_seen_at']==self.last_sample_time
                              and name not in teleport_start and name not in teleport_end)
                dt = max(0.,now-record['last_seen_at']) if continuous else 0.
                record['observed_seconds'] += dt
                record['distance_metres'] += .5*(record['last_speed']+speed)*dt
                threshold = self.config.get('waiting_speed_mps',.1)
                previous_stopped = record['last_speed'] < threshold
                stopped = speed < threshold
                record['waiting_seconds'] += .5*(int(previous_stopped)+int(stopped))*dt
                if record['status'] in ('teleporting','missing'):
                    self.log.emit('npc_reappeared',now,vehicle=name,previous_status=record['status'])
                record['last_seen_at'],record['last_speed'],record['status'] = now,speed,'active'
        for name in teleport_start:
            if name in self.npc_trips:
                record=self.npc_trips[name]
                record['teleport_count'] += 1
                record['status'] = 'teleporting'
        for name,record in self.npc_trips.items():
            if record['arrived_at'] is not None:
                continue
            if name in arrived:
                record['arrived_at'],record['status'] = now,'arrived'
                record['last_disappeared_at'] = now
                self.log.emit('npc_trip_completed',now,**self.trip_summary(record,now))
            elif name not in current:
                if record['status']=='active':
                    record['status']='missing'
                    record['last_disappeared_at']=now
                    self.log.emit('npc_disappeared_without_arrival',now,vehicle=name,
                                  reason='not present and no SUMO arrival event; trip remains incomplete')
                elif record['status']=='teleporting' and record['last_disappeared_at'] is None:
                    record['last_disappeared_at']=now
                    self.log.emit('npc_temporarily_absent',now,vehicle=name,reason='teleport')
        records=list(self.npc_trips.values())
        completed=[item for item in records if item['arrived_at'] is not None]
        total_trip=sum(item['arrived_at']-item['first_seen_at'] for item in completed)
        observed=sum(item['observed_seconds'] for item in records)
        distance=sum(item['distance_metres'] for item in records)
        self.values.update(npc_seen_count=len(records),npc_completed_trip_count=len(completed),
                           npc_incomplete_trip_count=len(records)-len(completed),npc_active_count=len(current),
                           npc_teleporting_count=sum(item['status']=='teleporting' for item in records),
                           npc_missing_count=sum(item['status']=='missing' for item in records),
                           npc_completed_trip_seconds=total_trip,
                           npc_mean_completed_trip_seconds=total_trip/len(completed) if completed else None,
                           npc_observed_vehicle_seconds=observed,npc_observed_distance_metres=distance,
                           npc_mean_speed_mps=distance/observed if observed else None,
                           npc_waiting_vehicle_seconds=sum(item['waiting_seconds'] for item in records))
        if now+1e-8 >= self.next_npc_report:
            for record in records:
                self.log.emit('npc_trip_statistics',now,level='debug',**self.trip_summary(record,now))
            self.next_npc_report=now+self.config.get('npc_metrics_interval',1.)

    def note_yield_state(self, vehicle_id, status, now, task_id, reason=''):
        """记录实际无线接收驱动的状态；调用者提供本轮相对仿真时间。

        received 建立延迟参考；缺失该事件时仍记录动作，但延迟保持未知。
        """
        if status not in ('received','yielding','cleared','unable','recovering','normal'):
            raise ValueError(f'Unknown yielding state: {status}')
        if not math.isfinite(float(now)):
            raise ValueError('Yield state timestamp must be finite.')
        task_id = task_id or self.active_yield_tasks.get(vehicle_id)
        if not task_id:
            return False
        key=(vehicle_id,task_id)
        record=self.yield_records.get(key)
        if record is None:
            if status in ('normal','recovering'):
                return False
            record=dict(vehicle=vehicle_id,task_id=task_id,received_at=None,response_at=None,
                        cleared_at=None,unable_at=None,restored_at=None,status=None,reason='',
                        last_update=now,response_seconds=None,clearance_seconds=None)
            self.yield_records[key]=record
            self.active_yield_tasks[vehicle_id]=task_id
        if now < record['last_update']:
            raise ValueError('Yield state time moved backwards.')
        if status=='received' and record['received_at'] is not None:
            return False
        if record['status']==status and record['reason']==reason:
            return False
        record['last_update']=now
        if status=='received' and record['received_at'] is None:
            record['received_at']=now
        if status in ('yielding','cleared','unable') and record['response_at'] is None:
            record['response_at']=now
        if status=='cleared' and record['cleared_at'] is None:
            record['cleared_at']=now
        if status=='unable' and record['unable_at'] is None:
            record['unable_at']=now
        if status=='normal':
            record['restored_at']=now
            if self.active_yield_tasks.get(vehicle_id)==task_id:
                del self.active_yield_tasks[vehicle_id]
        record['status'],record['reason']=status,reason
        received=record['received_at']
        for event,metric in (('response_at','response_seconds'),('cleared_at','clearance_seconds')):
            if received is not None and record[event] is not None and record[event]>=received:
                record[metric]=record[event]-received
        self.log.emit('npc_yield_state',now,**record)
        records=list(self.yield_records.values())
        responses=[item['response_seconds'] for item in records if item['response_seconds'] is not None]
        clearances=[item['clearance_seconds'] for item in records if item['clearance_seconds'] is not None]
        self.values.update(npc_yield_received_count=sum(item['received_at'] is not None for item in records),
                           npc_yield_response_count=sum(item['response_at'] is not None for item in records),
                           npc_yield_cleared_count=sum(item['cleared_at'] is not None for item in records),
                           npc_yield_unable_count=sum(item['unable_at'] is not None for item in records),
                           npc_yield_response_latency_count=len(responses),npc_yield_clearance_latency_count=len(clearances),
                           npc_mean_yield_response_seconds=sum(responses)/len(responses) if responses else None,
                           npc_mean_yield_clearance_seconds=sum(clearances)/len(clearances) if clearances else None)
        return True

    def sample(self, now, states, sumo):
        if self.last_sample_time is not None:
            if now < self.last_sample_time:
                raise ValueError('Traffic metrics time moved backwards; create a new Metrics for each run.')
            if now == self.last_sample_time:
                return
        self.values['elapsed_seconds'] = now
        pairs=set()
        for i,a in enumerate(states):
            for b in states[i+1:]:
                if rectangles_overlap(a,b):
                    pair=tuple(sorted((a['id'],b['id'])));pairs.add(pair)
                    if pair not in self.overlaps:
                        self.values['geometric_collision_events'] += 1
                        self.log.emit('geometric_collision_started',now,vehicles=pair)
        for pair in self.overlaps-pairs:self.log.emit('geometric_collision_ended',now,vehicles=pair)
        self.overlaps=pairs
        braking=set()
        for state in states:
            previous=self.speeds.get(state['id'])
            if previous and previous[0]==self.last_sample_time and now>previous[0] and (state['speed']-previous[1])/(now-previous[0]) < -self.config['hard_brake_mps2']:
                braking.add(state['id'])
                if state['id'] not in self.braking:
                    self.values['hard_brake_events']+=1
                    self.log.emit('hard_brake',now,vehicle=state['id'])
            self.speeds[state['id']]=(now,state['speed'])
        self.braking=braking
        collisions=self.simulation_events(sumo,'getCollidingVehiclesIDList')
        new=collisions-self.previous_sumo_collisions
        if new:
            self.values['sumo_collision_events']+=1
            self.log.emit('sumo_collision_report',now,vehicles=sorted(collisions))
        self.previous_sumo_collisions=collisions
        teleports=self.simulation_events(sumo,'getStartingTeleportIDList')
        self.values['teleports']+=len(teleports)
        if teleports:self.log.emit('sumo_teleport',now,vehicles=sorted(teleports))
        self.update_npc_trips(now,states,self.simulation_events(sumo,'getArrivedIDList'),
                              teleports,self.simulation_events(sumo,'getEndingTeleportIDList'))
        self.log.emit('traffic_frame',now,level='debug',vehicles=states)
        self.last_sample_time=now
