"""把实际收到的 RSU 净空反馈转换为限速建议，不代替车辆物理避障。"""
import copy
import math


DEFAULT_CLEARANCE_ADVICE = dict(message_lifetime_seconds=3.,lookahead_metres=120.,
    stopping_margin_metres=8.,braking_deceleration_mps2=2.5,
    route_tolerance_metres=2.,height_tolerance_metres=2.)


def project(position,samples,distances,begin,end):
    """只在本车当前路线邻域投影，避免把回环路线的远处路段当作眼前冲突。"""
    best=None
    for index,(first,second) in enumerate(zip(samples,samples[1:])):
        if distances[index+1]<begin or distances[index]>end:continue
        a,b=first.center,second.center
        dx,dy=b[0]-a[0],b[1]-a[1]
        squared=dx*dx+dy*dy
        if squared<1e-8:continue
        fraction=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/squared))
        point=tuple(a[axis]+fraction*(b[axis]-a[axis]) for axis in range(3))
        lateral=math.hypot(position[0]-point[0],position[1]-point[1])
        height=abs(position[2]-point[2])
        candidate=dict(station=distances[index]+fraction*(distances[index+1]-distances[index]),
                       lateral=lateral,height=height)
        score=lateral*lateral+height*height
        if best is None or score<best[0]:best=(score,candidate)
    return best[1] if best is not None else None


class ClearanceAdvice:
    def __init__(self,configuration=None,source='rsu0'):
        self.configuration=dict(DEFAULT_CLEARANCE_ADVICE)
        self.configuration.update(configuration or {})
        self.source=source
        self.message=None
        self.state=self.inactive('no_feedback')

    def inactive(self,reason):
        self.state=dict(active=False,speed_limit_kmh=None,reason=reason,
                        vehicles=[],nearest_distance_metres=None)
        return self.state

    def reset(self):
        self.message=None
        return self.inactive('no_feedback')

    def receive(self,message,sim_time,task_id,route_version):
        if (message.get('type')!='clearance' or message.get('source')!=self.source or
                message.get('task_id')!=task_id or message.get('route_version')!=route_version):
            return False
        try:
            sent=float(message['sent_at'])
            sequence=int(message['sequence'])
        except (KeyError,TypeError,ValueError,OverflowError):return False
        age=sim_time-sent
        if not math.isfinite(age) or age<0 or age>self.configuration['message_lifetime_seconds']:return False
        if not isinstance(message.get('vehicles'),dict) or not isinstance(message.get('conflict_points'),dict):return False
        previous=self.message
        if previous and previous.get('task_id')==task_id and previous.get('route_version')==route_version:
            if (sent,sequence)<=(float(previous['sent_at']),int(previous['sequence'])):return False
        self.message=copy.deepcopy(message)
        return True

    def evaluate(self,sim_time,task_id,route_version,position,samples,distances,progress):
        message=self.message
        if message is None:return self.inactive('no_feedback')
        if message.get('task_id')!=task_id or message.get('route_version')!=route_version:
            return self.inactive('different_task_or_route')
        age=sim_time-float(message['sent_at'])
        if age<0 or age>self.configuration['message_lifetime_seconds']:
            return self.inactive('future_feedback' if age<0 else 'expired_feedback')
        if len(samples)<2 or len(distances)!=len(samples):return self.inactive('route_unavailable')
        ego=project(position,samples,distances,max(0.,progress-10.),progress+45.)
        if ego is None or ego['height']>self.configuration['height_tolerance_metres']:
            return self.inactive('position_off_route')
        horizon=self.configuration['lookahead_metres']
        candidates=[]
        for vehicle,status in message['vehicles'].items():
            if status in ('cleared','normal'):continue
            point=message['conflict_points'].get(vehicle)
            try:
                xyz=tuple(float(point[axis]) for axis in ('x','y','z'))
            except (KeyError,TypeError,ValueError):continue
            if not all(math.isfinite(value) for value in xyz):continue
            conflict=project(xyz,samples,distances,max(0.,ego['station']-10.),ego['station']+horizon)
            if conflict is None:continue
            distance=conflict['station']-ego['station']
            if (distance<=0 or distance>horizon or
                    conflict['lateral']>self.configuration['route_tolerance_metres'] or
                    conflict['height']>self.configuration['height_tolerance_metres']):continue
            candidates.append((distance,vehicle))
        if not candidates:return self.inactive('no_pending_conflict_ahead')
        distance=min(item[0] for item in candidates)
        stopping_distance=max(0.,distance-self.configuration['stopping_margin_metres'])
        speed=math.sqrt(2.*self.configuration['braking_deceleration_mps2']*stopping_distance)*3.6
        self.state=dict(active=True,speed_limit_kmh=speed,reason='pending_rsu_clearance',
                        vehicles=sorted(item[1] for item in candidates),nearest_distance_metres=distance,
                        feedback_age_seconds=age)
        return self.state
