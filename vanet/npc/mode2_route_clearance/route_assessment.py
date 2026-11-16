"""Mode2 的路径前车与固定合流点判断；只有实际收包后才使用。"""
from vanet.npc.route_conflict_assessment.route_geometry import conflict, project


def on_ramp(state, config):
    if config.get('scenario_role')!='ramp_merging':return False
    road=state.get('road_id')
    if road is None:return True
    return road in config.get('mode2',{}).get('ramp_approach_edges',('39',':1082_9',':1082_16','31',':1061_0'))


def assess(state, task, config, now=None):
    if not on_ramp(state,config):
        result=conflict(state,task,dict(config,corridor_half_width=config.get('mode2',{}).get('road_half_width_metres',4.5)))
        if now is not None and result.get('kind')=='same_direction':
            age=max(0.,min(now-task.get('sent_at',now),config.get('emergency_interval',1.)))
            result['emergency_station']+=max(0.,task.get('speed',0.))*age
            if result['station']<=result['emergency_station']:result['relevant']=False
        return result
    policy=config.get('mode2',{})
    merge=policy['ramp_merge_point']
    at=project(merge,task['route'])
    ego=project(task.get('position',task['route'][0]),task['route'])
    relevant=bool(at and ego and at['height']<=2. and at['distance']<=policy.get('merge_route_tolerance_metres',6.)
                  and ego['station']<at['station']+policy.get('passed_clearance_metres',8.))
    return dict(relevant=relevant,kind='ramp_merging',inside_corridor=False,point=merge,
                station=at['station'] if at else 0.,emergency_station=ego['station'] if ego else 0.)


def passed_anchor(anchor, task, observation, config):
    if not anchor or not observation or not task:
        return False
    at=project(anchor,task['route']);ego=project(observation,task['route'])
    return bool(at and ego and ego['height']<=2. and ego['distance']<=6.
                and ego['station']>at['station']+config.get('mode2',{}).get('passed_clearance_metres',8.))
