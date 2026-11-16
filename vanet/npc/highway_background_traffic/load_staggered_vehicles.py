"""加载红框起点的两列联网背景车，配置与避让策略分别管理。"""
import copy
import json

from vanet.project_paths import SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE


def load_staggered_traffic():
    configuration = json.loads(SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text(encoding='utf-8'))
    vehicles = [dict(copy.deepcopy(configuration['vehicle_defaults']), **entry)
                for entry in configuration['vehicles']]
    count = configuration['vehicles_per_lane']
    if type(count) is not int or count < 1 or len(vehicles)!=2*count or any(
            sum(v['lane_index']==lane for v in vehicles)!=count for lane in (1,2)):
        raise ValueError('Highway background traffic requires the configured vehicle count in both SUMO lanes 1 and 2.')
    if len({v['id'] for v in vehicles}) != len(vehicles):
        raise ValueError('Highway background vehicle ids must be unique.')
    pattern=configuration['departure_pattern']
    if not pattern or any(branch not in ('straight','city') for branch in pattern):
        raise ValueError('Highway departure pattern must contain straight/city entries.')
    for vehicle in vehicles:
        if 'junction_departure' in vehicle:
            vehicle['junction_departure']['departure_pattern']=list(pattern)
    return vehicles, dict(lanes=configuration['speed_limit_lane_ids'],
                          maximum_speed_kmh=configuration['highway_speed_limit_kmh'])
