"""加载红框起点的两列联网背景车，配置与避让策略分别管理。"""
import copy
import json

from vanet.project_paths import SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE


def load_staggered_traffic():
    configuration = json.loads(SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text(encoding='utf-8'))
    vehicles = [dict(copy.deepcopy(configuration['vehicle_defaults']), **entry)
                for entry in configuration['vehicles']]
    if len(vehicles) != 20 or any(sum(v['lane_index']==lane for v in vehicles)!=10 for lane in (1,2)):
        raise ValueError('Highway background traffic requires ten vehicles in each of SUMO lanes 1 and 2.')
    if len({v['id'] for v in vehicles}) != 20:
        raise ValueError('Highway background vehicle ids must be unique.')
    return vehicles, dict(lanes=configuration['speed_limit_lane_ids'],
                          maximum_speed_kmh=configuration['highway_speed_limit_kmh'])
