"""加载独立汇入车队；不改变原有四条主路车道的场景定义。"""
import json

from vanet.project_paths import SCENARIO_HIGHWAY_RAMP_MERGING_FILE
from vanet.npc.highway_ramp_merging.ramp_queue_placement import queue_placement


def load_merging_vehicles():
    configuration = json.loads(SCENARIO_HIGHWAY_RAMP_MERGING_FILE.read_text(encoding='utf-8'))
    offsets = configuration['release_offsets_seconds']
    if len(offsets) != configuration['count'] or offsets != sorted(offsets) or offsets[0] < 0:
        raise ValueError('Ramp release offsets must be ordered and match the configured vehicle count.')
    if configuration.get('zipper_meter',{}).get('main_vehicles_per_ramp') != [1]:
        raise ValueError('The rolling zipper controller uses one main vehicle per ramp vehicle.')
    vehicles = []
    for index in range(configuration['count']):
        placement = queue_placement(configuration,index)
        vehicles.append(dict(id=f"{configuration['id_prefix']}_{index+1:02d}",
            **placement,
            speed=0.,depart_offset=0.,role='ramp_merging',network_enabled=True,yielding_enabled=False,
            maximum_speed_kmh=configuration['maximum_speed_kmh'],speed_factor=1.,speed_mode=31,
            # Keep SUMO's sublane centering after each requested lane change,
            # while disabling autonomous strategic/speed-gain lane changes.
            lane_change_mode=1536,hold_lane_before_start=True,hold_for_merge_release=True,
            lane_change_parameters=dict(lcKeepRight=0.0,lcSpeedGain=0.0),
            merge_release_delay_seconds=offsets[index],
            merge_queue_index=index,
            color=list(configuration['color'])))
    return vehicles,configuration
