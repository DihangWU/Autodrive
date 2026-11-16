"""读取实验中的 Lincoln 参数；所有路径由 project_paths 提供。"""
import json
import math
from vanet.project_paths import EXPERIMENT_DEFAULT_CONFIGURATION_FILE
from vanet.lincoln.v2x_data_exchange.clearance_advice import DEFAULT_CLEARANCE_ADVICE

DEFAULT_BORROWING = dict(evaluation_interval=.5,lookahead_metres=65.,minimum_length_metres=28.,
                        transition_metres=12.,prediction_speed_mps=10.)
DEFAULT_COLLISION_GUARD=dict(braking_deceleration_mps2=4.,reaction_seconds=.35,minimum_gap_metres=3.,
    release_margin_metres=3.,clearance_stable_seconds=.25,brake_release_per_second=2.,throttle_rise_per_second=1.5)


def load_collision_configuration():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    guard=dict(DEFAULT_COLLISION_GUARD)
    guard.update(configuration.get('lincoln_collision_guard',{}))
    for name,value in guard.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid positive Lincoln collision parameter: '+name)
    return guard


def load_control_configuration():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    raw_maximum=configuration.get('maximum_speed_kmh',120.)
    maximum=None if raw_maximum is None else float(raw_maximum)
    borrowing=dict(DEFAULT_BORROWING)
    borrowing.update(configuration.get('lincoln_borrowing',{}))
    values=list(borrowing.items())
    if maximum is not None:values.append(('maximum_speed_kmh',maximum))
    if isinstance(raw_maximum,bool):raise ValueError('maximum_speed_kmh must be numeric or null.')
    for name,value in values:
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid positive Lincoln control parameter: '+name)
    return maximum,borrowing


def load_highest_right_of_way():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    enabled=configuration.get('lincoln_highest_right_of_way',False)
    if not isinstance(enabled,bool):raise ValueError('lincoln_highest_right_of_way must be boolean.')
    return enabled


def load_clearance_configuration():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    advice=dict(DEFAULT_CLEARANCE_ADVICE)
    advice.update(configuration.get('lincoln_clearance_advice',{}))
    for name,value in advice.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid positive Lincoln clearance parameter: '+name)
    return advice,configuration.get('rsu',{}).get('id','rsu0')
