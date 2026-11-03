"""读取实验中的 Lincoln 参数；所有路径由 project_paths 提供。"""
import json
import math
from vanet.project_paths import EXPERIMENT_DEFAULT_CONFIGURATION_FILE
from vanet.lincoln.v2x_data_exchange.clearance_advice import DEFAULT_CLEARANCE_ADVICE

DEFAULT_BORROWING = dict(evaluation_interval=.5,lookahead_metres=65.,minimum_length_metres=28.,
                        transition_metres=12.,prediction_speed_mps=10.)


def load_control_configuration():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    maximum=float(configuration.get('maximum_speed_kmh',120.))
    borrowing=dict(DEFAULT_BORROWING)
    borrowing.update(configuration.get('lincoln_borrowing',{}))
    for name,value in [('maximum_speed_kmh',maximum),*borrowing.items()]:
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid positive Lincoln control parameter: '+name)
    return maximum,borrowing


def load_clearance_configuration():
    configuration=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    advice=dict(DEFAULT_CLEARANCE_ADVICE)
    advice.update(configuration.get('lincoln_clearance_advice',{}))
    for name,value in advice.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid positive Lincoln clearance parameter: '+name)
    return advice,configuration.get('rsu',{}).get('id','rsu0')
