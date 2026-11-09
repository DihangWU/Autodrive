"""按场景清单逐个加载 NPC；每辆车使用一个独立配置文件。"""
import json
from pathlib import Path
from vanet.project_paths import SCENARIO_NPC_DEFINITIONS_DIRECTORY


def load_vehicle_definitions(filenames):
    vehicles=[]
    identifiers=set()
    for filename in filenames:
        if not isinstance(filename,str) or Path(filename).name!=filename or not filename.endswith('.json'):
            raise ValueError('NPC configuration must be a JSON filename in the NPC definitions directory.')
        configuration_file=SCENARIO_NPC_DEFINITIONS_DIRECTORY/filename
        vehicle=json.loads(configuration_file.read_text(encoding='utf-8'))
        if not isinstance(vehicle,dict) or not isinstance(vehicle.get('id'),str) or not vehicle['id']:
            raise ValueError('NPC configuration requires a vehicle id: '+filename)
        if vehicle['id'] in identifiers:raise ValueError('Duplicate NPC id: '+vehicle['id'])
        identifiers.add(vehicle['id']);vehicles.append(vehicle)
    return vehicles
