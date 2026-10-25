"""读取官方 SUMO 车型定义，选择当前 CARLA 也具备的四轮轿车。"""
from pathlib import Path
import fnmatch
import xml.etree.ElementTree as ET

PROJECT_DIR = Path(__file__).resolve().parents[3]
OFFICIAL_SUMO_DIR = PROJECT_DIR / 'foundation/carla/carla-official/Co-Simulation/Sumo'
VEHICLE_TYPES_FILE = OFFICIAL_SUMO_DIR / 'examples/carlavtypes.rou.xml'
CAR_CLASSES = frozenset({'passenger', 'evehicle'})


def read_vehicle_types(blueprints, pattern):
    """只选两端都支持的车型，避免将自行车或缺失模型作为汽车加入。"""
    available = {bp.id: bp for bp in blueprints
                 if bp.has_attribute('number_of_wheels')
                 and int(bp.get_attribute('number_of_wheels')) == 4}
    selected = {}
    for element in ET.parse(VEHICLE_TYPES_FILE).getroot().findall('vType'):
        spec = dict(element.attrib)
        name = spec['id']
        if (name in available and fnmatch.fnmatchcase(name, pattern)
                and spec.get('vClass') in CAR_CLASSES):
            blueprint = available[name]
            spec['colors'] = (list(blueprint.get_attribute('color').recommended_values)
                              if blueprint.has_attribute('color') else [])
            selected[name] = spec
    if not selected:
        raise ValueError(f'官方车型配置与当前 CARLA 中没有匹配的四轮轿车：{pattern}')
    return selected


def install_vehicle_type(connection, spec, existing):
    """按需将官方 XML 定义加入已运行的 SUMO，不重新加载地图。"""
    name = spec['id']
    if name in existing:
        return
    types = connection.vehicletype
    types.copy('DEFAULT_VEHTYPE', name)
    types.setVehicleClass(name, spec['vClass'])
    types.setShapeClass(name, spec.get('guiShape', 'passenger/sedan'))
    types.setLength(name, float(spec['length']))
    types.setWidth(name, float(spec['width']))
    types.setHeight(name, float(spec['height']))
    existing.add(name)
