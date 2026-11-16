"""生成几何不变的实验路网副本，仅修正 1061 的双流拉链汇入。"""
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from vanet.project_paths import (SUMO_TOWN04_CONFIGURATION_FILE,SUMO_TOWN04_NETWORK_FILE,
                                 SUMO_TOWN04_OPENDRIVE_FILE,SCENARIO_ZIPPER_NETWORK_DIRECTORY,
                                 SCENARIO_ZIPPER_RULES_RELATIVE_FILE)


def prepare_zipper_configuration(directory=None):
    directory = Path(directory) if directory is not None else SCENARIO_ZIPPER_NETWORK_DIRECTORY
    directory.mkdir(parents=True,exist_ok=True)
    tree = ET.parse(SUMO_TOWN04_NETWORK_FILE)
    root = tree.getroot()
    junction = root.find("junction[@id='1061']")
    if junction is None:
        raise ValueError('Town04 ramp junction 1061 is missing.')
    lanes = junction.attrib['intLanes'].split()
    ramp = lanes.index(':1061_0_1')
    main = lanes.index(':1061_2_1')
    requests = {int(request.get('index')):request for request in junction.findall('request')}
    if len(requests) != len(lanes):
        raise ValueError('Unexpected request layout at Town04 ramp junction 1061.')
    # The ramp joins only the rightmost driving lane. The generated map also
    # conflicts it with all inner highway lanes, causing indefinite starvation.
    for index,request in requests.items():
        for attribute in ('foes','response'):
            bits = int(request.get(attribute),2)
            if index == ramp:
                bits = 1 << main
            elif index == main:
                bits |= 1 << ramp
            else:
                bits &= ~(1 << ramp)
            request.set(attribute,format(bits,f'0{len(lanes)}b'))
    junction.set('type','zipper')
    changed = []
    for connection in root.findall('connection'):
        if connection.get('via') in (':1061_0_1',':1061_2_1'):
            connection.set('state','Z')
            connection.set('visibility','100')
            changed.append(connection.get('via'))
    if set(changed) != {':1061_0_1',':1061_2_1'} or len(changed) != 2:
        raise ValueError('Expected one ramp and one main-road connection for zipper merging.')
    network = directory / SUMO_TOWN04_NETWORK_FILE.name
    tree.write(network,encoding='utf-8',xml_declaration=True)
    # Keep map-name/fingerprint discovery compatible with the existing bridge.
    shutil.copyfile(SUMO_TOWN04_OPENDRIVE_FILE,directory/SUMO_TOWN04_OPENDRIVE_FILE.name)
    config = ET.parse(SUMO_TOWN04_CONFIGURATION_FILE)
    config.getroot().find('input/net-file').set('value',network.name)
    configuration = directory / SUMO_TOWN04_CONFIGURATION_FILE.name
    config.write(configuration,encoding='utf-8',xml_declaration=True)
    (directory/SCENARIO_ZIPPER_RULES_RELATIVE_FILE).write_text(json.dumps(dict(junction='1061',
        main_lane='-46_1',ramp_lane='31_1',target_lane='-47_1',native_link_state='Z',
        source_network=str(SUMO_TOWN04_NETWORK_FILE),geometry_changed=False),indent=2)+'\n')
    return configuration
