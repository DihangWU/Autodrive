"""Map metadata belongs to the bridge, not the TraCI transport."""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


# 文件命名约定（动态地图路径仍由传入的 SUMO 配置解析）。
NETWORK_SUFFIX = '.net.xml'
OPENDRIVE_SUFFIX = '.xodr'


def read_network(config_file):
    config_file = Path(config_file).resolve()
    value = ET.parse(config_file).getroot().find('input/net-file').get('value')
    network_file = (config_file.parent / value).resolve()
    location = ET.parse(network_file).getroot().find('location')
    name = network_file.name.removesuffix(NETWORK_SUFFIX)
    xodr = network_file.with_name(name + OPENDRIVE_SUFFIX)
    return dict(network_file=network_file, map=name, offset=[float(v) for v in location.get('netOffset').split(',')],
                xodr_sha256=hashlib.sha256(xodr.read_text().strip().encode()).hexdigest())
