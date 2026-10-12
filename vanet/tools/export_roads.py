"""Export every Town04 SUMO lane, including junction lanes, for Qtenv."""
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.paths import SUMO_CO_SIM_DIR, OMNET_DIR


def main():
    source = SUMO_CO_SIM_DIR / 'examples/net/Town04.net.xml'
    network = ET.parse(source).getroot()
    bounds = network.find('location').get('convBoundary').split(',')
    lines = ['BOUNDS ' + ' '.join(bounds)]
    lanes = network.findall('./edge/lane')
    for lane in lanes:
        points = [point.split(',')[:2] for point in lane.get('shape').split()]
        lines.append(lane.get('width', '3.2') + ' ' + ' '.join(v for p in points for v in p))
    target = OMNET_DIR / 'roads.txt'
    target.write_text('\n'.join(lines) + '\n')
    print(f'Exported all {len(lanes)} lanes from {source.name} to {target}')


if __name__ == '__main__':
    main()
