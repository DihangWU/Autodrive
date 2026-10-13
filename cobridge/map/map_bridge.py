"""Export the active CARLA road map and display it in SUMO GUI."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import carla

BRIDGE_DIR = Path(__file__).resolve().parents[1]


def sumo_binary(name):
    home = Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0'))
    candidate = home / 'bin' / name
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return str(candidate)
    found = shutil.which(name)
    if found:
        return found
    raise RuntimeError(f'找不到 {name}，请设置 SUMO_HOME。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('--export-only', action='store_true', help='只导出路网，不打开 GUI')
    args = parser.parse_args()

    netconvert = sumo_binary('netconvert')
    gui = None if args.export_only else sumo_binary('sumo-gui')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    town = client.get_world().get_map()
    name = town.name.rsplit('/', 1)[-1]
    folder = BRIDGE_DIR / 'runtime' / name
    folder.mkdir(parents=True, exist_ok=True)
    xodr = folder / f'{name}.xodr'
    network = folder / f'{name}.net.xml'
    config = folder / f'{name}.sumocfg'
    log = folder / 'netconvert.log'

    xodr.write_text(town.to_opendrive(), encoding='utf-8')
    print(f'CARLA 当前地图：{town.name}', flush=True)
    print('正在转换 SUMO 路网……', flush=True)
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run([
            netconvert, '--opendrive-files', str(xodr),
            '--output-file', str(network),
            '--opendrive.lane-shapes', 'true',
            '--opendrive.internal-shapes', 'true',
        ], stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'路网转换失败，详情见：{log}')

    root = ET.Element('configuration')
    inputs = ET.SubElement(root, 'input')
    ET.SubElement(inputs, 'net-file', value=network.name)
    ET.ElementTree(root).write(config, encoding='utf-8', xml_declaration=True)
    print(f'SUMO 路网：{network}', flush=True)
    print(f'转换日志：{log}', flush=True)
    if gui:
        print('打开 SUMO GUI；关闭窗口即可退出。', flush=True)
        try:
            return subprocess.call([gui, '-c', str(config)])
        except KeyboardInterrupt:
            return 130
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as error:
        print(f'错误：{error}', file=sys.stderr)
        sys.exit(1)
