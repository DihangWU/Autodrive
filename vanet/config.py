"""One RSU definition shared by the OMNeT++ launcher and scene markers."""
import json
import math
from pathlib import Path

CONFIG = Path(__file__).with_name('town04.json')


def load_config():
    cfg = json.loads(CONFIG.read_text())
    for section, names in (('rsu', ('x', 'y', 'z')), ('network', ('offset_x', 'offset_y'))):
        for name in names:
            value = cfg[section][name]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{CONFIG}: {section}.{name} must be a finite number')
    return cfg


def sumo_rsu(cfg):
    p, net = cfg['rsu'], cfg['network']
    return p['x'] + net['offset_x'], -p['y'] + net['offset_y'], p['z']
