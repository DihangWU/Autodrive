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
    for name in ('base_z', 'beacon_radius'):
        value = cfg['rsu'].get(name, 0)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f'{CONFIG}: rsu.{name} must be a finite number')
    if cfg['rsu'].get('beacon_radius', 1) <= 0:
        raise ValueError('rsu.beacon_radius must be positive')
    if 'base_z' in cfg['rsu'] and cfg['rsu']['z'] <= cfg['rsu']['base_z']:
        raise ValueError('rsu.z must be above rsu.base_z')
    return cfg


def sumo_rsu(cfg):
    p, net = cfg['rsu'], cfg['network']
    return p['x'] + net['offset_x'], -p['y'] + net['offset_y'], p['z']
