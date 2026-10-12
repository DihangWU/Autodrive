"""RSU list shared by the OMNeT++ launcher and scene markers."""
import json
import math
import re
from pathlib import Path

CONFIG = Path(__file__).with_name('town04.json')


def load_config(path=CONFIG):
    cfg = json.loads(Path(path).read_text())
    if 'rsu' in cfg and 'rsus' in cfg:
        raise ValueError('Use rsus only; do not define both rsu and rsus')
    if 'rsus' not in cfg and 'rsu' in cfg:
        cfg['rsus'] = [dict(cfg.pop('rsu'), id='rsu0')]
    if not isinstance(cfg.get('rsus'), list):
        raise ValueError('rsus must be a list')
    def number(value, name):
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f'{path}: {name} must be a finite number')
    for name in ('offset_x', 'offset_y'):
        number(cfg['network'][name], 'network.' + name)
    ids = set()
    for p in cfg['rsus']:
        if not isinstance(p, dict) or not isinstance(p.get('id'), str) or not re.fullmatch(r'rsu[A-Za-z0-9_]*', p['id']):
            raise ValueError('Each RSU needs an id such as rsu0 or rsu_north (letters, digits, underscore)')
        if p['id'] in ids:
            raise ValueError('Duplicate RSU id: ' + p['id'])
        ids.add(p['id'])
        for name in ('x', 'y', 'z'):
            number(p.get(name), p['id'] + '.' + name)
        for name in ('base_z', 'beacon_radius'):
            number(p.get(name, 0), p['id'] + '.' + name)
        if p.get('beacon_radius', 1) <= 0 or p['z'] < 0 or p['z'] > 100:
            raise ValueError(p['id'] + ': beacon_radius must be positive; z must be within 0..100m')
        if 'base_z' in p and p['z'] <= p['base_z']:
            raise ValueError(p['id'] + ': z must be above base_z')
    return cfg


def sumo_rsu(cfg, p):
    net = cfg['network']
    return p['x'] + net['offset_x'], -p['y'] + net['offset_y'], p['z']


def rsu_definitions(cfg):
    return '\n'.join(p['id'] + ' ' + ' '.join(map(str, sumo_rsu(cfg, p))) for p in cfg['rsus'])
