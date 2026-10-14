"""Shared scene and radio configuration for all VANET entry points."""
import json
import math
import re
from pathlib import Path

from cobridge.common.paths import CONFIG


def load_config(path=CONFIG):
    path = Path(path)
    cfg = json.loads(path.read_text())
    rsu_file = cfg.pop('rsu_config', None)
    if rsu_file is not None:
        if not isinstance(rsu_file, str) or not rsu_file.strip():
            raise ValueError(f'{path}: rsu_config must be a configuration file path')
        if 'rsus' in cfg or 'rsu' in cfg:
            raise ValueError(f'{path}: use rsu_config or inline RSUs, not both')
        rsu_path = (path.parent / rsu_file).resolve()
        rsu_cfg = json.loads(rsu_path.read_text())
        if not isinstance(rsu_cfg, dict) or rsu_cfg.get('map') != cfg.get('map'):
            raise ValueError(f'{rsu_path}: RSU map must match {cfg.get("map")}')
        cfg['rsus'] = rsu_cfg.get('rsus')
    if 'rsu' in cfg and 'rsus' in cfg:
        raise ValueError('Use rsus only; do not define both rsu and rsus')
    if 'rsus' not in cfg and 'rsu' in cfg:
        cfg['rsus'] = [dict(cfg.pop('rsu'), id='rsu0')]
    if not isinstance(cfg.get('rsus'), list):
        raise ValueError('rsus must be a list')
    def number(value, name):
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f'{path}: {name} must be a finite number')
    radio = cfg.get('communication')
    if not isinstance(radio, dict):
        raise ValueError('communication must define vehicle_range and rsu_range in meters')
    for name in ('vehicle_range', 'rsu_range'):
        number(radio.get(name), 'communication.' + name)
        if radio[name] <= 0:
            raise ValueError('communication.' + name + ' must be positive')
    overrides = radio.setdefault('range_overrides', {})
    if not isinstance(overrides, dict):
        raise ValueError('communication.range_overrides must map node names to meters')
    for name, radius in overrides.items():
        if not isinstance(name, str) or not re.fullmatch(r'(?:lincoln|npc|carla|rsu)[A-Za-z0-9_]*', name):
            raise ValueError('Range override must use an exact vehicle/RSU node name: ' + str(name))
        number(radius, 'communication.range_overrides.' + name)
        if radius <= 0:
            raise ValueError('Range override must be positive: ' + name)
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


def radio_arguments(cfg):
    """The JSON is the sole source of radio defaults and per-node overrides."""
    radio = cfg['communication']
    overrides = radio['range_overrides']
    maximum = max(radio['vehicle_range'], radio['rsu_range'], *overrides.values())
    # Veins' neighbor prefilter uses 3D distance; hard ranges use XY distance.
    # Cover the entire configured 100m altitude span as well as antenna offsets.
    interference = math.hypot(maximum, 100) + 5
    return [f'--*.vehicleRange={radio["vehicle_range"]}m',
            f'--*.rsuRange={radio["rsu_range"]}m',
            f'--*.connectionManager.maxInterfDist={interference}m',
            *[f'--*.{name}.communicationRange={radius}m' for name, radius in overrides.items()]]
