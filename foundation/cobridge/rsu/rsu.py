"""RSU lifecycle and presentation; never advances simulation time."""
import json
import math
import os
from pathlib import Path
import re
import uuid

from .carla_rsu import RsuBody, MAX_HEIGHT
from .rsu_marker import draw_marker
from .rsu_label import draw_label, label_text

# 路径与 RSU 显示/校验参数；实际位置继续读取专属 JSON 配置。
PROJECT_DIR = Path(__file__).resolve().parents[3]
IMAGE_FILE = PROJECT_DIR / 'foundation' / 'cobridge' / 'rsu' / 'assets' / 'rsu.png'
STATE_TEMP_SUFFIX = '.tmp'
DEFAULT_BEACON_RADIUS = 5.0
MAX_BEACON_RADIUS = 50.0
POI_COLOR = (255, 180, 0, 255)
POI_TYPE = 'rsu'
POI_LAYER = 100
POI_WIDTH = 10.0
POI_HEIGHT = 10.0
DRAW_LIFETIME_STEPS = 2
LABEL_MIN_MAST_HEIGHT = 2.0
LABEL_MAST_RADIUS_FACTOR = 1.6
LABEL_VERTICAL_GAP = 5.0


def read_configuration(path):
    config = json.loads(path.read_text())
    if config.get('coordinate_system', 'carla') != 'carla':
        raise ValueError('RSU 配置应使用 CARLA 世界坐标，单位为米。')
    rsus = config['rsus']
    if not rsus:
        raise ValueError('rsus 列表不能为空。')
    names = set()
    for rsu in rsus:
        name = rsu['id']
        if not re.fullmatch(r'[A-Za-z0-9_]+', name) or name in names:
            raise ValueError(f'RSU ID 无效或重复：{name}')
        names.add(name)
        for key in ('x', 'y', 'z', 'base_z'):
            rsu[key] = float(rsu[key])
            if not math.isfinite(rsu[key]):
                raise ValueError(f'{name}.{key} 必须为有限数值。')
        if not 0 < rsu['z'] - rsu['base_z'] <= MAX_HEIGHT:
            raise ValueError(f'{name} 塔身高度必须在 0～{MAX_HEIGHT:g} 米之间。')
        radius = float(rsu.get('beacon_radius', DEFAULT_BEACON_RADIUS))
        if not math.isfinite(radius) or not 0 < radius <= MAX_BEACON_RADIUS:
            raise ValueError(f'{name}.beacon_radius 必须在 0～{MAX_BEACON_RADIUS:g} 米之间。')
        rsu['beacon_radius'] = radius
    return config


class RsuBridge:
    def __init__(self, world, sumo, network, config, state_file):
        self.world, self.sumo = world, sumo
        self.network = network
        self.config = read_configuration(config)
        self.state_file = state_file
        self.bodies = []
        self.session = uuid.uuid4().hex
        self.state = None
        self.coverage = None

    def save(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_file.with_suffix(STATE_TEMP_SUFFIX)
        temporary.write_text(json.dumps(self.state, indent=2))
        temporary.replace(self.state_file)

    def recover(self):
        if not self.state_file.exists():
            return
        state = json.loads(self.state_file.read_text())
        if state['world_id'] == self.world.id:
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name', '').startswith(f"rsu_{state['session']}_"):
                    if not actor.destroy():
                        raise RuntimeError(f'无法回收 RSU actor {actor.id}')
        existing = set(self.sumo.poi.getIDList())
        for name in state['pois']:
            if name in existing:
                self.sumo.poi.remove(name, layer=POI_LAYER)
        self.state_file.unlink()

    def setup(self):
        if self.config['map'] != self.network['map']:
            raise RuntimeError('RSU 配置地图与当前地图不一致。')
        self.recover()
        self.state = dict(session=self.session, world_id=self.world.id, pois=[])
        self.save()
        offset_x, offset_y = self.network['offset']
        image = str(IMAGE_FILE)
        for rsu in self.config['rsus']:
            name = rsu['id']
            if name in self.sumo.poi.getIDList():
                raise RuntimeError(f'SUMO 中已存在 POI：{name}')
            # Persist intent before creation so partial failures remain recoverable.
            self.state['pois'].append(name)
            self.save()
            self.sumo.poi.add(name, rsu['x'] + offset_x, -rsu['y'] + offset_y,
                              POI_COLOR, poiType=POI_TYPE, layer=POI_LAYER,
                              imgFile=image, width=POI_WIDTH, height=POI_HEIGHT)
            self.sumo.poi.setParameter(name, 'name', name)
            body = RsuBody(self.session)
            self.bodies.append(body)
            body.spawn(self.world, rsu, rsu['base_z'])
        if os.environ.get('AUTODRIVE_VANET') == '1':
            from vanet.rsu.coverage_display.coverage_circle import CoverageCircle
            self.coverage = CoverageCircle(self.sumo, self.network)
            self.coverage.setup(self.config['rsus'])

    def after_step(self, elapsed, step_length):
        for rsu in self.config['rsus']:
            draw_marker(self.world, rsu, rsu['base_z'], lifetime=step_length * DRAW_LIFETIME_STEPS)
            label_z = rsu['z'] + max(LABEL_MIN_MAST_HEIGHT, rsu['beacon_radius'] * LABEL_MAST_RADIUS_FACTOR) + LABEL_VERTICAL_GAP
            draw_label(self.world, rsu['x'], rsu['y'], label_z, label_text(rsu['id']),
                       lifetime=step_length * DRAW_LIFETIME_STEPS, elapsed=elapsed)

    def close(self):
        errors = []
        if self.coverage is not None:
            try:
                self.coverage.close()
            except Exception as error:
                errors.append(str(error))
        for body in reversed(self.bodies):
            try:
                body.destroy()
            except Exception as error:
                errors.append(str(error))
        if self.state:
            try:
                existing = set(self.sumo.poi.getIDList())
                for name in self.state['pois']:
                    if name in existing:
                        self.sumo.poi.remove(name, layer=POI_LAYER)
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('RSU 清理未完成，保留恢复记录：' + '; '.join(errors))
        if self.state:
            self.state_file.unlink(missing_ok=True)
