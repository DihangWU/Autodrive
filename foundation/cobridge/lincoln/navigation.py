"""SUMO 导航目标校验与坐标转换；任务经既有登记连接交给 Lincoln。"""
from functools import lru_cache
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

MAX_SNAP_DISTANCE = 12.0
OVERLAP_HEIGHT = 3.0
OVERLAP_DISTANCE = 0.5
NAVIGATION_STATES = {'planning', 'navigating', 'arrived', 'cancelled', 'unreachable', 'error'}


@lru_cache(maxsize=2)
def read_driving_lanes(filename, modified):
    """只缓存当前路网的可行驶车道，保留原始三维高度。"""
    lanes = []
    for edge in ET.parse(filename).getroot().findall('edge'):
        for lane in edge.findall('lane'):
            allowed, forbidden = lane.get('allow', '').split(), lane.get('disallow', '').split()
            if (allowed and not {'passenger', 'all'} & set(allowed)) or {'passenger', 'all'} & set(forbidden):
                continue
            shape = [tuple(map(float, point.split(','))) for point in lane.get('shape', '').split()]
            lanes.append((lane.get('id'), [(p[0], p[1], p[2] if len(p) > 2 else 0.) for p in shape]))
    return lanes


def closest_point(x, y, shape):
    best = None
    for first, second in zip(shape, shape[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = dx * dx + dy * dy
        fraction = max(0., min(1., ((x - first[0]) * dx + (y - first[1]) * dy) / length)) if length else 0.
        point = tuple(a + fraction * (b - a) for a, b in zip(first, second))
        candidate = (math.hypot(x - point[0], y - point[1]), point)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best


def resolve_destination(network, x, y):
    """匹配可行驶车道；二维重叠且高度不同时拒绝猜测目标层。"""
    if isinstance(x, bool) or isinstance(y, bool) or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y)):
        raise ValueError('Invalid destination coordinates.')
    path = Path(network['network_file'])
    matches = []
    for lane_id, shape in read_driving_lanes(str(path), path.stat().st_mtime_ns):
        match = closest_point(x, y, shape)
        if match is not None:
            matches.append((match[0], lane_id, match[1]))
    matches.sort()
    if not matches or matches[0][0] > MAX_SNAP_DISTANCE:
        raise ValueError('Click a drivable road; this point is too far from a lane.')
    distance, lane_id, point = matches[0]
    if any(item[0] <= distance + OVERLAP_DISTANCE and abs(item[2][2] - point[2]) > OVERLAP_HEIGHT for item in matches[1:]):
        raise ValueError('Roads overlap at different heights; select a non-overlapping section.')
    ox, oy = network['offset']
    return dict(x=point[0] - ox, y=oy - point[1], z=point[2], sumo_lane=lane_id)


class LincolnNavigation:
    def __init__(self, registry):
        self.registry = registry

    def find_vehicle(self, command, carla, sumo):
        if carla is None or sumo is None:
            raise ValueError('Connect CARLA, SUMO and CoBridge first.')
        if Path(command.get('network_file', '')).resolve() != Path(sumo.network['network_file']).resolve():
            raise ValueError('This SUMO window uses a different network than CoBridge.')
        record = next((r for r in self.registry.records.values() if r['sumo_id'] == command.get('sumo_id')), None)
        if record is None or command.get('owner_key') != f"lincoln:{record['world_id']}:{record['actor_id']}":
            raise ValueError('Vehicle identity changed or the vehicle exited; select the Lincoln again.')
        if record['world_id'] != carla.world.id or record['state'] != 'synced':
            raise ValueError('Vehicle is not synchronized with the current map.')
        if not record.get('navigation_supported'):
            raise ValueError('Restart this Lincoln to enable navigation support.')
        return record

    def handle(self, command, carla, sumo):
        record = self.find_vehicle(command, carla, sumo)
        if command['action'] == 'lincoln.navigate':
            target = resolve_destination(sumo.network, command.get('x'), command.get('y'))
            task = dict(id=uuid.uuid4().hex, target=target, issued_at=time.monotonic())
            record['navigation_request'] = task
            record['navigation'] = dict(id=task['id'], state='planning', detail='Waiting for Lincoln route planning')
        return record.get('navigation', dict(state='idle', detail='No destination'))

    @staticmethod
    def receive_status(record, status):
        if not isinstance(status, dict) or not record.get('navigation_request'):
            return
        if status.get('id') != record['navigation_request']['id'] or status.get('state') not in NAVIGATION_STATES:
            return
        record['navigation'] = dict(id=status['id'], state=status['state'], detail=str(status.get('detail', ''))[:300])
