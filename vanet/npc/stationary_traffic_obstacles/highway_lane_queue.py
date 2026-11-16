"""展开固定堵车车队；仅生成场景定义，不控制时钟或注册无线节点。"""
import json
import math

from vanet.project_paths import SCENARIO_STATIONARY_HIGHWAY_QUEUE_FILE


def generate_queue_vehicles(configuration, vehicle_length):
    gap = float(configuration['bumper_gap_metres'])
    duration = float(configuration['stop_duration_seconds'])
    pitch = float(vehicle_length) + gap
    if not all(math.isfinite(value) and value > 0 for value in (gap, duration, vehicle_length)):
        raise ValueError('Stationary queue dimensions and stop duration must be positive finite numbers.')
    vehicles = []
    identifiers = set()
    for segment in configuration['segments']:
        edge = segment['edge']
        end = float(segment['end_position'])
        for lane, start in segment['lane_start_positions'].items():
            lane_index, start = int(lane), float(start)
            if (not math.isfinite(start) or not math.isfinite(end)
                    or start < vehicle_length or end < start or lane_index not in (3, 4)):
                raise ValueError('Stationary highway queues require valid stations in driving lanes 3 and 4.')
            count = math.floor((end - start) / pitch) + 1
            for index in range(count):
                name = f"{configuration['id_prefix']}_{edge.lstrip('-')}_{lane_index}_{index + 1:03d}"
                if name in identifiers:
                    raise ValueError('Duplicate stationary queue vehicle: ' + name)
                identifiers.add(name)
                vehicles.append(dict(id=name, route=[edge], lane_index=lane_index,
                    position=round(start + index * pitch, 6), speed=0.,
                    role='stationary_obstacle', depart_offset=0., stationary=True,
                    network_enabled=False, color=list(configuration['color']),
                    stop_duration_seconds=duration))
    for connector in configuration.get('junction_connectors', []):
        lane = connector['internal_lane']
        internal_edge, internal_index = lane.rsplit('_', 1)
        start, end = 6.6, float(connector['length']) - 3.
        if not lane.startswith(':') or end < start or connector['from_lane'] not in (3, 4):
            raise ValueError('Invalid left-lane junction connector for stationary obstacles.')
        for index in range(math.floor((end - start) / pitch) + 1):
            name = f"{configuration['id_prefix']}_junction_{lane.lstrip(':')}_{index + 1:03d}"
            if name in identifiers:
                raise ValueError('Duplicate stationary queue vehicle: ' + name)
            identifiers.add(name)
            vehicles.append(dict(id=name, route=[connector['from_edge'], connector['to_edge']],
                lane_index=connector['from_lane'], position=round(start + index * pitch, 6),
                placement_lane_id=lane, stop_edge=internal_edge, stop_lane_index=int(internal_index),
                speed=0., role='stationary_obstacle', depart_offset=0., stationary=True,
                network_enabled=False, color=list(configuration['color']), stop_duration_seconds=duration))
    return vehicles


def load_stationary_highway_vehicles(vehicle_type):
    configuration = json.loads(SCENARIO_STATIONARY_HIGHWAY_QUEUE_FILE.read_text(encoding='utf-8'))
    return generate_queue_vehicles(configuration, float(vehicle_type['length']))
