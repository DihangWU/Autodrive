"""一次性生成不联网的随机车流；运行中仅使用 SUMO 原生简单跟车。"""
import json
import logging
import math
import random
import xml.etree.ElementTree as ET

from vanet.project_paths import (SCENARIO_NON_NETWORK_HIGHWAY_TRAFFIC_FILE,
                                 SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE,
                                 SUMO_TOWN04_NETWORK_FILE)

LOG = logging.getLogger(__name__)


def generate_background_vehicles(configuration, lanes, vehicle_length, reference_pitch):
    density = float(configuration['density_relative_to_network_lanes'])
    if not all(math.isfinite(v) and v > 0 for v in (density,reference_pitch,vehicle_length)):
        raise ValueError('Background traffic density, reference pitch and vehicle length must be positive.')
    pitch = reference_pitch / density
    rng = random.Random(configuration['random_seed'])
    route = configuration['route']
    vehicles = []
    for corridor in configuration['corridors']:
        lane_index = corridor['sumo_lane_index']
        if lane_index not in (3,4):
            raise ValueError('Non-network background traffic must stay in the two leftmost driving lanes.')
        geometry = []
        total = 0.
        for lane_id in corridor['lanes']:
            lane = lanes[lane_id]
            points = [tuple(map(float,point.split(',')[:2])) for point in lane['shape'].split()]
            length = sum(math.dist(first,second) for first,second in zip(points,points[1:]))
            declared = float(lane['length'])
            if length <= 0 or declared <= 0:
                raise ValueError('Background traffic requires nonempty lane geometry: '+lane_id)
            geometry.append((lane_id,total,length,declared))
            total += length
        first = geometry[0]
        start = float(configuration['start_front_positions'][str(lane_index)]) * first[2] / first[3]
        last = geometry[-1]
        end = total - float(configuration['end_front_margin_metres']) * last[2] / last[3]
        count = max(2,round((end-start)/pitch)+1)
        weights = [rng.uniform(*configuration['spacing_random_weight_range']) for _ in range(count-1)]
        scale = (end-start) / sum(weights)
        if min(weights)*scale < vehicle_length+3.:
            raise ValueError('Background density leaves insufficient initial following space.')
        station = start
        for index in range(count):
            lane_id,origin,length,declared = next(part for part in geometry if station < part[1]+part[2]+1e-7)
            position = (station-origin) * declared / length
            # SUMO normal insertion needs a full car inside its starting lane.
            # A small boundary adjustment avoids clipped or deferred departures.
            position = max(vehicle_length+.2,position)
            edge = lane_id.rsplit('_',1)[0]
            preceding = edge if not edge.startswith(':') else next(
                part[0].rsplit('_',1)[0] for part in reversed(geometry)
                if part[1] < origin and not part[0].startswith(':'))
            entry = dict(id=f"{configuration['id_prefix']}_lane{corridor['left_to_right_lane']}_{index+1:03d}",
                route=route[route.index(preceding):],lane_index=lane_index,position=round(position,6),
                speed=0.,depart_offset=0.,role='non_network_background',stationary=False,
                network_enabled=False,yielding_enabled=False,color=list(configuration['color']),
                maximum_speed_kmh=round(rng.uniform(*configuration['maximum_speed_kmh_range']),3),
                driver_imperfection=round(rng.uniform(*configuration['driver_imperfection_range']),3),
                speed_factor=1.,speed_mode=31,lane_change_mode=512,hold_lane_before_start=True,
                initial_placement=dict(lane_id=lane_id,left_to_right_lane=corridor['left_to_right_lane'],
                    corridor_distance_metres=round(origin+position*length/declared-start,6),
                    corridor_length_metres=round(end-start,6)))
            if edge.startswith(':'):
                entry['placement_lane_id'] = lane_id
            vehicles.append(entry)
            if index < len(weights):
                station += weights[index]*scale
    return vehicles


def load_background_vehicles(vehicle_type):
    configuration = json.loads(SCENARIO_NON_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text(encoding='utf-8'))
    reference = json.loads(SCENARIO_NETWORK_HIGHWAY_TRAFFIC_FILE.read_text(encoding='utf-8'))
    selected = {name for corridor in configuration['corridors'] for name in corridor['lanes']}
    root = ET.parse(SUMO_TOWN04_NETWORK_FILE).getroot()
    lanes = {lane.attrib['id']:lane.attrib for edge in root.findall('edge')
             for lane in edge.findall('lane') if lane.attrib['id'] in selected}
    vehicles = generate_background_vehicles(configuration,lanes,float(vehicle_type['length']),
                                             float(reference['placement_reference']['same_lane_pitch_metres']))
    LOG.info('NON_NETWORK_TRAFFIC count=%s density_ratio=%s speed_range_kmh=%s seed=%s straight_only=True',
             len(vehicles),configuration['density_relative_to_network_lanes'],
             configuration['maximum_speed_kmh_range'],configuration['random_seed'])
    return vehicles
