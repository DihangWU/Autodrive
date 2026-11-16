"""沿连续匝道和上游连接向后排队，不压缩原有车距。"""


def queue_placement(configuration, index):
    position = configuration['front_vehicle_position']-index*configuration['initial_pitch_metres']
    for number,segment in enumerate(configuration['initial_queue_lanes']):
        if number:
            position += segment['length_metres']
        if position < 0.:
            continue
        lane = segment['lane_id']
        if position > segment['length_metres'] or (not lane.startswith(':') and position < 4.):
            raise ValueError('Ramp queue position is outside its usable lane.')
        route = segment.get('route_prefix',[])+configuration['route']
        entry = dict(route=route,lane_index=configuration['ramp_lane_index'],position=round(position,6))
        if lane.startswith(':'):
            entry['placement_lane_id'] = lane
        else:
            entry['lane_index'] = int(lane.rsplit('_',1)[1])
        return entry
    raise ValueError('Ramp queue extends beyond its configured upstream lanes.')
