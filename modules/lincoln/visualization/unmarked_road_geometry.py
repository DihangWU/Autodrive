"""无油漆标线的道路仍显示几何外边缘，避免在地图里整段消失。"""


def unmarked_road_edges(groups, painted_roads):
    for (road_id, _section_id, _lane_id), group in groups.items():
        if road_id in painted_roads:
            continue
        for a, b in zip(group, group[1:]):
            if b.s-a.s > .8 or a.transform.location.distance(b.transform.location) > 1.:
                continue
            if min(a.lane_width, b.lane_width) <= .01:
                continue
            for side in (-1, 1):
                neighbour_a = a.get_left_lane() if side < 0 else a.get_right_lane()
                neighbour_b = b.get_left_lane() if side < 0 else b.get_right_lane()
                # 相邻可驾驶车道之间不是道路外缘，不在无标线路上补画中心线。
                if any(neighbour is not None and str(neighbour.lane_type) == 'Driving'
                       for neighbour in (neighbour_a, neighbour_b)):
                    continue
                yield a, b, side
