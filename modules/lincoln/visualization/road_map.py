"""从当前 CARLA 地图生成贴地窄标线带，不绘制实心路面。

OpenDRIVE supplies lane geometry and marking style, not Unreal road textures.
Where explicit dash lengths are absent, broken-line rendering is illustrative.
"""
from collections import defaultdict
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def road_markers(carla_map):
    groups = defaultdict(list)
    for waypoint in carla_map.generate_waypoints(.5):
        groups[(waypoint.road_id, waypoint.section_id, waypoint.lane_id)].append(waypoint)
    markers = {}
    seen = set()
    colors = {'White': (1., 1., 1.), 'Standard': (1., 1., 1.),
              'Yellow': (1., .85, .05), 'Blue': (.1, .3, 1.),
              'Green': (.1, 1., .1), 'Red': (1., .1, .1)}

    def marker(key, color):
        if key not in markers:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id = 'carla_map', len(markers)
            m.type, m.action = Marker.TRIANGLE_LIST, Marker.ADD
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = 1.
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.
            markers[key] = m
        return markers[key]

    def boundary(w, offset, height=.03):
        transform = w.transform
        loc, right = transform.location, transform.get_right_vector()
        # CARLA x-forward/y-right -> ROS x-forward/y-left, retaining elevation/roll.
        return Point(x=loc.x + offset * right.x,
                     y=-(loc.y + offset * right.y),
                     z=loc.z + offset * right.z + height)

    def marking_strip(a, b, offset_a, offset_b, width):
        """用道路横向确定标线宽度，不让相机朝向改变标线形状。"""
        al, ar = boundary(a, offset_a - width / 2), boundary(a, offset_a + width / 2)
        bl, br = boundary(b, offset_b - width / 2), boundary(b, offset_b + width / 2)
        points = (al, ar, bl, ar, br, bl)
        # 正负车道的朝向不同，统一三角形正面朝上，避免背面剔除和发黑。
        normal_z = (ar.x - al.x) * (bl.y - al.y) - (ar.y - al.y) * (bl.x - al.x)
        if normal_z < 0:
            points = (al, bl, ar, ar, bl, br)
        return points

    styles = {'Solid': ('Solid',), 'Broken': ('Broken',),
              'SolidSolid': ('Solid', 'Solid'), 'SolidBroken': ('Solid', 'Broken'),
              'BrokenSolid': ('Broken', 'Solid'), 'BrokenBroken': ('Broken', 'Broken')}
    for group in groups.values():
        group.sort(key=lambda w: w.s)
        for a, b in zip(group, group[1:]):
            if b.s - a.s > .8 or a.transform.location.distance(b.transform.location) > 1.:
                continue  # Never connect unrelated lane sections or branches.
            for side, marking in ((-1, a.left_lane_marking), (1, a.right_lane_marking)):
                kind = str(marking.type)
                if kind not in styles:
                    continue  # NONE/curb/grass are not painted lane markings.
                width = float(marking.width)
                if width <= 0:
                    continue
                color_name = str(marking.color)
                color = colors.get(color_name)
                if color is None:
                    continue
                lines = styles[kind]
                for i, style in enumerate(lines):
                    # API omits dash pattern on Town04: indicative 3m/3m only.
                    if style == 'Broken' and ((a.s + b.s) / 2) % 6 >= 3:
                        continue
                    extra = (i - (len(lines) - 1) / 2) * (width * 2)
                    p = boundary(a, side * a.lane_width / 2 + extra)
                    q = boundary(b, side * b.lane_width / 2 + extra)
                    endpoints = tuple(sorted((tuple(round(v, 3) for v in (p.x, p.y, p.z)),
                                              tuple(round(v, 3) for v in (q.x, q.y, q.z)))))
                    identity = (endpoints, color_name, round(width, 4))
                    if identity in seen:
                        continue
                    seen.add(identity)
                    strip = marker((color_name, width), color)
                    strip.points.extend(marking_strip(
                        a, b, side * a.lane_width / 2 + extra,
                        side * b.lane_width / 2 + extra, width))
                    # 每三角形指定颜色，使 RViz 不用场景灯光调暗标线。
                    strip.colors.extend((strip.color, strip.color))
    print(f'RVIZ_MAP source={carla_map.name} lane_sections={len(groups)} '
          f'marking_segments={len(seen)}; broken dash spacing is illustrative', flush=True)
    return MarkerArray(markers=list(markers.values()))
