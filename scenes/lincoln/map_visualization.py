"""ROS road geometry derived only from the active CARLA map.

OpenDRIVE supplies lane geometry and marking style, not Unreal road textures.
Where explicit dash lengths are absent, broken-line rendering is illustrative.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
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

    def marker(key, width, color, kind=Marker.LINE_LIST):
        if key not in markers:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id = 'carla_map', len(markers)
            m.type, m.action = kind, Marker.ADD
            m.pose.orientation.w = 1.
            m.scale.x, m.scale.y, m.scale.z = (1., 1., 1.) if kind == Marker.TRIANGLE_LIST else (width, 1., 1.)
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

    styles = {'Solid': ('Solid',), 'Broken': ('Broken',),
              'SolidSolid': ('Solid', 'Solid'), 'SolidBroken': ('Solid', 'Broken'),
              'BrokenSolid': ('Broken', 'Solid'), 'BrokenBroken': ('Broken', 'Broken')}
    surface = marker(('surface',), 1., (.16, .17, .18), Marker.TRIANGLE_LIST)
    for group in groups.values():
        group.sort(key=lambda w: w.s)
        for a, b in zip(group, group[1:]):
            if b.s - a.s > .8 or a.transform.location.distance(b.transform.location) > 1.:
                continue  # Never connect unrelated lane sections or branches.
            al, ar = boundary(a, -a.lane_width / 2, 0.), boundary(a, a.lane_width / 2, 0.)
            bl, br = boundary(b, -b.lane_width / 2, 0.), boundary(b, b.lane_width / 2, 0.)
            surface.points.extend((al, ar, bl, ar, br, bl))
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
                    marker((color_name, width), width, color).points.extend((p, q))
    print(f'RVIZ_MAP source={carla_map.name} lane_sections={len(groups)} '
          f'marking_segments={len(seen)}; broken dash spacing is illustrative', flush=True)
    return MarkerArray(markers=list(markers.values()))
