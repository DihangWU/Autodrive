"""将实际规划路线绘制为贴地、半透明的车道宽绿色带和带内白色箭头。

只使用目标之前的 CARLA waypoint，不绘制 TM 的目标后前瞻缓冲。
保留高度和道路横坡；CARLA→ROS 的 Y 轴变换与现有车道标线一致。
"""
from dataclasses import dataclass
import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

SAMPLE_DISTANCE = 1.0
RIBBON_HEIGHT = 0.012
ARROW_HEIGHT = 0.007
RIBBON_COLOR = (0.05, 0.9, 0.2, 0.35)
ARROW_COLOR = (1., 1., 1., 0.95)
ARROW_INTERVAL = 12.0
ARROW_FIRST_DISTANCE = 5.0
FRAME_ID = 'map'
TM_LINE_WIDTH = 0.12
TM_LINE_HEIGHT = 0.12
TM_LINE_LIFETIME = 0.8


def tm_path_markers(location, waypoints):
    """红色直线仅连接 TM 实际公开的动作路点，不进行路线推测或平滑。"""
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.ns, marker.id = 'tm_action_path', 0
    marker.type = Marker.LINE_STRIP
    marker.pose.orientation.w = 1.
    marker.scale.x = TM_LINE_WIDTH
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .05, .05, .95
    marker.lifetime.nanosec = int(TM_LINE_LIFETIME * 1e9)
    if waypoints:
        for point in [location, *waypoints]:
            converted = Point(x=point.x, y=-point.y, z=point.z + TM_LINE_HEIGHT)
            if marker.points and sum((getattr(converted, axis)-getattr(marker.points[-1], axis))**2
                                     for axis in ('x', 'y', 'z')) < .01:
                continue
            marker.points.append(converted)
    marker.action = Marker.ADD if len(marker.points) >= 2 else Marker.DELETE
    return MarkerArray(markers=[marker])


@dataclass(frozen=True)
class RouteSample:
    center: tuple
    forward: tuple
    right: tuple
    width: float


def vector(value):
    return value.x, value.y, value.z


def normalize(value):
    length = math.sqrt(sum(v * v for v in value))
    return tuple(v / length for v in value) if length > 1e-8 else (1., 0., 0.)


def sample_waypoint(waypoint):
    transform = waypoint.transform
    return RouteSample(vector(transform.location), vector(transform.get_forward_vector()),
                       vector(transform.get_right_vector()), float(waypoint.lane_width))


def interpolate_segment(first, second, fraction):
    """跨道路/换道处采用连续切线曲线，避免把换道画成横向直角折线。"""
    a, b = first.transform, second.transform
    p, q = vector(a.location), vector(b.location)
    distance = a.location.distance(b.location)
    u, v = vector(a.get_forward_vector()), vector(b.get_forward_vector())
    t = fraction
    h = (2*t**3 - 3*t*t + 1, t**3 - 2*t*t + t, -2*t**3 + 3*t*t, t**3 - t*t)
    d = (6*t*t - 6*t, 3*t*t - 4*t + 1, -6*t*t + 6*t, 3*t*t - 2*t)
    center = tuple(h[0]*p[i] + h[1]*distance*u[i] + h[2]*q[i] + h[3]*distance*v[i] for i in range(3))
    forward = normalize(tuple(d[0]*p[i] + d[1]*distance*u[i] + d[2]*q[i] + d[3]*distance*v[i] for i in range(3)))
    up_a, up_b = vector(a.get_up_vector()), vector(b.get_up_vector())
    up = normalize(tuple(x*(1-t) + y*t for x, y in zip(up_a, up_b)))
    right = normalize((up[1]*forward[2] - up[2]*forward[1],
                       up[2]*forward[0] - up[0]*forward[2],
                       up[0]*forward[1] - up[1]*forward[0]))
    return RouteSample(center, forward, right, float(first.lane_width)*(1-t) + float(second.lane_width)*t)


def sample_route(waypoints, road_map=None):
    """同道路车道用原始 OpenDRIVE 曲线细分，交界处保留规划顺序。"""
    if not waypoints:
        return []
    samples = [sample_waypoint(waypoints[0])]
    for first, second in zip(waypoints, waypoints[1:]):
        distance = first.transform.location.distance(second.transform.location)
        if distance < .01:
            continue
        steps = max(1, math.ceil(distance / SAMPLE_DISTANCE))
        same_lane = first.road_id == second.road_id and first.lane_id == second.lane_id
        for index in range(1, steps + 1):
            fraction = index / steps
            waypoint = None
            if road_map is not None and same_lane:
                waypoint = road_map.get_waypoint_xodr(first.road_id, first.lane_id,
                                                      first.s + (second.s - first.s)*fraction)
            samples.append(sample_waypoint(waypoint) if waypoint is not None
                           else interpolate_segment(first, second, fraction))
    return samples


def surface_point(sample, forward=0., right=0., height=RIBBON_HEIGHT):
    x, y, z = (sample.center[i] + forward*sample.forward[i] + right*sample.right[i] for i in range(3))
    return Point(x=x, y=-y, z=z + height)


def triangle_marker(namespace, color):
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.ns, marker.id = namespace, 0
    marker.type, marker.action = Marker.TRIANGLE_LIST, Marker.ADD
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = marker.scale.z = 1.
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def add_triangle(marker, a, b, c):
    """统一朝上的正面，防止弯道处因背面剔除出现缺口。"""
    normal_z = (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
    marker.points.extend((a, b, c) if normal_z >= 0 else (a, c, b))
    # 每面颜色避免灯光调暗；透明度只由 marker.color.a 控制，防止相乘变成 alpha²。
    marker.colors.append(ColorRGBA(r=marker.color.r, g=marker.color.g, b=marker.color.b, a=1.))


def add_ribbon_segment(marker, first, second):
    al, ar = surface_point(first, right=-first.width/2), surface_point(first, right=first.width/2)
    bl, br = surface_point(second, right=-second.width/2), surface_point(second, right=second.width/2)
    add_triangle(marker, al, ar, bl)
    add_triangle(marker, ar, br, bl)


def add_arrow(marker, sample):
    """箭头头部与轴都在车道宽度内，沿地面绘制，无竖立箭杆。"""
    length = min(3., sample.width * .8)
    head_width, shaft_width = sample.width * .28, sample.width * .07
    height = RIBBON_HEIGHT + ARROW_HEIGHT
    tip = surface_point(sample, forward=length/2, height=height)
    left = surface_point(sample, right=-head_width, height=height)
    right = surface_point(sample, right=head_width, height=height)
    a = surface_point(sample, forward=-length/2, right=-shaft_width, height=height)
    b = surface_point(sample, forward=-length/2, right=shaft_width, height=height)
    c = surface_point(sample, right=-shaft_width, height=height)
    d = surface_point(sample, right=shaft_width, height=height)
    add_triangle(marker, tip, left, right)
    add_triangle(marker, a, b, c)
    add_triangle(marker, b, d, c)


def clear_route_markers():
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def navigation_route_markers(waypoints, road_map=None):
    samples = sample_route(waypoints, road_map)
    ribbon = triangle_marker('navigation_lane', RIBBON_COLOR)
    arrows = triangle_marker('navigation_arrows', ARROW_COLOR)
    distance, next_arrow = 0., ARROW_FIRST_DISTANCE
    total_length = sum(math.dist(a.center, b.center) for a, b in zip(samples, samples[1:]))
    for first, second in zip(samples, samples[1:]):
        add_ribbon_segment(ribbon, first, second)
        distance += math.dist(first.center, second.center)
        if distance >= next_arrow and second.width > .2 and distance + min(3., second.width*.8)/2 <= total_length:
            add_arrow(arrows, second)
            next_arrow += ARROW_INTERVAL
    # 使用固定两个 ID，覆盖上一条路线；DELETEALL 也可供后加入的 RViz 清除缓存。
    return MarkerArray(markers=[ribbon, arrows])
