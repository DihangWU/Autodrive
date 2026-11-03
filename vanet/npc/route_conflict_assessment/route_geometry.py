"""无传感器的路线冲突几何；坐标为 CARLA 世界坐标，heading 为度。"""
import math


def point(value):
    return float(value['x']), float(value['y']), float(value.get('z', 0.))


def project(position, route):
    """返回到有限路线最近投影的距离、里程、切向与高度差。"""
    if len(route) < 2:
        return None
    x, y, z = point(position)
    best, station = None, 0.
    for a, b in zip(route, route[1:]):
        ax, ay, az = point(a); bx, by, bz = point(b)
        dx, dy, dz = bx-ax, by-ay, bz-az
        length = math.hypot(dx, dy)
        if length < .001:
            continue
        fraction = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/(length*length)))
        distance = math.hypot(x-ax-fraction*dx, y-ay-fraction*dy)
        candidate = dict(distance=distance, station=station+fraction*length,
                         direction=(dx/length, dy/length), height=abs(z-az-fraction*dz),
                         point=dict(x=ax+fraction*dx,y=ay+fraction*dy,z=az+fraction*dz))
        if best is None or (candidate['height'] > 2., distance) < (best['height'] > 2., best['distance']):
            best = candidate
        station += length
    return best


def conflict(state, task, config):
    """只使用传入的本车/已收到信标及实际收到的任务做预测。"""
    route = task.get('route', [])
    nearest = project(state, route)
    if nearest is None or nearest['height'] > config.get('height_tolerance', 2.):
        return dict(relevant=False, reason='different_height_or_empty_route')
    ego = task.get('position', task.get('emergency_position', route[0]))
    progress = project(ego, route)
    advance = progress['station'] if progress else 0.
    yaw = math.radians(float(state.get('heading', 0.)))
    dot = math.cos(yaw)*nearest['direction'][0]+math.sin(yaw)*nearest['direction'][1]
    kind = 'same_direction' if dot > .5 else ('opposing' if dot < -.5 else 'crossing')
    width = config.get('corridor_half_width', 1.25)+float(state.get('width', 1.85))/2
    relevant = (nearest['distance'] <= width and
                nearest['station'] >= advance-config.get('passed_clearance', 8.))
    eta = max(0., nearest['station']-advance)/max(2., float(task.get('speed', 8.)))
    prediction = None
    if not relevant and kind == 'crossing':
        horizon = min(config.get('prediction_seconds', 8.), eta+2.)
        travel = max(0., float(state.get('speed', 0.)))*horizon
        for i in range(1, 17):
            sample = dict(x=float(state['x'])+math.cos(yaw)*travel*i/16,
                          y=float(state['y'])+math.sin(yaw)*travel*i/16,
                          z=float(state.get('z', 0.)))
            at = project(sample, route)
            if at and at['distance'] <= width and at['height'] <= config.get('height_tolerance', 2.) and at['station'] >= advance:
                relevant, prediction = True, at
                break
    return dict(relevant=relevant, kind=kind, distance=nearest['distance'],
                station=nearest['station'], emergency_station=advance,
                inside_corridor=nearest['distance'] <= width,
                prediction=prediction, eta=eta, point=nearest['point'])
