"""判断车身是否已让出 Lincoln 通行宽度，不要求横移或车身转向结束。"""
import math


def leaves_passage(state, assessment, config):
    point=assessment.get('point')
    if point is None:return False
    dx=float(state['x'])-point['x'];dy=float(state['y'])-point['y']
    distance=math.hypot(dx,dy)
    if distance<1e-6:return False
    yaw=math.radians(float(state.get('heading',0.)))
    cosine,sine=math.cos(yaw),math.sin(yaw)
    # Project the actual oriented body onto the route's lateral direction.
    # A tilted vehicle may already leave enough room for Lincoln to pass.
    extent=abs(dx*cosine+dy*sine)/distance*float(state.get('length',5.))/2
    extent+=abs(-dx*sine+dy*cosine)/distance*float(state.get('width',1.85))/2
    return distance-extent>=config.get('corridor_half_width',1.25)
