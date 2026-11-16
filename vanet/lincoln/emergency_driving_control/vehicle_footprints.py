"""按双方车身朝向检查路径车辆包围盒，避免邻道斜车身的误阻挡。"""
import math


def overlaps_route_vehicle(sample, obstacle, front, half_width, margin=.4):
    forward=sample.forward
    norm=math.hypot(forward[0],forward[1])
    ego_forward=(forward[0]/norm,forward[1]/norm) if norm>1e-9 else (1.,0.)
    ego_side=(-ego_forward[1],ego_forward[0])
    other_forward=(math.cos(obstacle['yaw']),math.sin(obstacle['yaw']))
    other_side=(-other_forward[1],other_forward[0])
    delta=(obstacle['center'][0]-sample.center[0],obstacle['center'][1]-sample.center[1])
    def dot(a,b):return a[0]*b[0]+a[1]*b[1]
    # Separating-axis test on both rectangles. The old test aligned Lincoln's
    # inflated box with the other car, extending the corridor along its yaw.
    for axis in (ego_forward,ego_side,other_forward,other_side):
        ego_extent=front*abs(dot(ego_forward,axis))+half_width*abs(dot(ego_side,axis))
        other_extent=obstacle['length']*abs(dot(other_forward,axis))+obstacle['width']*abs(dot(other_side,axis))
        if abs(dot(delta,axis))>=ego_extent+other_extent+margin:return False
    return True
