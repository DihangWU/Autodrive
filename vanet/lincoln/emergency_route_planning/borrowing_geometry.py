"""不依赖 CARLA 的借道轨迹几何及预测冲突检查。"""
import math
from dataclasses import replace


def smooth_fraction(value):
    t = max(0., min(1., value))
    return t*t*t*(10.-15.*t+6.*t*t)


def blend_borrowing_segment(samples, targets, start, finish, transition=10.):
    """按实际行驶方向重建切线；对向车道的法定 forward 不用于追踪。"""
    changed = list(samples)
    distances = [0.]
    for a, b in zip(samples, samples[1:]):
        distances.append(distances[-1]+math.dist(a.center, b.center))
    length = distances[finish]-distances[start]
    transition = min(transition, length*.35)
    for i in range(start, finish+1):
        elapsed = distances[i]-distances[start]
        weight = min(smooth_fraction(elapsed/transition), smooth_fraction((length-elapsed)/transition))
        center = tuple(samples[i].center[j]*(1-weight)+targets[i-start][j]*weight for j in range(3))
        changed[i] = replace(samples[i], center=center)
    for i in range(max(0,start-1), min(len(changed)-1,finish+1)):
        delta = tuple(changed[i+1].center[j]-changed[i].center[j] for j in range(3))
        norm = math.sqrt(sum(v*v for v in delta))
        if norm > 1e-8:
            forward = tuple(v/norm for v in delta)
            right = (-forward[1], forward[0], 0.)
            changed[i] = replace(changed[i], forward=forward, right=right)
    return changed


def trajectory_clear(samples, obstacles, speed, ego_half_width, ego_front, margin=.5):
    """预测各轨迹点到达时刻，使用车辆朝向包围盒并区分道路高度。"""
    distance = 0.
    previous = samples[0].center if samples else (0.,0.,0.)
    for sample in samples:
        distance += math.dist(previous, sample.center)
        previous = sample.center
        arrival = distance/max(2., speed)
        for obstacle in obstacles:
            center = tuple(obstacle['center'][j] + obstacle['velocity'][j]*arrival for j in range(3))
            dx, dy, dz = (sample.center[j]-center[j] for j in range(3))
            if abs(dz) > obstacle['height']+1.:
                continue
            angle = obstacle['yaw']
            along = dx*math.cos(angle)+dy*math.sin(angle)
            across = -dx*math.sin(angle)+dy*math.cos(angle)
            if (abs(along) < obstacle['length']+ego_front+margin
                    and abs(across) < obstacle['width']+ego_half_width+margin):
                return False
    return True
