"""按相对闭合速度判断车辆碰撞边界，不对远处前车提前跟车限速。"""
import bisect
import math
from .vehicle_footprints import overlaps_route_vehicle


def collision_speed_limit(controller,obstacles,target,speed):
    config=controller.collision_configuration
    deceleration=config['braking_deceleration_mps2']
    reaction=config['reaction_seconds'];margin=config['minimum_gap_metres']
    release=config['release_margin_metres']
    previous=getattr(controller,'collision_active_actors',set())
    active=set();nearest=None;minimum_brake=0.
    begin=max(0,bisect.bisect_left(controller.distances,controller.progress))
    candidates = (obstacles.collision_candidates(controller,speed,begin)
                  if hasattr(obstacles,'collision_candidates') else
                  ((obstacle,range(begin,len(controller.samples))) for obstacle in obstacles))
    for obstacle,indices in candidates:
        vehicle=obstacle.get('kind','vehicle.').startswith('vehicle.')
        latched=vehicle and obstacle['id'] in previous
        cosine,sine=math.cos(obstacle['yaw']),math.sin(obstacle['yaw'])
        body_reach=math.hypot(obstacle['length'],obstacle['width'])+math.hypot(controller.front,controller.half_width)+.4
        for index in indices:
            sample=controller.samples[index]
            lead_speed=sum(obstacle['velocity'][axis]*sample.forward[axis] for axis in range(3))
            closing=max(0.,speed-lead_speed)
            boundary=margin+closing*reaction+closing*closing/(2.*deceleration)
            gap=max(0.,controller.distances[index]-controller.progress)
            horizon=boundary+(release if latched else 0.) if vehicle else max(12.,speed*2.+speed*speed/5.)
            if gap>horizon+controller.front+2.*obstacle['length']+5.:break
            dx,dy,dz=(sample.center[axis]-obstacle['center'][axis] for axis in range(3))
            if abs(dz)>=obstacle['height']+1.:continue
            if vehicle:
                if abs(dx)>=body_reach or abs(dy)>=body_reach:continue
                if not overlaps_route_vehicle(sample,obstacle,controller.front,controller.half_width):continue
            elif (abs(dx*cosine+dy*sine)>=obstacle['length']+controller.front+.4 or
                  abs(-dx*sine+dy*cosine)>=obstacle['width']+controller.half_width+.4):continue
            required=brake=0.
            if vehicle:
                if gap>boundary+(release if latched else 0.):break
                active.add(obstacle['id'])
                safe_closing=max(0.,math.sqrt((deceleration*reaction)**2+2.*deceleration*max(0.,gap-margin))-deceleration*reaction)
                limit=max(0.,lead_speed+safe_closing)
                if latched:limit=min(limit,speed)
                space=max(.01,gap-margin-closing*reaction)
                required=closing*closing/(2.*space)
                brake=1. if gap<=margin else min(1.,required/deceleration)
                if limit<speed or gap<=margin:minimum_brake=max(minimum_brake,brake)
            else:limit=math.sqrt(5.*max(0.,gap-2.))
            candidate=dict(actor_id=obstacle['id'],gap_metres=gap,closing_speed_mps=closing,
                braking_boundary_metres=boundary,release_boundary_metres=boundary+release,
                required_deceleration_mps2=required,minimum_brake=brake,kind=obstacle.get('kind','vehicle.'),
                obstacle_center=obstacle['center'],obstacle_yaw_degrees=math.degrees(obstacle['yaw']),
                route_point=sample.center)
            if vehicle and (nearest is None or gap<nearest['gap_metres']):nearest=candidate
            if limit<target:
                target=limit
                if not vehicle:nearest=candidate
                controller.wait_reason='collision_guard' if vehicle else 'obstacle'
            break
    controller.collision_guard_state=nearest
    controller.collision_minimum_brake=minimum_brake
    controller.collision_active_actors=active
    return target
