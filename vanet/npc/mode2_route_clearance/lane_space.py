"""只检查实际占位及并入时是否会相撞，不使用固定车距或跟车时距。"""
import math


def available_space(connection, name, state, policy):
    vehicle=connection.vehicle
    lane=vehicle.getLaneID(name)
    distance=max(0.,(connection.lane.getWidth(lane)-state.get('width',1.85))/2)
    acceleration=policy.get('lateral_acceleration_mps2',8.)
    maximum=policy.get('lateral_speed_mps',3.5)
    acceleration_distance=maximum*maximum/(2*acceleration)
    contact_seconds=(math.sqrt(2*distance/acceleration) if distance<=acceleration_distance else
                     maximum/acceleration+(distance-acceleration_distance)/maximum)+.05
    speed=vehicle.getSpeed(name)
    for side,neighbors in (('front',vehicle.getLeftLeaders(name)),('rear',vehicle.getLeftFollowers(name))):
        for other,native_gap in neighbors:
            # TraCI has already subtracted the following vehicle's minGap.
            # Restore it to measure actual bumper-to-bumper space, not a
            # comfort reserve that would veto a physically available slot.
            following=name if side=='front' else other
            gap=native_gap+vehicle.getMinGap(following)
            if gap<.25:return False,dict(side=side,vehicle=other,reason='occupied',gap_metres=gap,native_gap_metres=native_gap)
            other_speed=vehicle.getSpeed(other)
            closing=max(0.,speed-other_speed if side=='front' else other_speed-speed)
            # Only relative closing distance until bodies enter the same lane
            # and the following car can brake to the leader's speed. Equal
            # speeds require no extra gap; there is no 8 m/19 m threshold.
            deceleration=max(.1,vehicle.getDecel(name if side=='front' else other))
            collision_space=closing*contact_seconds+closing*closing/(2*deceleration)
            if gap<collision_space+.25:
                return False,dict(side=side,vehicle=other,reason='collision',gap_metres=gap,
                                  native_gap_metres=native_gap,closing_speed_mps=closing,
                                  collision_space_metres=collision_space+.25)
    return True,None
