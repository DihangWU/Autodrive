"""目标道后车短暂收速形成并入空位；不变道、不停车锁定、不转发无线消息。"""
import logging
import math

LOG=logging.getLogger(__name__)


class RearGapAssistance:
    def __init__(self):
        self.requested=set()

    def request(self, connection, name, now):
        vehicle=connection.vehicle;speed=vehicle.getSpeed(name)
        for other,gap in vehicle.getLeftFollowers(name):
            if other in self.requested or other.startswith(('lincoln','background_')):continue
            other_speed=vehicle.getSpeed(other)
            deceleration=max(.1,vehicle.getDecel(other))
            closing=max(0.,other_speed-speed)
            # Reserve only native bumper clearance, not a normal time headway.
            if gap>=.25+closing*.55+closing*closing/(2*deceleration):continue
            target=max(0.,speed-math.sqrt(2*deceleration*max(0.,.25-gap)))
            if other_speed<=target:continue
            duration=max(.1,(other_speed-target)/deceleration)
            vehicle.slowDown(other,target,duration)
            self.requested.add(other)
            # slowDown's finite SUMO timeline releases automatically. Never
            # leave a setSpeed command on the supporting third-lane vehicle.
            LOG.info('MODE2_REAR_GAP_ASSIST vehicle=%s follower=%s sim_time=%.3f target_mps=%.3f duration=%.3f native_gap=%.3f',
                     name,other,now,target,duration,gap)
