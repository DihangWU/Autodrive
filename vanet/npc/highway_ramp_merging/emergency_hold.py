"""收到协同请求后在匝道内平稳停车，避免与拉链配速同时写速度。"""
import logging
from vanet.npc.highway_background_traffic.junction_route_split import value

LOG=logging.getLogger(__name__)


class RampEmergencyHold:
    def __init__(self, connection, meter):
        self.connection,self.meter=connection,meter
        self.reset()

    def reset(self):
        self.policy={}
        self.requested=set();self.braking=set();self.stopped=set();self.committed=set();self.resume=set()

    def request(self, name, policy=None):
        if policy is not None:self.policy=dict(policy)
        self.requested.add(name)

    def release(self, name):
        self.requested.discard(name)
        self.resume.add(name)

    def status(self, name):
        if name in self.stopped:return 'stopped'
        if name in self.committed:return 'committed'
        return 'braking' if name in self.braking else 'pending'

    def update(self, states, constants, sim_time):
        resuming=bool(self.resume & self.braking)
        for name in self.resume:
            self.braking.discard(name);self.stopped.discard(name);self.committed.discard(name)
            self.meter.commanded.pop(name,None)
            if name in states:self.connection.vehicle.setSpeed(name,-1.)
            LOG.info('MODE2_RAMP_RESUME sim_time=%.3f vehicle=%s',sim_time,name)
        self.resume.clear()
        if resuming:
            self.meter.rebooking=True
            for name in list(self.meter.reservations):
                state=states.get(name)
                if isinstance(state,dict) and self.meter.ramp_distance(value(state,constants.VAR_LANE_ID),value(state,constants.VAR_LANEPOSITION)) is not None:
                    self.meter.reservations.pop(name)
                    self.meter.commanded.pop(name,None)
        held=set()
        for name in self.requested:
            state=states.get(name)
            if not isinstance(state,dict):continue
            distance=self.meter.ramp_distance(value(state,constants.VAR_LANE_ID),value(state,constants.VAR_LANEPOSITION))
            if distance is None or name in self.committed:continue
            speed=value(state,constants.VAR_SPEED) or 0.
            if name not in self.braking:
                deceleration=self.policy.get('ramp_braking_mps2',4.)
                margin=self.policy.get('ramp_stop_margin_metres',6.)
                if distance<speed*speed/(2*deceleration)+margin:
                    self.committed.add(name)
                    LOG.info('MODE2_RAMP_CLEAR_MERGE sim_time=%.3f vehicle=%s remaining=%.3f speed=%.3f',sim_time,name,distance,speed)
                    continue
                self.connection.vehicle.slowDown(name,0.,max(.5,speed/deceleration))
                self.braking.add(name)
                LOG.info('MODE2_RAMP_BRAKE sim_time=%.3f vehicle=%s remaining=%.3f speed=%.3f',sim_time,name,distance,speed)
            held.add(name)
            if speed<.3 and name not in self.stopped:
                self.connection.vehicle.setSpeed(name,0.)
                self.stopped.add(name)
        return held
