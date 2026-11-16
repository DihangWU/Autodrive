"""危险时立即收油制动，风险解除后按仿真时间平顺恢复。"""


class GuardActuation:
    def __init__(self,config):
        self.config=config;self.recovering=False
        self.clear_seconds=self.ramp_seconds=self.throttle=self.brake=0.

    def apply(self,throttle,brake,minimum_brake,active,dt):
        requested=throttle
        if minimum_brake>0. or active:
            self.recovering=True;self.clear_seconds=self.ramp_seconds=0.
        elif self.recovering:self.clear_seconds+=dt
        if self.recovering:
            self.brake=max(brake,minimum_brake,max(0.,self.brake-self.config['brake_release_per_second']*dt))
            if self.brake>1e-9 or active or self.clear_seconds+1e-9<self.config['clearance_stable_seconds']:
                throttle=0.;self.ramp_seconds=0.
            else:
                self.ramp_seconds+=dt
                throttle=min(throttle,self.throttle+self.config['throttle_rise_per_second']*dt)
                if self.ramp_seconds*self.config['throttle_rise_per_second']>=1.-1e-9 and throttle>=requested-1e-9:
                    self.recovering=False
            brake=self.brake
        if brake>1e-9:throttle=0.
        self.throttle,self.brake=throttle,brake
        return throttle,brake
