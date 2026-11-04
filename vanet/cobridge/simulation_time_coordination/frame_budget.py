"""一帧的现实时间预算包含所有屏障等待，仿真步长保持不变。"""
import time


class FrameBudget:
    def __init__(self, step_length, clock=time.monotonic):
        self.step_length = step_length
        self.clock = clock
        self.started = None
        self.stages = {}
        self.samples = []

    def begin(self):
        if self.started is None:
            self.started = self.clock()

    def add(self, stage, elapsed):
        self.stages[stage] = self.stages.get(stage, 0.) + elapsed

    def remaining(self):
        return max(0., self.step_length - (self.clock() - self.started))

    def suspend(self):
        self.started = None
        self.stages = {}

    def finish(self):
        self.stages['frame_wall'] = self.clock() - self.started
        self.samples.append(self.stages)
        self.started = None
        self.stages = {}
        if len(self.samples) < 20:
            return None
        samples, self.samples = self.samples, []
        keys = set().union(*(sample.keys() for sample in samples))
        return dict(frames=len(samples), mean_ms={key:round(1000 * sum(s.get(key, 0.) for s in samples) / len(samples), 3) for key in sorted(keys)},
                    frame_p95_ms=round(1000 * sorted(s['frame_wall'] for s in samples)[18], 3))
