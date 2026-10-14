"""Delayed display interpolation; never extrapolate a physical vehicle pose."""

from collections import deque
import math
import threading
import time


def _rotation_between(first, second, fraction):
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = max(-1., min(1., dot))
    if dot > .9995:
        result = tuple(a + fraction * (b - a) for a, b in zip(first, second))
        length = math.sqrt(sum(value * value for value in result))
        return tuple(value / length for value in result)
    angle = math.acos(dot)
    scale = math.sin(angle)
    a = math.sin((1. - fraction) * angle) / scale
    b = math.sin(fraction * angle) / scale
    return tuple(a * x + b * y for x, y in zip(first, second))


class DisplayPoseBuffer:
    def __init__(self, delay_seconds):
        if delay_seconds <= 0:
            raise ValueError('Display pose delay must be positive')
        self.delay = delay_seconds
        self.samples = deque(maxlen=32)
        self.last_display_time = None
        self.last_wall_time = None
        self.playback_rate = None
        self.lock = threading.Lock()

    def push(self, simulation_time, position, rotation, wall_time=None):
        wall_time = time.monotonic() if wall_time is None else wall_time
        with self.lock:
            if self.samples and simulation_time < self.samples[-1][0]:
                self.samples.clear()
                self.last_display_time = None
                self.last_wall_time = None
                self.playback_rate = None
            if self.samples and simulation_time == self.samples[-1][0]:
                return
            self.samples.append((simulation_time, wall_time, tuple(position), tuple(rotation)))

    def sample(self, wall_time=None):
        wall_time = time.monotonic() if wall_time is None else wall_time
        with self.lock:
            if not self.samples:
                return None
            latest = self.samples[-1]
            earlier = self.samples[0]
            elapsed = latest[1] - earlier[1]
            rate = (latest[0] - earlier[0]) / elapsed if elapsed > 0 else 1.
            desired = latest[0] + max(0., wall_time - latest[1]) * rate - self.delay
            if self.last_wall_time is None:
                target = desired
                self.playback_rate = rate
            else:
                wall_step = max(0., wall_time - self.last_wall_time)
                # Advance a continuous playback clock. Re-anchoring on every
                # receipt would turn network/scheduling jitter into speed jumps.
                blend = 1. - math.exp(-wall_step / .25)
                self.playback_rate += blend * (rate - self.playback_rate)
                advance = wall_step * self.playback_rate
                predicted = self.last_display_time + advance
                correction = (desired - predicted) * blend
                correction = max(-advance * .1, min(advance * .1, correction))
                target = predicted + correction
            target = max(self.samples[0][0], min(latest[0], target))
            self.last_wall_time = wall_time
            if self.last_display_time is not None and target <= self.last_display_time:
                return None
            self.last_display_time = target
            first = self.samples[0]
            for second in list(self.samples)[1:]:
                if target <= second[0]:
                    fraction = (target - first[0]) / (second[0] - first[0])
                    position = tuple(a + fraction * (b - a)
                                     for a, b in zip(first[2], second[2]))
                    rotation = _rotation_between(first[3], second[3], fraction)
                    return target, position, rotation
                first = second
            return latest[0], latest[2], latest[3]
