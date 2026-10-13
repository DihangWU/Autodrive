"""Optional per-step timing for the existing SUMO/CARLA/VANET bridge.

Set VANET_PROFILE_PATH to a JSONL file when starting the shared bridge.
Only aggregate windows are written, so profiling does not add per-step I/O.
"""
import json
from pathlib import Path
import statistics
import time


class PhaseProfile:
    def __init__(self, path, window=100):
        self.path = Path(path)
        self.window = window
        self.samples = []

    def record(self, timestamps, sumo_vehicles, carla_vehicles):
        values = tuple(1000 * (b-a) for a, b in zip(timestamps, timestamps[1:]))
        self.samples.append(values)
        if len(self.samples) < self.window:
            return
        names = ('sumo_step', 'sumo_to_carla', 'carla_step', 'carla_to_sumo', 'network_wait')
        result = dict(wall_time=time.time(), frames=len(self.samples),
                      sumo_vehicles=sumo_vehicles, carla_vehicles=carla_vehicles,
                      stages_ms={})
        for index, name in enumerate(names):
            durations = sorted(row[index] for row in self.samples)
            result['stages_ms'][name] = dict(mean=round(statistics.mean(durations), 3),
                                            p95=round(durations[int(.95*(len(durations)-1))], 3))
        result['stages_ms']['total'] = dict(mean=round(statistics.mean(sum(row) for row in self.samples), 3))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('a') as output:
            output.write(json.dumps(result, ensure_ascii=False) + '\n')
        self.samples.clear()
