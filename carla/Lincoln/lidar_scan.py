"""Collect native per-frame lidar fragments into a complete horizontal sweep."""
from types import SimpleNamespace

import numpy as np


class LidarScan:
    def __init__(self, rotation_frequency, output_rate):
        self.scan_period = 1. / rotation_frequency
        self.output_period = 1. / output_rate
        self.start = None
        self.fragments = []
        self.last_output = None

    def receive(self, measurement):
        if self.start is None:
            self.start = measurement.timestamp
        self.fragments.append(measurement)
        if measurement.timestamp - self.start + 1e-8 < self.scan_period:
            return None

        fragments, self.fragments = self.fragments, []
        self.start = measurement.timestamp
        if (self.last_output is not None and
                measurement.timestamp - self.last_output + 1e-8 < self.output_period):
            return None

        # Each fragment was measured at a different vehicle pose. Express all
        # hits in the last fragment's sensor frame, preserving world positions.
        inverse = np.asarray(measurement.transform.get_inverse_matrix())
        clouds = []
        for fragment in fragments:
            points = np.frombuffer(fragment.raw_data, dtype='<f4').reshape(-1, 4).copy()
            relative = inverse @ np.asarray(fragment.transform.get_matrix())
            points[:, :3] = points[:, :3] @ relative[:3, :3].T + relative[:3, 3]
            clouds.append(points)
        points = np.concatenate(clouds)
        self.last_output = measurement.timestamp
        return SimpleNamespace(timestamp=measurement.timestamp,
                               raw_data=points.astype('<f4', copy=False).tobytes())
