"""只在固定场景拥有期间覆盖本侧高速限速，清理后恢复原值。"""
import logging

LOG = logging.getLogger(__name__)


class HighwaySpeedLimits:
    def __init__(self, connection, configuration=None):
        self.connection = connection
        self.configuration = configuration or {}
        self.saved = {}

    def apply(self):
        if self.saved or not self.configuration:
            return
        maximum = float(self.configuration['maximum_speed_kmh']) / 3.6
        try:
            for lane in self.configuration['lanes']:
                self.saved[lane] = self.connection.lane.getMaxSpeed(lane)
                self.connection.lane.setMaxSpeed(lane, maximum)
                LOG.info('Highway speed limit lane=%s original_mps=%s maximum_kmh=%s',
                         lane,self.saved[lane],self.configuration['maximum_speed_kmh'])
        except Exception:
            self.restore()
            raise

    def restore(self):
        for lane, speed in list(self.saved.items()):
            self.connection.lane.setMaxSpeed(lane, speed)
            del self.saved[lane]
