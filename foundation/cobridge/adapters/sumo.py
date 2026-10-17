"""Claim a running SUMO runtime's clock; detach never closes the simulator."""
from foundation.sumo.client import SumoClient
from foundation.cobridge.network import read_network

TIME_TOLERANCE = 1e-6


class SumoAdapter:
    def __init__(self, path, timeout, step_length, fingerprint=None, on_operation=None):
        self.client = SumoClient(path, timeout)
        self.claimed = False
        self.on_operation = on_operation
        try:
            status = self.client.status()
            self.network = read_network(status['config'])
            if fingerprint is not None and self.network['xodr_sha256'] != fingerprint:
                raise RuntimeError('SUMO 与已接入 CARLA 的地图不一致。')
            self.elapsed = self.client.claim_clock(step_length)['time']
            self.claimed = True
            self.connection = self.client.connection
            self.step_length = step_length
        except BaseException:
            self.client.close()
            raise

    def step(self):
        self.connection.simulationStep()
        elapsed = self.connection.simulation.getTime()
        if abs(elapsed - self.elapsed - self.step_length) > TIME_TOLERANCE:
            raise RuntimeError('SUMO 时间增量不一致。')
        self.elapsed = elapsed
        self.relay_events()

    def relay_events(self):
        if self.on_operation is None:
            return
        pending = self.client.poll_events()
        for event in pending['events']:
            self.on_operation(event['message'])
        if pending['dropped']:
            self.on_operation(f"有 {pending['dropped']} 条操作通知因缓冲区满而丢弃。")
        if pending['events'] or pending['dropped']:
            self.client.acknowledge_events([event['id'] for event in pending['events']],
                                           pending['dropped'])

    def close(self):
        try:
            if self.claimed:
                self.client.release_clock()
        finally:
            self.claimed = False
            self.client.close()
