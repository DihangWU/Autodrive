"""只向 CoBridge 登记 CARLA 车辆身份；不连接 SUMO，不提交过时的位置。"""
import logging
import threading

from foundation.cobridge.control import ControlClient

RECONNECT_INTERVAL = 1.0
REQUEST_TIMEOUT = 1.0
JOIN_TIMEOUT = REQUEST_TIMEOUT * 2 + 1


class LincolnBridgeRegistration:
    def __init__(self, node, path, reporter):
        self.node, self.path, self.reporter = node, path, reporter
        self.sumo_id = None
        self.state = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run_registration_loop,
                                       name='lincoln-bridge-registration', daemon=True)
        self.thread.start()

    def vehicle_identity(self):
        return dict(actor_id=self.node.car.id, world_id=self.node.world.id,
                    tm_port=self.node.driver.traffic_manager_port)

    def publish_registration_state(self, status):
        self.sumo_id = status['sumo_id']
        current = (status['state'], status['detail'], self.sumo_id)
        if current != self.state:
            logging.info('桥登记：%s', status)
            print(f"[{self.node.uid}] SUMO {self.sumo_id}：{status['detail']}", flush=True)
            self.state = current
        self.reporter.set_state('running', f"CARLA {self.node.car.id} / SUMO {self.sumo_id}: {status['state']}")

    def run_registration_loop(self):
        client = None
        try:
            while not self.stop.is_set():
                try:
                    if client is None:
                        client = ControlClient(self.path, REQUEST_TIMEOUT)
                        status = client.command('lincoln.register', **self.vehicle_identity(),
                                                role_name=self.node.uid, sumo_id=self.sumo_id)
                    else:
                        status = client.command('lincoln.status', **self.vehicle_identity())
                    self.publish_registration_state(status)
                except Exception as error:
                    if client is not None:
                        client.close()
                        client = None
                    self.reporter.set_state('running', 'CARLA 车辆运行；等待 CoBridge 登记')
                    if self.state != ('disconnected', str(error)):
                        logging.info('等待 CoBridge：%s', error)
                        self.state = ('disconnected', str(error))
                self.stop.wait(RECONNECT_INTERVAL)
        finally:
            if client is not None:
                try:
                    client.command('lincoln.unregister', **self.vehicle_identity())
                except Exception:
                    logging.info('登记连接已断开，桥通过断线检查清理 SUMO 投影')
                finally:
                    client.close()

    def close(self):
        self.stop.set()
        self.thread.join(JOIN_TIMEOUT)
