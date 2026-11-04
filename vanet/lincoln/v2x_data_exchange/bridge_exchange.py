"""高频控制状态交换；无线消息仍由 CoBridge / OMNeT++ 实际传输。"""
import logging
import threading
from foundation.cobridge.control import ControlClient

LOG=logging.getLogger('vanet.lincoln.exchange')


class LincolnBridgeExchange:
    def __init__(self,node,service):
        self.node,self.service=node,service
        self.stopped=threading.Event()
        self.wakeup=threading.Event()
        self.thread=threading.Thread(target=self.run,name='vanet-lincoln-exchange',daemon=True)
        self.thread.start()

    def run(self):
        client=None
        last_version=None
        last_error=None
        try:
            while not self.stopped.is_set():
                self.wakeup.clear()
                try:
                    if client is None:
                        client=ControlClient(self.node.args.bridge_socket,1.)
                        last_version=None
                    with self.service.lock:
                        version=(self.service.task.task_id,self.service.task.route_version,self.service.task.state)
                        report=self.service.snapshot(include_route=version!=last_version)
                        ack_frame=self.service.ack_frame
                        command_ack_id=self.service.task.command_ack_id
                    result=client.command('vanet.lincoln.exchange',actor_id=self.node.car.id,world_id=self.node.world.id,
                                          ack_frame=ack_frame,status=report,
                                          command_ack_id=command_ack_id)
                    last_version=version
                    self.service.simulation_time=float(result.get('simulation_time',self.service.simulation_time))
                    if 'communication_paths' in result:
                        self.service.set_communication_paths(result['communication_paths'],result.get('rsu_connected',False))
                    if result.get('command'):self.service.receive_command(result['command'])
                    if result.get('deliveries'):self.service.receive_deliveries(result['deliveries'])
                    last_error=None
                except Exception as error:
                    if str(error)!=last_error:
                        LOG.warning('BRIDGE_EXCHANGE_WAIT %s',error)
                        last_error=str(error)
                    if client:client.close()
                    client=None
                self.wakeup.wait(.05 if client else .2)
        finally:
            if client:client.close()

    def notify_frame(self):
        self.wakeup.set()

    def close(self):
        self.stopped.set()
        self.wakeup.set()
        self.thread.join(2.)
