"""帧在 IPC 交换期间完成时，通知不能丢失或等待下一次心跳。"""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from vanet.lincoln.v2x_data_exchange import bridge_exchange


class FrameExchangeTests(unittest.TestCase):
    def test_notification_during_request_is_consumed_before_heartbeat_wait(self):
        exchange=bridge_exchange.LincolnBridgeExchange.__new__(bridge_exchange.LincolnBridgeExchange)
        exchange.stopped=threading.Event()
        exchange.wakeup=threading.Event()
        exchange.node=SimpleNamespace(args=SimpleNamespace(bridge_socket='unused-test-socket'),
                                      car=SimpleNamespace(id=7),world=SimpleNamespace(id=3))
        exchange.service=SimpleNamespace(lock=threading.RLock(),ack_frame=10,simulation_time=0.,
            task=SimpleNamespace(task_id='trip',route_version=1,state='running',command_ack_id='start'),
            snapshot=Mock(return_value=dict(state='running')))
        client=Mock();requests=[];wake_states=[]

        def command(action,**fields):
            requests.append(fields)
            if len(requests)==1:
                exchange.service.ack_frame=11
                exchange.notify_frame()
            else:
                exchange.stopped.set()
            return dict(simulation_time=.05)

        def wait(timeout):
            wake_states.append(exchange.wakeup.is_set())
            return exchange.wakeup.is_set()

        client.command.side_effect=command
        with patch.object(bridge_exchange,'ControlClient',return_value=client),patch.object(exchange.wakeup,'wait',side_effect=wait):
            exchange.run()
        self.assertEqual([request['ack_frame'] for request in requests],[10,11])
        self.assertTrue(wake_states[0])
        self.assertEqual(exchange.service.snapshot.call_args_list[0].kwargs,dict(include_route=True))
        self.assertEqual(exchange.service.snapshot.call_args_list[1].kwargs,dict(include_route=False))
        client.close.assert_called_once_with()
