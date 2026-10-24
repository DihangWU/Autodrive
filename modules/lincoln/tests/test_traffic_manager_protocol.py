"""同步驾驶的官方 TM RPC：覆盖拆包、异常回复及端口隔离。"""
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import Mock, patch

from foundation.cobridge.lincoln.traffic_manager_clock import build_tm_request, receive_tm_reply
from modules.lincoln.driving.traffic_manager_port import reserve_traffic_manager_port


class TrafficManagerProtocolTests(unittest.TestCase):
    def test_tick_request_targets_official_method_without_parameters(self):
        self.assertEqual(build_tm_request('synchronous_tick'),
                         bytes([0x94, 0, 1, 0xb0]) + b'synchronous_tick' + bytes([0x90]))

    def test_partial_tcp_reply_is_reassembled(self):
        stream = Mock()
        stream.recv.side_effect = [b'\x94', b'\x01\x01', b'\xc0', b'\xc3']
        self.assertTrue(receive_tm_reply(stream))

    def test_error_reply_is_not_reported_as_success(self):
        stream = Mock()
        stream.recv.return_value = b'\x94\x01\x01\xa1x'
        with self.assertRaises(RuntimeError):
            receive_tm_reply(stream)

    def test_closed_connection_does_not_wait_forever(self):
        stream = Mock()
        stream.recv.return_value = b''
        with self.assertRaises(ConnectionError):
            receive_tm_reply(stream)

    def test_busy_port_is_skipped(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch('modules.lincoln.driving.traffic_manager_port.TM_PORT_LOCK_FILE', Path(folder) / 'tm.lock'):
                with socket.socket() as occupied:
                    occupied.bind(('0.0.0.0', 0))
                    used = occupied.getsockname()[1]
                    with reserve_traffic_manager_port(used) as selected:
                        self.assertGreater(selected, used)


if __name__ == '__main__':
    unittest.main()
