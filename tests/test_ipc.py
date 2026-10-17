"""A non-JSON RPC result must not terminate a simulator runtime."""
import json
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import MagicMock

from foundation.ipc import JsonServer


class IpcTests(unittest.TestCase):
    def test_unserializable_result_returns_error_and_server_continues(self):
        with tempfile.TemporaryDirectory() as directory:
            server = JsonServer(Path(directory) / 'test.sock')
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            disconnected = MagicMock()
            try:
                peer.connect(str(server.path))
                peer.sendall(b'{"action":"call"}\n')
                server.poll(lambda *args:object(),disconnected)
                self.assertFalse(json.loads(peer.recv(4096))['ok'])
                peer.sendall(b'{"action":"status"}\n')
                server.poll(lambda *args:dict(alive=True),disconnected)
                self.assertEqual(json.loads(peer.recv(4096)),dict(ok=True,result=dict(alive=True)))
            finally:
                peer.close()
                server.close()


if __name__ == '__main__':
    unittest.main()
