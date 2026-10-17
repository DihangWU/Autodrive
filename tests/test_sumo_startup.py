"""Readiness must observe the listener without taking a TraCI connection."""
import os
import select
import socket
import unittest

from foundation.sumo.init.sumo_init import process_identity, process_listening

LISTEN_HOST = '127.0.0.1'


class SumoStartupTests(unittest.TestCase):
    def test_owned_listener_is_detected_without_connecting(self):
        with socket.socket() as listener:
            listener.bind((LISTEN_HOST, 0))
            port = listener.getsockname()[1]
            self.assertFalse(process_listening(os.getpid(), port))
            listener.listen()
            self.assertTrue(process_listening(os.getpid(), port))
            self.assertEqual(select.select([listener], [], [], 0)[0], [])
        self.assertFalse(process_listening(os.getpid(), port))

    def test_identity_distinguishes_live_and_missing_process(self):
        self.assertIsNotNone(process_identity(os.getpid()))
        self.assertIsNone(process_identity(-1))


if __name__ == '__main__':
    unittest.main()
