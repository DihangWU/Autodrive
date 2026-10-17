"""Module notifications are generic, optional, and reconnect after observer restart."""
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock

from foundation.ipc import JsonServer
from foundation.module_status import ModuleRegistry, ModuleReporter

TEST_TIMEOUT = 5.0
POLL_INTERVAL = .005


def wait_until(check):
    deadline = time.monotonic() + TEST_TIMEOUT
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(POLL_INTERVAL)
    raise AssertionError('module status timed out')


class ModuleStatusTests(unittest.TestCase):
    def test_arbitrary_module_state_and_disconnect_are_display_only(self):
        display = MagicMock()
        registry = ModuleRegistry(display)
        owner = object()
        update = dict(name='FutureModule',state='running',detail='ready')
        registry.update(update,owner)
        registry.update(update,owner)
        display.assert_called_once_with('模块 FutureModule：运行中（ready）')
        self.assertEqual(registry.snapshot(),[update])
        registry.disconnected(owner)
        self.assertEqual(registry.snapshot(),[])
        self.assertIn('状态连接已断开',display.call_args.args[0])

    def test_invalid_notification_does_not_change_registry(self):
        display = MagicMock()
        registry = ModuleRegistry(display)
        with self.assertRaises(ValueError):
            registry.update(dict(name='bad\nname',state='running'),object())
        self.assertEqual(registry.snapshot(),[])
        display.assert_not_called()

    def test_optional_reporter_late_observer_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'observer.sock'
            reporter = ModuleReporter('FutureModule',path)
            reporter.set_state('running','ready')
            servers = []
            def start_observer():
                registry = ModuleRegistry(MagicMock())
                server = JsonServer(path)
                stopped = threading.Event()
                def pump():
                    while not stopped.is_set():
                        server.poll(lambda command,owner:registry.update(command,owner),registry.disconnected)
                        stopped.wait(POLL_INTERVAL)
                thread = threading.Thread(target=pump,daemon=True)
                thread.start()
                servers.append((server,stopped,thread))
                return registry
            def stop_observer():
                server,stopped,thread = servers.pop()
                stopped.set();thread.join(TEST_TIMEOUT);server.close()
            try:
                registry = start_observer()
                wait_until(lambda:any(r['state']=='running' for r in registry.snapshot()))
                stop_observer()
                reporter.set_state('recovering','dependency unavailable')
                registry = start_observer()
                wait_until(lambda:any(r['state']=='recovering' for r in registry.snapshot()))
                reporter.close()
                self.assertFalse(reporter.thread.is_alive())
                wait_until(lambda:any('已停止' in c.args[0] for c in registry.display.call_args_list))
                self.assertFalse(any('状态连接已断开' in c.args[0] for c in registry.display.call_args_list))
            finally:
                reporter.close()
                while servers:stop_observer()


if __name__ == '__main__':
    unittest.main()
