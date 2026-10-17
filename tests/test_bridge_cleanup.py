"""Regression checks for independent clock lifetime and cleanup failures."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from foundation.cobridge import cobridge

CONTROL_FILE = Path(__file__).resolve().parents[1] / '.runtime/cobridge/test-control.sock'


class BridgeCleanupTests(unittest.TestCase):
    def run_case(self, step_error=None, startup_error=None, cleanup_error=None):
        runtime, control = MagicMock(), MagicMock()
        runtime.frames = 0
        def step():
            if step_error is not None:
                raise step_error
            runtime.frames += 1
        runtime.step.side_effect = step
        runtime.close.side_effect = cleanup_error
        args = SimpleNamespace(control_socket=CONTROL_FILE, step_length=.05, steps=1, fast=True)
        with patch.object(cobridge, 'BridgeRuntime', return_value=runtime), \
                patch.object(cobridge, 'ControlServer', return_value=control, side_effect=startup_error):
            try:
                cobridge.run_clock(args, MagicMock(is_set=lambda:False))
            finally:
                runtime.close.assert_called_once()

    def test_step_failure_survives_cleanup_failure(self):
        with self.assertRaisesRegex(RuntimeError, '^original step failure$'):
            self.run_case(step_error=RuntimeError('original step failure'),
                          cleanup_error=RuntimeError('secondary cleanup failure'))

    def test_control_startup_failure_survives_cleanup_failure(self):
        with self.assertRaisesRegex(ValueError, '^original startup failure$'):
            self.run_case(startup_error=ValueError('original startup failure'),
                          cleanup_error=RuntimeError('secondary cleanup failure'))

    def test_cleanup_failure_is_reported_after_successful_step(self):
        with self.assertRaisesRegex(RuntimeError, '^cleanup failure$'):
            self.run_case(cleanup_error=RuntimeError('cleanup failure'))

    def test_unavailable_simulators_do_not_stop_clock(self):
        args = SimpleNamespace(host='unused',port=2000,timeout=.1,step_length=.05,
                               sumo_socket=CONTROL_FILE,rpc_timeout=.1)
        runtime = cobridge.BridgeRuntime(args)
        with patch.object(cobridge, 'CarlaAdapter', side_effect=ConnectionError('offline')), \
                patch.object(cobridge, 'SumoAdapter', side_effect=ConnectionError('offline')):
            runtime.discover()
            runtime.step()
            self.assertEqual(runtime.frames,1)
            self.assertIsNone(runtime.carla)
            self.assertIsNone(runtime.sumo)
            runtime.close()

    def test_late_simulator_is_discovered_on_retry(self):
        args = SimpleNamespace(host='unused',port=2000,timeout=.1,step_length=.05,
                               sumo_socket=CONTROL_FILE,rpc_timeout=.1)
        runtime = cobridge.BridgeRuntime(args)
        adapter = MagicMock()
        with patch.object(cobridge,'CarlaAdapter',side_effect=[ConnectionError('offline'),adapter]), \
                patch.object(cobridge,'SumoAdapter',side_effect=ConnectionError('offline')):
            runtime.discover()
            self.assertIsNone(runtime.carla)
            runtime.next_discovery = 0
            runtime.discover()
            self.assertIs(runtime.carla,adapter)
            runtime.close()
            adapter.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
