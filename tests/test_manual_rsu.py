"""RSU owns its entities independently of CoBridge and simulator lifetime."""
import hashlib
from pathlib import Path
from types import SimpleNamespace
import unittest
import threading
from unittest.mock import MagicMock, patch

from modules.rsu import rsu

CONFIG_FILE = Path(__file__).resolve().parents[1] / 'foundation/cobridge/rsu/config/town04.json'
MAP_CONTENT = 'test map'


class ManualRsuTests(unittest.TestCase):
    def run_open(self, setup_error=None, fingerprint=None):
        world, client, sumo, component = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        client.get_world.return_value = world
        world.get_map.return_value.to_opendrive.return_value = MAP_CONTENT
        sumo.status.return_value = {'config': str(CONFIG_FILE)}
        network = dict(xodr_sha256=fingerprint or hashlib.sha256(MAP_CONTENT.encode()).hexdigest())
        component.setup.side_effect = setup_error
        args = SimpleNamespace(host='unused',port=2000,timeout=1,sumo_socket=CONFIG_FILE,
                               rpc_timeout=1,config=CONFIG_FILE)
        with patch.dict('sys.modules', {'carla':SimpleNamespace(Client=lambda *a:client)}), \
                patch.object(rsu,'SumoClient',return_value=sumo), \
                patch.object(rsu,'read_network',return_value=network), \
                patch.object(rsu,'RsuBridge',return_value=component):
            try:
                result = rsu.open_session(args)
            except Exception:
                sumo.close.assert_called_once()
                if setup_error is not None:component.close.assert_called_once()
                raise
            return result, component, sumo

    def test_creates_without_connecting_to_cobridge_or_changing_clock(self):
        result, component, sumo = self.run_open()
        component.setup.assert_called_once()
        result[1].tick.assert_not_called()
        result[1].apply_settings.assert_not_called()
        sumo.claim_clock.assert_not_called()
        sumo.connection.simulationStep.assert_not_called()
        sumo.close.assert_not_called()

    def test_failed_setup_cleans_partial_resources_and_client(self):
        with self.assertRaisesRegex(ValueError,'bad setup'):
            self.run_open(setup_error=ValueError('bad setup'))

    def test_map_mismatch_closes_only_own_client(self):
        with self.assertRaisesRegex(RuntimeError,'地图一致'):
            self.run_open(fingerprint='different map')

    def test_entity_session_publishes_status_without_observer_dependency(self):
        stopped = threading.Event()
        client,world,sumo,component,reporter = (MagicMock() for _ in range(5))
        client.get_world.return_value = world
        component.after_step.side_effect = lambda *args:stopped.set()
        args = SimpleNamespace(duration=0)
        with patch.object(rsu,'open_session',return_value=(client,world,sumo,component)):
            rsu.run(args,stopped,reporter)
        reporter.set_state.assert_called_once_with('running','实体已创建')
        component.close.assert_called_once()
        sumo.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
