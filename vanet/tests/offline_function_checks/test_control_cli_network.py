"""人工控制命令使用当前 SUMO 的实验路网路径。"""
import contextlib
import io
from pathlib import Path
import unittest
from unittest.mock import Mock,patch

from vanet.experiments.experiment_session_management import control_cli


class ControlCliNetworkTests(unittest.TestCase):
    def test_start_uses_active_sumo_configuration_network(self):
        control=Mock();control.command.return_value=dict(ok=True)
        sumo=Mock();sumo.status.return_value=dict(config='/tmp/active/Town04.sumocfg')
        with patch.object(control_cli,'ControlClient',return_value=control), \
             patch.object(control_cli,'SumoClient',return_value=sumo), \
             patch.object(control_cli,'read_network',return_value=dict(network_file=Path('/tmp/active/Town04.net.xml'))) as read, \
             patch('sys.argv',['vanet_control','start']),contextlib.redirect_stdout(io.StringIO()):
            control_cli.main()
        read.assert_called_once_with('/tmp/active/Town04.sumocfg')
        control.command.assert_called_once_with('vanet.start',source='operator_cli',network_file='/tmp/active/Town04.net.xml')
        sumo.close.assert_called_once();control.close.assert_called_once()

    def test_status_does_not_require_a_sumo_connection(self):
        control=Mock();control.command.return_value=dict(ok=True)
        with patch.object(control_cli,'ControlClient',return_value=control), \
             patch.object(control_cli,'SumoClient') as sumo, \
             patch('sys.argv',['vanet_control','status']),contextlib.redirect_stdout(io.StringIO()):
            control_cli.main()
        sumo.assert_not_called()


if __name__=='__main__':unittest.main()
