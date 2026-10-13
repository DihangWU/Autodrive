"""Check that CARLA-controlled SUMO lights only send changed TraCI states."""

import os
from pathlib import Path
import sys
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault('SUMO_HOME', str(Path.home() / 'sumo-1.22.0'))
sys.path.insert(0, str(ROOT / 'carla-source/Co-Simulation/Sumo'))

from sumo_integration.sumo_simulation import SumoTLManager, SumoSignalState


def main():
    manager = object.__new__(SumoTLManager)
    manager._last_link_state = {}
    manager._off = False
    manager.get_all_signals = Mock(return_value={('junction', 0), ('junction', 1)})
    manager.get_all_associated_signals = Mock(side_effect=lambda landmark: {
        'first': {('junction', 0)},
        'second': {('junction', 1)},
    }[landmark])

    with patch('sumo_integration.sumo_simulation.traci.trafficlight.setLinkState') as send:
        manager.switch_off()
        assert send.call_count == 2
        assert set(call.args for call in send.call_args_list) == {
            ('junction', 0, SumoSignalState.OFF),
            ('junction', 1, SumoSignalState.OFF),
        }

        send.reset_mock()
        for _ in range(100):
            manager.set_state('first', SumoSignalState.RED)
            manager.set_state('second', SumoSignalState.GREEN)
        assert send.call_count == 2, send.call_args_list

        manager.set_state('first', SumoSignalState.GREEN)
        assert send.call_count == 3
        assert send.call_args.args == ('junction', 0, SumoSignalState.GREEN)

    print('PASS: initial state, unchanged frames, and light transition')


if __name__ == '__main__':
    main()
