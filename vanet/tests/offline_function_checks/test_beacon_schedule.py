"""验证可复现的 NPC 信标初始相位及保持不变的仿真时间周期。"""
import math
import unittest
from unittest.mock import Mock

from vanet.npc.cooperation import NpcCooperation
from vanet.npc.v2x_data_exchange.beacon_schedule import beacon_phase_seconds


class BeaconScheduleTests(unittest.TestCase):
    def test_phase_repeats_for_same_seed_and_node(self):
        phase = beacon_phase_seconds('npc0', seed=20261104, spread_seconds=.8)
        self.assertEqual(phase, beacon_phase_seconds('npc0', seed=20261104, spread_seconds=.8))
        self.assertNotEqual(phase, beacon_phase_seconds('npc1', seed=20261104, spread_seconds=.8))
        self.assertNotEqual(phase, beacon_phase_seconds('npc0', seed=20261105, spread_seconds=.8))

    def test_phase_is_bounded_and_zero_spread_is_immediate(self):
        for index in range(200):
            phase = beacon_phase_seconds(f'npc{index}', seed=20261104, spread_seconds=.8)
            self.assertGreaterEqual(phase, 0.)
            self.assertLessEqual(phase, .8)
            self.assertEqual(beacon_phase_seconds(f'npc{index}'), 0.)

    def test_default_config_preserves_immediate_first_beacon(self):
        connection = Mock()
        npc = NpcCooperation('npc0', {})
        output = npc.step(25., {'id': 'npc0', 'x': 10., 'y': 0.}, connection)
        self.assertEqual([packet['payload']['type'] for packet in output], ['beacon'])
        self.assertEqual(output[0]['payload']['sent_at'], 25.)
        self.assertEqual(npc.step(25., {'id': 'npc0'}, connection), [])
        self.assertEqual(connection.mock_calls, [])

    def test_initial_phase_is_relative_to_first_step_and_period_stays_one_second(self):
        for start in (0., 25.):
            with self.subTest(start=start):
                connection = Mock()
                config = {'seed': 20261104, 'beacon_phase_spread_seconds': .8,
                          'beacon_interval': 1.}
                phase = beacon_phase_seconds('npc0', seed=config['seed'], spread_seconds=.8)
                self.assertGreater(phase, 1e-5)
                npc = NpcCooperation('npc0', config)
                own_state = {'id': 'npc0', 'x': 10., 'y': 0.}
                self.assertEqual(npc.step(start, own_state, connection), [])
                first = start + phase
                self.assertEqual(npc.step(first - 1e-5, own_state, connection), [])
                sent = []
                for second in range(4):
                    when = first + second
                    if second:
                        self.assertEqual(npc.step(when - .01, own_state, connection), [])
                    output = npc.step(when, own_state, connection)
                    self.assertEqual([packet['payload']['type'] for packet in output], ['beacon'])
                    self.assertEqual(output[0]['sender'], 'npc0')
                    sent.append(output[0]['payload']['sent_at'])
                    self.assertEqual(npc.step(when, own_state, connection), [])
                for earlier, later in zip(sent, sent[1:]):
                    self.assertAlmostEqual(later - earlier, 1.)
                self.assertEqual(connection.mock_calls, [])

    def test_fifty_millisecond_steps_quantize_only_first_phase(self):
        config = {'seed': 20261104, 'beacon_phase_spread_seconds': .8, 'beacon_interval': 1.}
        npc = NpcCooperation('npc0', config)
        connection = Mock()
        phase = beacon_phase_seconds('npc0', seed=config['seed'], spread_seconds=.8)
        sent = []
        for step in range(81):
            output = npc.step(step * .05, {'id': 'npc0'}, connection)
            sent.extend(packet['payload']['sent_at'] for packet in output)
        self.assertAlmostEqual(sent[0], math.ceil(phase / .05) * .05)
        self.assertGreaterEqual(len(sent), 3)
        for earlier, later in zip(sent, sent[1:]):
            self.assertAlmostEqual(later - earlier, 1.)
        self.assertEqual(connection.mock_calls, [])


if __name__ == '__main__':
    unittest.main()
