"""验证 RSU 有界回补、反馈有效期、常规续租和冲突点汇总。"""
import unittest

from vanet.rsu.cooperation import RsuCooperation
from vanet.tests.offline_function_checks.test_cooperation import message, state


class RsuBackfillTests(unittest.TestCase):
    def create_selected(self, **config):
        values = dict(lease_seconds=100., beacon_lifetime=100., notice_interval=1.)
        values.update(config)
        rsu = RsuCooperation(values)
        self.assertTrue(rsu.receive(message(), 0.))
        self.assertTrue(rsu.receive(message('beacon', source='npc0', state=state()), 0.))
        first = self.notice(rsu, 0.)
        self.assertFalse(first['backfill'])
        return rsu

    def notice(self, rsu, when):
        notices = [packet for packet in rsu.step(when) if packet['payload']['type'] == 'notice']
        self.assertEqual(len(notices), 1)
        self.assertEqual(notices[0]['recipient'], 'npc0')
        self.assertEqual(notices[0]['payload']['sent_at'], when)
        return notices[0]['payload']

    def test_default_three_backfills_then_regular_lease_notices_continue(self):
        rsu = self.create_selected()
        notices = [self.notice(rsu, float(second)) for second in range(1, 8)]
        self.assertEqual([notice['backfill'] for notice in notices],
                         [True, True, True, False, False, False, False])
        self.assertTrue(all(notice['lease_seconds'] == 100. for notice in notices))
        self.assertTrue(all(notice['route'] == message()['route'] for notice in notices))
        self.assertTrue(all(notice['route_version'] == 1 for notice in notices))

    def test_configured_cap_and_disabled_backfill_keep_periodic_notices(self):
        for limit in (0, 1, 2):
            with self.subTest(limit=limit):
                rsu = self.create_selected(max_backfill_attempts=limit)
                flags = [self.notice(rsu, float(second))['backfill'] for second in range(1, 5)]
                self.assertEqual(flags, [True] * limit + [False] * (4 - limit))
                self.assertFalse(any(packet['payload']['type'] == 'notice' for packet in rsu.step(4.5)))

    def test_expired_feedback_restarts_backfill_after_inclusive_lifetime(self):
        rsu = self.create_selected()
        self.assertTrue(rsu.receive(message('feedback', source='npc0', time=0., sequence=2,
                                            status='yielding'), 0.))
        self.assertEqual([self.notice(rsu, float(second))['backfill'] for second in (1, 2, 3)],
                         [False, False, False])
        self.assertTrue(self.notice(rsu, 4.)['backfill'])

    def test_valid_feedback_replenishes_exhausted_backfill_budget(self):
        rsu = self.create_selected(max_backfill_attempts=2)
        self.assertTrue(self.notice(rsu, 1.)['backfill'])
        self.assertTrue(self.notice(rsu, 2.)['backfill'])
        self.assertFalse(self.notice(rsu, 3.)['backfill'])
        self.assertTrue(rsu.receive(message('feedback', source='npc0', time=3.5, sequence=2,
                                            status='yielding'), 3.5))
        self.assertEqual([self.notice(rsu, float(second))['backfill'] for second in (4, 5, 6)],
                         [False, False, False])
        self.assertTrue(self.notice(rsu, 7.)['backfill'])
        self.assertTrue(self.notice(rsu, 8.)['backfill'])
        self.assertFalse(self.notice(rsu, 9.)['backfill'])

    def test_route_version_change_starts_fresh_notice_and_retry_budget(self):
        rsu = self.create_selected(max_backfill_attempts=1)
        self.assertTrue(self.notice(rsu, 1.)['backfill'])
        self.assertFalse(self.notice(rsu, 2.)['backfill'])
        self.assertTrue(rsu.receive(message(time=2.5, sequence=2, version=2), 2.5))
        notice = self.notice(rsu, 2.5)
        self.assertEqual(notice['route_version'], 2)
        self.assertFalse(notice['backfill'])
        self.assertTrue(self.notice(rsu, 3.5)['backfill'])
        self.assertFalse(self.notice(rsu, 4.5)['backfill'])

    def test_reselected_vehicle_starts_fresh_notice_and_retry_budget(self):
        rsu = self.create_selected(max_backfill_attempts=1)
        self.assertTrue(self.notice(rsu, 1.)['backfill'])
        self.assertFalse(self.notice(rsu, 2.)['backfill'])
        self.assertTrue(rsu.receive(message(time=2.5, sequence=2,
                                            position=dict(x=100., y=0., z=0.)), 2.5))
        removed = rsu.step(2.5)
        self.assertEqual(rsu.selected, {})
        self.assertTrue(any(packet['payload']['type'] == 'cancel' for packet in removed))
        self.assertTrue(rsu.receive(message('beacon', source='npc0', time=3., sequence=2,
                                            state=state(x=130.)), 3.))
        self.assertFalse(self.notice(rsu, 3.)['backfill'])
        self.assertTrue(self.notice(rsu, 4.)['backfill'])
        self.assertFalse(self.notice(rsu, 5.)['backfill'])

    def test_clearance_reports_selected_conflict_points_as_deep_copies(self):
        rsu = RsuCooperation(dict(lease_seconds=100., beacon_lifetime=100.))
        self.assertTrue(rsu.receive(message(), 0.))
        self.assertTrue(rsu.receive(message('beacon', source='npc0',
                                            state=state(x=30., y=10., heading=-90.)), 0.))
        self.assertTrue(rsu.receive(message('beacon', source='far_npc',
                                            state=state(id='far_npc', x=300., y=100.)), 0.))
        first = next(packet['payload'] for packet in rsu.step(0.)
                     if packet['payload']['type'] == 'clearance')
        self.assertEqual(set(first['conflict_points']), {'npc0'})
        self.assertAlmostEqual(first['conflict_points']['npc0']['x'], 30.)
        self.assertAlmostEqual(first['conflict_points']['npc0']['y'], 0.)
        self.assertAlmostEqual(first['conflict_points']['npc0']['z'], 0.)
        first['conflict_points']['npc0']['x'] = 999.
        self.assertTrue(rsu.receive(message('beacon', source='npc0', time=1., sequence=2,
                                            state=state(x=30., y=8., heading=-90., speed=0.)), 1.))
        later = next(packet['payload'] for packet in rsu.step(1.)
                     if packet['payload']['type'] == 'clearance')
        self.assertAlmostEqual(later['conflict_points']['npc0']['x'], 30.)
        self.assertAlmostEqual(later['conflict_points']['npc0']['y'], 0.)


if __name__ == '__main__':
    unittest.main()
