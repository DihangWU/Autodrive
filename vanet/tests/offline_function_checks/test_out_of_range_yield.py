"""失联只延续已有避让动作；30 秒截止不随轮询或重入自动延长。"""
import unittest
from unittest.mock import Mock
from vanet.npc.cooperation import NpcCooperation
from vanet.tests.offline_function_checks.test_cooperation import message,state


class OutOfRangeYieldTests(unittest.TestCase):
    def npc(self):
        npc=NpcCooperation('npc0',dict(communication_ranges=dict(v2v_metres=50.,rsu_metres=170.),
                                      out_of_range_yield_seconds=30.))
        npc.actions=Mock(saved=None,action=None,restore_started=None)
        npc.actions.apply.return_value=('yielding','changing_to_clear_lane')
        npc.actions.restore.return_value=False
        npc.set_communication_links(['lincoln0'],0.)
        npc.set_communication_path('direct',0.)
        return npc

    def leave(self,npc,now=1.):
        npc.set_communication_links([],now)
        npc.set_communication_path(None,now)

    def test_task_continues_for_30_seconds_after_exit_despite_original_lease(self):
        npc=self.npc();npc.receive(message(),0.);npc.step(0.,state(),Mock())
        self.leave(npc)
        for now in (1.,10.1,30.999):
            output=npc.step(now,state(),Mock())
            self.assertFalse(npc.recovering)
            self.assertEqual(output,[])
            self.assertEqual(npc.lease_until,31.)
            self.assertTrue(npc.snapshot()['out_of_range_yielding'])
        npc.actions.restore.assert_not_called()
        npc.step(31.,state(),Mock())
        self.assertTrue(npc.recovering)
        self.assertEqual(npc.reason,'out_of_range_yield_expired')
        npc.actions.restore.assert_called_once()

    def test_no_received_task_cannot_create_a_hold_or_receive_outside_range(self):
        npc=self.npc();self.leave(npc)
        self.assertFalse(npc.receive(message(time=2.),2.))
        npc.step(2.,state(),Mock())
        self.assertFalse(npc.snapshot()['out_of_range_yielding'])
        npc.actions.apply.assert_not_called()

    def test_polling_and_reentry_without_a_packet_do_not_extend_deadline(self):
        npc=self.npc();npc.receive(message(),0.);self.leave(npc)
        for now in (2.,5.,10.):
            npc.set_communication_links(['lincoln0'],now)
            npc.set_communication_path('direct',now)
            self.leave(npc,now+.1)
        self.assertEqual(npc.range_leases.current(30.)['until'],31.)
        self.assertIsNone(npc.range_leases.current(31.))

    def test_fresh_message_clears_hold_and_next_exit_starts_a_new_period(self):
        npc=self.npc();npc.receive(message(),0.);self.leave(npc)
        npc.set_communication_links(['lincoln0'],2.);npc.set_communication_path('direct',2.)
        self.assertTrue(npc.receive(message(time=2.,sequence=2),2.))
        self.assertFalse(npc.range_leases.holding(2.))
        self.leave(npc,3.)
        self.assertEqual(npc.range_leases.current(4.)['until'],33.)

    def test_expired_task_does_not_acquire_a_new_hold(self):
        npc=self.npc();npc.receive(message(),0.);self.leave(npc,11.)
        self.assertIsNone(npc.range_leases.current(11.))

    def test_decimal_simulation_time_does_not_add_an_extra_traffic_step(self):
        npc=self.npc();npc.receive(message(),0.);self.leave(npc,.35)
        npc.step(30.349999999999998,state(),Mock())
        self.assertTrue(npc.recovering)
        self.assertEqual(npc.reason,'out_of_range_yield_expired')

    def test_cancel_received_after_reentry_clears_hold_immediately(self):
        npc=self.npc();npc.receive(message(),0.);self.leave(npc)
        npc.set_communication_links(['lincoln0'],2.);npc.set_communication_path('direct',2.)
        self.assertTrue(npc.receive(message('cancel',time=2.,sequence=2),2.))
        npc.step(2.,state(),Mock())
        self.assertFalse(npc.range_leases.holding(2.))
        self.assertTrue(npc.recovering)

    def test_handover_holds_action_but_requires_new_channel_packet_for_feedback(self):
        npc=self.npc();npc.receive(message(),0.)
        npc.set_communication_links(['rsu0'],1.);npc.set_communication_path('rsu',1.)
        output=npc.step(1.,state(),Mock())
        self.assertFalse(npc.recovering)
        self.assertFalse(any(m['payload']['type']=='feedback' for m in output))
        self.assertTrue(npc.range_leases.holding(1.))
        notice=message('notice',source='rsu0',time=2.,route=message()['route'])
        self.assertTrue(npc.receive(notice,2.))
        self.assertFalse(npc.range_leases.holding(2.))

    def test_leaving_corridor_during_hold_does_not_start_recovery(self):
        npc=self.npc();npc.receive(message(),0.)
        npc.actions.saved={'speed_mode':31}
        self.leave(npc)
        npc.step(1.,state(y=20.),Mock())
        self.assertEqual(npc.status,'cleared')
        self.assertFalse(npc.recovering)
        npc.actions.restore.assert_not_called()
