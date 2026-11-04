"""直连优先、双端 RSU 覆盖、无线因果及反馈通道切换。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from vanet.protocol.communication_path_selection.paths import select_communication_paths
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.lincoln.v2x_data_exchange.wireless_clearance_advice import WirelessClearanceAdvice
from vanet.tests.offline_function_checks.test_cooperation import message,state,connection

CONFIG=dict(communication_ranges=dict(v2v_metres=50.,rsu_metres=170.),rsu=dict(id='rsu0',x=100.,y=0.))


class CommunicationPathTests(unittest.TestCase):
    def test_direct_wins_and_exact_range_boundaries_are_included(self):
        states=[dict(id='lincoln0',role='lincoln',x=0.,y=0.)]
        states += [dict(id=str(x),role='npc',x=x,y=0.) for x in (50.,50.1,270.,270.1)]
        paths,connected=select_communication_paths(states,CONFIG['rsu'],CONFIG['communication_ranges'])
        self.assertTrue(connected)
        self.assertEqual(list(paths.values()),['direct','rsu','rsu',None])

    def test_npc_in_rsu_circle_is_insufficient_when_lincoln_is_outside(self):
        states=[dict(id='lincoln0',role='lincoln',x=300.,y=0.),
                dict(id='near',role='npc',x=330.,y=0.),dict(id='remote',role='npc',x=100.,y=0.)]
        paths,connected=select_communication_paths(states,CONFIG['rsu'],CONFIG['communication_ranges'])
        self.assertFalse(connected);self.assertEqual(paths,dict(near='direct',remote=None))

    def test_direct_npc_feedback_and_beacon_go_to_lincoln_without_rsu_notice(self):
        npc=NpcCooperation('npc0',CONFIG)
        npc.set_communication_links(['lincoln0','rsu0'],0.)
        npc.set_communication_path('direct',0.)
        npc.step(0.,state(),connection())
        self.assertIsNone(npc.task)  # Range eligibility cannot create an emergency.
        self.assertFalse(npc.receive(message('notice',source='rsu0',route=message()['route']),0.))
        self.assertTrue(npc.receive(message(),0.))
        feedback=[m for m in npc.step(.05,state(),connection()) if m['payload']['type']=='feedback']
        self.assertTrue(feedback)
        self.assertTrue(all(m['recipient']=='lincoln0' for m in feedback))
        self.assertIn('position',feedback[-1]['payload'])

    def test_path_handover_requires_a_new_message_on_selected_channel(self):
        npc=NpcCooperation('npc0',CONFIG)
        npc.set_communication_links(['lincoln0','rsu0'],0.)
        npc.set_communication_path('direct',0.);npc.receive(message(),0.)
        npc.set_communication_path('rsu',1.)
        self.assertIsNone(npc.range_leases.current(1.))
        self.assertFalse(npc.receive(message(time=1.,sequence=2),1.))
        self.assertTrue(npc.receive(message('notice',source='rsu0',time=1.,route=message()['route']),1.))
        npc.set_communication_path('direct',2.)
        self.assertIsNone(npc.range_leases.current(2.))

    def test_rsu_notices_only_relay_targets_and_discards_cache_on_handover(self):
        rsu=RsuCooperation(CONFIG)
        rsu.set_communication_links(['lincoln0','near','remote'],0.)
        rsu.set_relay_targets(['remote'],0.)
        rsu.receive(message(),0.)
        self.assertFalse(rsu.receive(message('beacon',source='near',state=state()),0.))
        self.assertTrue(rsu.receive(message('beacon',source='remote',state=state(x=80.)),0.))
        notices=[m for m in rsu.step(0.) if m['payload']['type']=='notice']
        self.assertEqual([m['recipient'] for m in notices],['remote'])
        rsu.set_relay_targets([],1.)
        self.assertEqual(rsu.step(1.),[])
        self.assertFalse(rsu.beacons.entries)
        rsu.set_relay_targets(['remote'],2.)
        self.assertFalse(any(m['payload']['type']=='notice' for m in rsu.step(2.)))


class DirectAdviceTests(unittest.TestCase):
    def setUp(self):
        self.advice=WirelessClearanceAdvice()
        self.samples=[SimpleNamespace(center=(0.,0.,0.)),SimpleNamespace(center=(200.,0.,0.))]

    def feedback(self,**fields):
        return message('feedback',source='npc0',status='yielding',conflict_point=dict(x=30,y=0,z=0),**fields)

    def evaluate(self,now=0.):
        return self.advice.evaluate(now,'task-a',1,(0.,0.,0.),self.samples,[0.,200.],0.)

    def test_actual_direct_feedback_is_used_without_any_rsu_message(self):
        self.advice.set_paths({'npc0':'direct'},False)
        self.assertFalse(self.evaluate()['active'])
        self.assertTrue(self.advice.receive(self.feedback(),0.,'task-a',1))
        result=self.evaluate()
        self.assertTrue(result['active']);self.assertEqual(result['reason'],'pending_direct_feedback')
        self.assertEqual(result['vehicles'],['npc0'])
        self.assertFalse(self.advice.receive(self.feedback(),0.,'task-a',1))

    def test_direct_feedback_expires_and_does_not_revive_after_reentry(self):
        self.advice.set_paths({'npc0':'direct'},True)
        self.advice.receive(self.feedback(),0.,'task-a',1)
        self.assertFalse(self.evaluate(3.1)['active'])
        self.advice.set_paths({'npc0':'rsu'},True)
        self.assertFalse(self.advice.receive(self.feedback(sequence=2,time=.1),.1,'task-a',1))
        self.advice.set_paths({'npc0':'direct'},True)
        self.assertFalse(self.evaluate(.2)['active'])

    def test_rsu_and_direct_feedback_can_cover_different_npcs(self):
        self.advice.set_paths({'npc0':'direct','remote':'rsu'},True)
        self.advice.receive(self.feedback(),0.,'task-a',1)
        summary=message('clearance',source='rsu0',vehicles={'remote':'yielding','npc0':'unable'},
                        conflict_points={'remote':dict(x=80,y=0,z=0),'npc0':dict(x=5,y=0,z=0)})
        self.assertTrue(self.advice.receive(summary,0.,'task-a',1))
        result=self.evaluate()
        self.assertEqual(result['vehicles'],['npc0','remote'])
        self.assertEqual(result['nearest_distance_metres'],30.)
        self.advice.set_paths({'npc0':None,'remote':None},False)
        self.assertFalse(self.evaluate()['active'])
