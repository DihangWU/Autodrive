"""逐 NPC 行程与实际无线避让时延的测量契约。"""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from vanet.experiments.metrics_and_event_recording.metrics import Metrics


def vehicle(name='npc0',speed=0.,x=0.):
    return dict(id=name,x=x,y=0.,z=.75,heading=0.,length=4.,width=2.,height=1.5,speed=speed,role='npc')


class NpcMetricsTests(unittest.TestCase):
    def setUp(self):
        self.log=Mock();self.metrics=Metrics(self.log,dict(step_length=.05,hard_brake_mps2=4.))
        self.sumo=SimpleNamespace(simulation=Mock())
        for method in ('getCollidingVehiclesIDList','getStartingTeleportIDList','getEndingTeleportIDList','getArrivedIDList'):
            getattr(self.sumo.simulation,method).return_value=[]

    def sample(self,t,states):self.metrics.sample(t,states,self.sumo)

    def test_waiting_uses_elapsed_simulation_time_and_first_frame_has_no_invented_duration(self):
        self.sample(10.,[vehicle()]);self.sample(10.4,[vehicle()]);self.sample(11.,[vehicle()])
        self.assertAlmostEqual(self.metrics.values['npc_waiting_vehicle_seconds'],1.)
        self.assertEqual(self.metrics.values['npc_completed_trip_count'],0)
        self.assertIsNone(self.metrics.values['npc_mean_completed_trip_seconds'])

    def test_actual_arrival_completes_trip_and_integrates_observed_speed(self):
        self.sample(1.,[vehicle(speed=10.)]);self.sample(3.,[vehicle(speed=10.,x=20.)])
        self.sumo.simulation.getArrivedIDList.return_value=['npc0'];self.sample(4.,[])
        values=self.metrics.values
        self.assertEqual(values['npc_completed_trip_count'],1)
        self.assertEqual(values['npc_incomplete_trip_count'],0)
        self.assertEqual(values['npc_mean_completed_trip_seconds'],3.)
        self.assertEqual(values['npc_observed_distance_metres'],20.)
        self.assertEqual(values['npc_mean_speed_mps'],10.)
        events=[call.args[0] for call in self.log.emit.call_args_list]
        self.assertIn('npc_trip_completed',events)

    def test_missing_vehicle_is_not_arrival(self):
        self.sample(0.,[vehicle()]);self.sample(1.,[])
        self.assertEqual(self.metrics.npc_trips['npc0']['status'],'missing')
        self.assertEqual(self.metrics.values['npc_completed_trip_count'],0)
        self.assertEqual(self.metrics.values['npc_incomplete_trip_count'],1)

    def test_teleport_gap_does_not_inflate_distance_or_fake_arrival(self):
        self.sample(0.,[vehicle(speed=10.)]);self.sample(1.,[vehicle(speed=10.,x=10.)])
        self.sumo.simulation.getStartingTeleportIDList.return_value=['npc0'];self.sample(2.,[])
        self.sumo.simulation.getStartingTeleportIDList.return_value=[];self.sample(10.,[])
        self.sumo.simulation.getEndingTeleportIDList.return_value=['npc0'];self.sample(11.,[vehicle(speed=10.,x=1000.)])
        self.assertEqual(self.metrics.values['npc_observed_distance_metres'],10.)
        self.assertEqual(self.metrics.values['npc_completed_trip_count'],0)
        self.assertEqual(self.metrics.npc_trips['npc0']['teleport_count'],1)
        self.assertEqual(self.metrics.npc_trips['npc0']['first_seen_at'],0.)

    def test_repeated_sampling_same_time_does_not_double_count(self):
        self.sample(0.,[vehicle()]);self.sample(1.,[vehicle()]);self.sample(1.,[vehicle()])
        self.assertEqual(self.metrics.values['npc_waiting_vehicle_seconds'],1.)

    def test_yield_response_and_clearance_are_from_actual_received_reference(self):
        m=self.metrics
        m.note_yield_state('npc0','received',5.,'task-a','radio')
        m.note_yield_state('npc0','yielding',5.05,'task-a','lane_change')
        m.note_yield_state('npc0','cleared',7.,'task-a','outside_corridor')
        m.note_yield_state('npc0','cleared',8.,'task-a','outside_corridor')
        self.assertEqual(m.values['npc_yield_received_count'],1)
        self.assertEqual(m.values['npc_yield_cleared_count'],1)
        self.assertAlmostEqual(m.values['npc_mean_yield_response_seconds'],.05)
        self.assertEqual(m.values['npc_mean_yield_clearance_seconds'],2.)
        m.note_yield_state('npc0','normal',10.,None,'restored')
        self.assertEqual(m.yield_records[('npc0','task-a')]['restored_at'],10.)

    def test_missing_receive_reference_has_unknown_latency(self):
        self.metrics.note_yield_state('npc0','yielding',5.,'task-a','observed')
        self.metrics.note_yield_state('npc0','cleared',7.,'task-a','observed')
        self.assertIsNone(self.metrics.values['npc_mean_yield_response_seconds'])
        self.assertIsNone(self.metrics.values['npc_mean_yield_clearance_seconds'])

    def test_normal_traffic_does_not_create_yield_episode(self):
        self.assertFalse(self.metrics.note_yield_state('npc0','normal',1.,None,''))
        self.assertFalse(self.metrics.note_yield_state('npc0','normal',1.,'unused-task',''))
        self.assertEqual(self.metrics.yield_records,{})

    def test_same_vehicle_new_task_has_separate_latency(self):
        m=self.metrics
        for task,start,delay in [('first',1.,2.),('second',10.,4.)]:
            m.note_yield_state('npc0','received',start,task,'radio')
            m.note_yield_state('npc0','unable',start+delay,task,'no_space')
        self.assertEqual(m.values['npc_yield_unable_count'],2)
        self.assertEqual(m.values['npc_mean_yield_response_seconds'],3.)


if __name__=='__main__':unittest.main()
