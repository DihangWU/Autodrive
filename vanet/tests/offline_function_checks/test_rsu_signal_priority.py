"""信号优先的无线触发、逐路口释放、重传幂等及原配时恢复。"""
from unittest import TestCase
from unittest.mock import Mock,patch

from vanet.rsu.traffic_signal_priority.priority_controller import TrafficSignalPriority


class RsuSignalPriorityTests(TestCase):
    def setUp(self):
        self.connection=Mock();self.lights=self.connection.trafficlight
        self.connection.simulation.getTime.return_value=100.
        self.lights.getProgram.return_value='0';self.lights.getPhase.return_value=2
        self.lights.getNextSwitch.return_value=112.;self.lights.getRedYellowGreenState.return_value='rG'
        self.events=[]
        self.control=TrafficSignalPriority({},lambda event,now,**fields:self.events.append((event,fields)))
        self.message=dict(type='emergency',source='lincoln0',task_id='task',route_version=1,
                          route=[dict(x=0.,y=0.,z=0.),dict(x=200.,y=0.,z=0.)],
                          position=dict(x=0.,y=0.,z=0.))
        self.targets=[dict(controller=name,entry_station=entry,exit_station=entry+10.,green_state='Gr',
                           green_indices=[0],incoming=name+'_in',outgoing=name+'_out')
                      for name,entry in (('first',30.),('second',90.))]
        self.index=patch('vanet.rsu.traffic_signal_priority.priority_controller.RouteSignalIndex')
        self.index.start().return_value.match.return_value=self.targets
        self.addCleanup(self.index.stop)

    def receive(self):
        self.control.receive(self.message,1.,self.connection,'network',self.message['position'])

    def test_only_emergency_can_acquire_and_all_future_lights_change_together(self):
        self.control.update(self.message['position'],0.)
        for kind in ('beacon','identity','notice'):
            self.control.receive(dict(self.message,type=kind),0.,self.connection,'network',self.message['position'])
        self.lights.setRedYellowGreenState.assert_not_called()
        self.receive()
        self.assertEqual(set(self.control.active),{'first','second'})
        self.assertEqual(self.lights.setRedYellowGreenState.call_count,2)
        self.lights.setRedYellowGreenState.assert_any_call('first','Gr')
        self.lights.setRedYellowGreenState.assert_any_call('second','Gr')

    def test_wait_for_rear_to_leave_junction_and_restore_each_original_program(self):
        self.receive();self.lights.getRedYellowGreenState.return_value='Gr'
        self.control.update(dict(x=42.,y=0.,z=0.,length=5.),2.)
        self.assertIn('first',self.control.active)
        self.control.update(dict(x=44.,y=0.,z=0.,length=5.),3.)
        self.assertEqual(set(self.control.active),{'second'})
        self.lights.setProgram.assert_called_with('first','0')
        self.lights.setPhase.assert_called_with('first',2)
        self.lights.setPhaseDuration.assert_called_with('first',12.)
        self.receive()  # Same task/version periodic transmission must not relock first.
        self.assertEqual(set(self.control.active),{'second'})
        self.control.update(dict(x=104.,y=0.,z=0.,length=5.),4.)
        self.assertFalse(self.control.active)
        self.assertEqual(self.control.passed,{'first','second'})

    def test_cancel_and_reroute_restore_held_lights(self):
        self.receive()
        self.control.release_all(2.,'operator_cancelled')
        self.assertEqual(self.lights.setProgram.call_count,2)
        self.assertIsNone(self.control.task)
        self.receive();self.message['route_version']=2
        self.receive()
        self.assertEqual(self.lights.setProgram.call_count,4)
        self.assertEqual(self.control.task['route_version'],2)

    def test_initially_passed_lights_are_not_taken_over(self):
        self.message['position']['x']=60.;self.receive()
        self.assertEqual(set(self.control.active),{'second'})
        self.lights.setRedYellowGreenState.assert_called_once_with('second','Gr')

    def test_restoration_failure_keeps_record_for_retry_and_restores_other_lights(self):
        self.receive()
        self.lights.setProgram.side_effect=lambda name,program:(_ for _ in ()).throw(RuntimeError('failed')) if name=='first' else None
        with self.assertRaises(RuntimeError):self.control.release_all(2.,'reset')
        self.assertEqual(set(self.control.active),{'first'})
        self.assertIsNone(self.control.task)
        self.lights.setProgram.side_effect=None
        self.control.release_all(3.,'reset_retry')
        self.assertFalse(self.control.active)

    def test_disabled_feature_preserves_normal_signals(self):
        self.control.policy['enabled']=False;self.receive()
        self.lights.setRedYellowGreenState.assert_not_called()
