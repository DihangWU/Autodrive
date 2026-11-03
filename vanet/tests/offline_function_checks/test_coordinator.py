"""离线检查实验协调器：准备隔离、无线因果、任务过滤和人工状态边界。"""
import logging
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from vanet.experiments.experiment_session_management import coordinator as module


class MemoryLog:
    def __init__(self,*args,**kwargs):
        self.records=[]
        self.handler=logging.NullHandler()
        self.logger=logging.getLogger('test.coordinator')

    def emit(self,event,sim_time=None,**fields):
        self.records.append((event,sim_time,fields))

    def save_config(self,configuration):pass
    def save_metrics(self,metrics):pass
    def close(self):pass

    def network_boundary(self,result):
        self.records.append(('network_boundary',result.get('time'),result))


class FakeNetwork:
    def __init__(self):
        self.status=dict(ready=True,error=None,reset_pending=False)
        self.requests=[]
        self.results=[]
        self.reset_requests=[]
        self.pending=False
        self.ready=True

    def start(self):pass
    def poll(self):pass
    def close(self):pass

    def submit(self,*args):
        self.requests.append(args)

    def take_result(self):
        return self.results.pop(0) if self.results else None

    def reset(self,seed,results_directory=None):
        self.reset_requests.append(seed)
        self.status['reset_pending']=True

    def request_shutdown(self):
        self.results.append(dict(ok=True,stopped=True))

    def acknowledge_reset(self):
        self.status['reset_pending']=False
        self.results.append(dict(ok=True,reset=True,time=0.))


class FakeMetrics:
    def __init__(self,*args):
        self.values=dict(status='prepared',messages_sent=0,elapsed_seconds=0.)
        self.samples=[]
        self.deliveries=[]
        self.yield_states=[]

    def sample(self,*args):
        self.samples.append(args)

    def delivery(self,message):
        self.deliveries.append(message)

    def note_yield_state(self,*args):
        self.yield_states.append(args)


class CoordinatorContractTests(unittest.TestCase):
    def setUp(self):
        self.snapshot=SimpleNamespace(frame=100)
        self.world=SimpleNamespace(get_snapshot=lambda:self.snapshot)
        self.bridge=SimpleNamespace(
            carla=SimpleNamespace(world=self.world),
            sumo=SimpleNamespace(network=dict(network_file='/tmp/vanet-test-town04.net.xml'),connection=Mock()),
            lincoln=SimpleNamespace(records={'3:7':dict(actor_id=7,world_id=3)},navigation=Mock()),
            npc=SimpleNamespace(batches={}))
        self.patchers=[
            patch.object(module,'new_result_directory',return_value=Path('/tmp/vanet-coordinator-test')),
            patch.object(module,'EventLog',MemoryLog),
            patch.object(module,'Metrics',FakeMetrics),
            patch.object(module,'load_scenario',return_value=dict(lincoln=dict(destination=dict(x=10.,y=0.,z=0.)))),
            patch.object(module,'collect_mobility',return_value=([],[])),
        ]
        for patcher in self.patchers:patcher.start()
        self.coordinator=module.ExperimentCoordinator(self.bridge)
        self.coordinator.network=FakeNetwork()
        self.coordinator.scenario=Mock()
        self.coordinator.scenario.started=False
        self.coordinator.scenario.snapshot.return_value=dict(initial_npcs_added=True)
        self.coordinator.ack_frame=100
        self.coordinator.task_id='current-task'
        self.coordinator.group='cooperative'
        self.coordinator.lincoln_status=dict(emergency=True,state='prepared',manual=False,
                                            route_version=1,route=[dict(x=0,y=0,z=0),dict(x=10,y=0,z=0)],
                                            position=dict(x=0,y=0,z=0),speed=0.,heading=0.,eta=5.)
        self.coordinator.rsu=Mock()
        self.coordinator.rsu.step.return_value=[]
        self.coordinator.rsu.snapshot.return_value={}

    def tearDown(self):
        self.coordinator.close()
        logging.getLogger('vanet.npc').removeHandler(self.coordinator.log.handler)
        logging.getLogger('vanet.rsu').removeHandler(self.coordinator.log.handler)
        for patcher in reversed(self.patchers):patcher.stop()

    def gui(self,action,**fields):
        return dict(action=action,source='sumo_gui',network_file='/tmp/vanet-test-town04.net.xml',**fields)

    def exchange(self,**fields):
        return dict(actor_id=7,world_id=3,ack_frame=100,**fields)

    def delivery(self,recipient='npc-a',task='current-task',kind='notice'):
        return dict(sender='rsu0',recipient=recipient,sent_at=.01,received_at=.02,
                    payload=dict(type=kind,source='rsu0',task_id=task,route_version=1,sequence=1,sent_at=.01))

    def put_result(self,deliveries=()):
        self.coordinator.network.results.append(dict(ok=True,time=.05,step_id=1,
                                                    deliveries=list(deliveries),stats={}))
        self.coordinator.pending_network=True

    def test_preparation_never_advances_network_sends_messages_or_counts_travel_time(self):
        c=self.coordinator
        c.phase='preparing'
        c.lincoln_status['state']='preparing'
        for _ in range(3):self.assertFalse(c.before_step())
        self.assertEqual(c.network.requests,[])
        self.assertEqual(c.network_time,0.)
        self.assertEqual(c.metrics.samples,[])
        c.lincoln_status['state']='prepared'
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'prepared')
        self.assertFalse(c.before_step())
        c.scenario.start.assert_not_called()
        c.rsu.step.assert_not_called()
        self.assertEqual(c.network.requests,[])

    def test_start_rejected_until_prepared_and_waits_for_lincoln_acknowledgement(self):
        c=self.coordinator
        for phase in ('waiting','ready','preparing','running','paused'):
            c.phase=phase
            with self.assertRaisesRegex(RuntimeError,'准备'):
                c.handle(self.gui('vanet.start'))
        c.phase='prepared'
        c.handle(self.gui('vanet.start'))
        command=c.command
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'prepared')
        c.scenario.start.assert_not_called()
        c.exchange(self.exchange(command_ack_id=command['id'],status=dict(state='running',emergency=True,route_version=1)))
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'running')
        c.scenario.start.assert_called_once_with(0.)
        self.assertEqual(len(c.network.requests),1)
        self.assertEqual(c.run_start_network_time,0.)

    def test_received_network_events_wait_for_traffic_step_commit(self):
        c=self.coordinator;c.phase='running'
        npc=Mock();c.npc['npc-a']=npc
        message=self.delivery()
        self.put_result([message])
        self.assertTrue(c.before_step())
        npc.receive.assert_not_called()
        self.assertEqual(c.network_time,0.)
        self.assertIsNotNone(c.network_result)
        self.assertEqual(c.metrics.samples,[])
        # CoBridge 在 before_step 返回 True 后完成 CARLA/SUMO 推进，才调用 after_step。
        self.snapshot.frame+=1
        c.after_step()
        npc.receive.assert_called_once_with(message['payload'],.02)
        self.assertEqual(c.network_time,.05)
        self.assertEqual(len(c.metrics.samples),1)
        c.after_step()
        npc.receive.assert_called_once()

    def test_noncooperative_group_records_delivery_without_triggering_yield(self):
        c=self.coordinator;c.phase='running';c.group='emergency'
        npc=Mock();c.npc['npc-a']=npc
        message=self.delivery()
        self.put_result([message])
        self.assertTrue(c.before_step());c.after_step()
        npc.receive.assert_not_called()
        c.rsu.receive.assert_not_called()
        self.assertEqual(c.metrics.deliveries,[message])
        self.assertEqual(c.application_step([]),[])

    def test_stale_task_messages_cannot_reach_npc_rsu_or_lincoln(self):
        c=self.coordinator;c.phase='running'
        npc=Mock();c.npc['npc-a']=npc
        messages=[self.delivery(recipient,'previous-task') for recipient in ('npc-a','rsu0','lincoln0')]
        self.put_result(messages)
        self.assertTrue(c.before_step());c.after_step()
        npc.receive.assert_not_called();c.rsu.receive.assert_not_called()
        self.assertEqual(c.deliveries,[])
        self.assertEqual(sum(event=='stale_task_discarded' for event,_,_ in c.log.records),3)

    def test_manual_takeover_preserves_identity_and_active_route_request(self):
        c=self.coordinator;c.phase='running'
        c.exchange(self.exchange(status=dict(state='manual',manual=True,emergency=True,route_version=1)))
        self.assertTrue(c.emergency)
        self.assertEqual(c.phase,'running')
        messages=c.application_step([])
        self.assertEqual([m['payload']['type'] for m in messages],['emergency'])
        self.assertIsNone(c.command)

    def test_arrival_ends_route_request_but_preserves_identity_until_sumo_cancel(self):
        c=self.coordinator;c.phase='running'
        c.lincoln_status['state']='arrived'
        self.put_result()
        self.assertTrue(c.before_step());c.after_step()
        self.assertEqual(c.phase,'arrived')
        self.assertTrue(c.emergency)
        self.assertTrue(c.ended)
        messages=c.application_step([])
        self.assertEqual([m['payload']['type'] for m in messages],['cancel','identity'])
        self.assertEqual(messages[0]['payload']['reason'],'route_arrived')
        c.handle(self.gui('vanet.cancel'))
        self.assertTrue(c.emergency)  # 等待车辆真正执行取消后才显示已取消。
        command=c.command
        c.exchange(self.exchange(command_ack_id=command['id'],status=dict(state='cancelled',emergency=False,route_version=1)))
        self.assertFalse(c.emergency)

    def test_reset_waits_for_both_network_reset_and_lincoln_command(self):
        c=self.coordinator
        c.phase='resetting';c.network_reset_required=True;c.reset_steps=0
        c.lincoln_status.update(state='reset',emergency=False)
        c.issue('reset',pose=dict(x=0,y=0,z=.5,yaw=0))
        command=c.command
        self.assertFalse(c.before_step())
        self.assertEqual(c.network.reset_requests,[c.config['seed']])
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'resetting')
        c.network.acknowledge_reset()
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'resetting')
        c.exchange(self.exchange(command_ack_id=command['id'],status=dict(state='reset',emergency=False,route_version=0,route=[])))
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'ready')
        self.assertEqual(c.network.requests,[])

    def test_cancel_before_start_keeps_cleanup_out_of_experiment_metrics(self):
        c=self.coordinator;c.phase='prepared'
        c.handle(self.gui('vanet.cancel'))
        self.assertTrue(c.ended)
        self.assertEqual(c.metrics.values['status'],'cancelled_before_start')
        self.put_result([self.delivery('lincoln0',kind='clearance')])
        self.assertTrue(c.before_step());c.after_step()
        self.assertEqual(c.metrics.samples,[])
        self.assertEqual(c.metrics.deliveries,[])
        c.scenario.start.assert_not_called()

    def test_identity_broadcasts_after_arrival_do_not_change_finished_trip_metrics(self):
        c=self.coordinator;c.phase='arrived';c.ended=True
        c.lincoln_status['state']='arrived'
        c.metrics.values['messages_sent']=12
        self.assertFalse(c.before_step())
        self.assertEqual(len(c.network.requests),1)
        self.assertEqual(c.network.requests[0][-1][0]['payload']['type'],'identity')
        self.assertEqual(c.metrics.values['messages_sent'],12)
        self.put_result([self.delivery('lincoln0',kind='clearance')])
        self.assertTrue(c.before_step());c.after_step()
        self.assertEqual(c.metrics.deliveries,[])
        self.assertEqual(c.metrics.samples,[])

    def test_frame_barrier_prevents_network_advance_before_control_ack(self):
        c=self.coordinator;c.phase='running';c.ack_frame=99
        self.assertFalse(c.before_step())
        self.assertEqual(c.network.requests,[])
        c.exchange(self.exchange(status=dict(state='running',emergency=True,route_version=1)))
        self.assertFalse(c.before_step())
        self.assertEqual(len(c.network.requests),1)

    def test_network_disconnect_pauses_without_clearing_emergency_identity(self):
        c=self.coordinator;c.phase='running'
        c.network.status.update(error='connection lost',ready=False)
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'paused')
        self.assertTrue(c.emergency)
        self.assertEqual(c.task_id,'current-task')
        self.assertEqual(c.network.requests,[])
        self.assertEqual(c.metrics.values['status'],'invalid_paused')

    def test_simulator_disconnect_pauses_without_accepting_pending_network_result(self):
        c=self.coordinator;c.phase='running'
        self.put_result([self.delivery()])
        self.bridge.carla=None
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'paused')
        self.assertTrue(c.emergency)
        self.assertIsNone(c.network_result)
        self.assertEqual(c.metrics.deliveries,[])

    def test_failed_start_ack_does_not_start_traffic_or_wireless_experiment(self):
        c=self.coordinator;c.phase='prepared'
        c.handle(self.gui('vanet.start'))
        command=c.command
        c.exchange(self.exchange(command_ack_id=command['id'],status=dict(
            state='error',error='Prepared control route unavailable',emergency=True,route_version=1)))
        self.assertFalse(c.before_step())
        self.assertNotEqual(c.phase,'running')
        c.scenario.start.assert_not_called()
        self.assertEqual(c.network.requests,[])

    def test_start_ack_followed_by_manual_takeover_does_not_deadlock_prepared_phase(self):
        c=self.coordinator;c.phase='prepared'
        c.handle(self.gui('vanet.start'))
        command=c.command
        c.exchange(self.exchange(command_ack_id=command['id'],status=dict(
            state='manual',manual=True,emergency=True,route_version=1)))
        self.assertFalse(c.before_step())
        self.assertEqual(c.phase,'running')
        self.assertTrue(c.emergency)
        c.scenario.start.assert_called_once_with(0.)
        self.assertEqual(len(c.network.requests),1)


if __name__=='__main__':unittest.main()
