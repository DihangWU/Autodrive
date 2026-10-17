"""SUMO emits successful-operation events, bridge acknowledges, SUMO displays."""
import unittest
from unittest.mock import MagicMock, patch

from foundation.cobridge.adapters.sumo import SumoAdapter
from foundation.sumo.events import OperationEvents
from foundation.sumo.runtime import SumoRuntime

STEP_LENGTH = .05
CONFIG = 'test.sumocfg'
ENTITY_ID = 'test_object'


class SumoEventTests(unittest.TestCase):
    def make_runtime(self):
        display,connection = MagicMock(),MagicMock()
        connection.simulation.getTime.return_value = 0
        return SumoRuntime(connection,CONFIG,STEP_LENGTH,on_operation_confirmed=display),display

    def test_successful_creation_and_deletion_display_after_bridge_ack(self):
        runtime,display = self.make_runtime()
        module,bridge = object(),object()
        runtime.handle(dict(action='call',method='poi.add',args=[ENTITY_ID,1,2,[255,0,0,255]]),module)
        runtime.handle(dict(action='call',method='poi.remove',args=[ENTITY_ID]),module)
        display.assert_not_called()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),bridge)
        pending = runtime.handle(dict(action='events.poll'),bridge)['events']
        self.assertEqual([e['method'] for e in pending],['poi.add','poi.remove'])
        self.assertEqual([e['entity'] for e in pending],[ENTITY_ID,ENTITY_ID])
        ids = [e['id'] for e in pending]
        runtime.handle(dict(action='events.ack',ids=ids),bridge)
        self.assertEqual([c.args[0] for c in display.call_args_list],
                         [f'poi {ENTITY_ID}：创建成功。',f'poi {ENTITY_ID}：删除成功。'])
        runtime.handle(dict(action='events.ack',ids=ids),bridge)
        self.assertEqual(display.call_count,2)
        self.assertEqual(runtime.events.poll()['events'],[])

    def test_failed_operation_and_read_only_call_do_not_emit_success(self):
        runtime,display = self.make_runtime()
        module = object()
        runtime.connection.poi.add.side_effect = ValueError('creation rejected')
        with self.assertRaises(ValueError):
            runtime.handle(dict(action='call',method='poi.add',args=[ENTITY_ID]),module)
        runtime.handle(dict(action='call',method='poi.getIDList'),module)
        self.assertEqual(runtime.events.poll()['events'],[])
        display.assert_not_called()

    def test_pending_events_survive_bridge_disconnect_and_other_clients_cannot_ack(self):
        runtime,display = self.make_runtime()
        module,first,second = object(),object(),object()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),first)
        runtime.handle(dict(action='call',method='vehicle.add',args=[ENTITY_ID]),module)
        with self.assertRaises(RuntimeError):
            runtime.handle(dict(action='events.poll'),module)
        ids=[e['id'] for e in runtime.events.poll()['events']]
        with self.assertRaises(RuntimeError):
            runtime.handle(dict(action='events.ack',ids=ids),module)
        runtime.disconnected(first)
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),second)
        self.assertEqual([e['id'] for e in runtime.handle(dict(action='events.poll'),second)['events']],ids)
        runtime.handle(dict(action='events.ack',ids=ids),second)
        display.assert_called_once()

    def test_bridge_displays_received_message_before_returning_ack(self):
        adapter=SumoAdapter.__new__(SumoAdapter)
        adapter.client=MagicMock()
        adapter.on_operation=MagicMock()
        adapter.client.poll_events.return_value=dict(events=[dict(id='event-1',message='created')],dropped=0)
        order=MagicMock()
        order.attach_mock(adapter.on_operation,'display')
        order.attach_mock(adapter.client.acknowledge_events,'ack')
        adapter.relay_events()
        self.assertEqual([c[0] for c in order.mock_calls],['display','ack'])
        adapter.client.acknowledge_events.assert_called_once_with(['event-1'],0)

    def test_bounded_queue_and_batches_report_lost_notifications(self):
        display=MagicMock()
        events=OperationEvents(display)
        with patch('foundation.sumo.events.EVENT_CAPACITY',2),patch('foundation.sumo.events.EVENT_BATCH_SIZE',1):
            for entity in ('a','b','c'):events.record_success('poi.add',[entity])
            pending=events.poll()
            self.assertEqual(len(pending['events']),1)
            self.assertEqual(pending['events'][0]['entity'],'b')
            self.assertEqual(pending['dropped'],1)
            events.acknowledge([pending['events'][0]['id']],pending['dropped'])
            self.assertEqual(events.poll()['dropped'],0)
            self.assertEqual(events.poll()['events'][0]['entity'],'c')


if __name__ == '__main__':
    unittest.main()
