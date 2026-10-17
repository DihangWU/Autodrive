"""A bridge may control SUMO time, but never owns SUMO's process lifetime."""
import unittest
from unittest.mock import MagicMock
from foundation.sumo.runtime import SumoRuntime

STEP_LENGTH = .05
CONFIG = 'test.sumocfg'


class SumoRuntimeTests(unittest.TestCase):
    def make_runtime(self):
        connection = MagicMock()
        connection.simulation.getTime.return_value = 0
        return SumoRuntime(connection, CONFIG, STEP_LENGTH)

    def test_autonomous_then_claim_then_disconnect_resumes(self):
        runtime = self.make_runtime()
        owner = object()
        runtime.autonomous_step()
        runtime.connection.simulationStep.assert_called_once()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        runtime.next_step = 0
        runtime.autonomous_step()
        self.assertEqual(runtime.connection.simulationStep.call_count,1)
        runtime.handle(dict(action='call',method='simulationStep'),owner)
        self.assertEqual(runtime.connection.simulationStep.call_count,2)
        runtime.disconnected(owner)
        runtime.autonomous_step()
        self.assertEqual(runtime.connection.simulationStep.call_count,3)
        runtime.connection.close.assert_not_called()

    def test_release_and_remote_close_protection(self):
        runtime = self.make_runtime()
        owner, other = object(), object()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        with self.assertRaises(RuntimeError):
            runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),other)
        with self.assertRaises(ValueError):
            runtime.handle(dict(action='call',method='close'),owner)
        runtime.handle(dict(action='clock.release'),owner)
        runtime.connection.close.assert_not_called()
        runtime.autonomous_step()
        runtime.connection.simulationStep.assert_called_once()

    def test_generic_domain_call_needs_no_business_dispatch(self):
        runtime = self.make_runtime()
        owner = object()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        runtime.handle(dict(action='call',method='poi.add',args=['rsu0',1,2,[255,0,0,255]],kwargs={'layer':100}),owner)
        runtime.connection.poi.add.assert_called_once_with('rsu0',1,2,[255,0,0,255],layer=100)
        with self.assertRaises(ValueError):
            runtime.handle(dict(action='call',method='_socket.close'),owner)

    def test_entity_client_can_work_without_claiming_clock(self):
        runtime = self.make_runtime()
        owner, entity_client = object(), object()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        runtime.handle(dict(action='call',method='poi.getIDList'),entity_client)
        runtime.connection.poi.getIDList.assert_called_once()
        with self.assertRaises(RuntimeError):
            runtime.handle(dict(action='call',method='simulationStep'),entity_client)
        runtime.disconnected(entity_client)
        self.assertIs(runtime.clock_owner,owner)
        runtime.connection.close.assert_not_called()

    def test_clock_mode_notification_only_on_transition(self):
        changed = MagicMock()
        connection = MagicMock()
        connection.simulation.getTime.return_value = 0
        runtime = SumoRuntime(connection,CONFIG,STEP_LENGTH,on_clock_change=changed)
        owner, other = object(),object()
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        runtime.handle(dict(action='clock.claim',step_length=STEP_LENGTH),owner)
        runtime.disconnected(other)
        changed.assert_called_once_with(True)
        runtime.disconnected(owner)
        self.assertEqual([c.args for c in changed.call_args_list],[(True,),(False,)])
        runtime.handle(dict(action='clock.release'),owner)
        self.assertEqual(changed.call_count,2)


if __name__ == '__main__':
    unittest.main()
