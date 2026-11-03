"""车辆视觉状态去重、复位，以及 RSU 终端状态轮询检查。"""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from vanet.sumo.vehicle_state_display.vehicle_state_display import VehicleStateDisplay,NPC_COLORS,EMERGENCY_COLOR
from vanet.rsu.terminal_status_display.status_reader import RsuTerminalStatus,summarize_status


class VehicleStateDisplayTests(unittest.TestCase):
    def setUp(self):
        self.connection=SimpleNamespace(vehicle=Mock())
        self.vehicle=self.connection.vehicle
        self.vehicle.getColor.return_value=(10,20,30,255)
        self.vehicle.getParameter.return_value=''
        self.display=VehicleStateDisplay(self.connection)

    def test_repeated_status_does_not_repeat_rpc(self):
        self.display.npc('npc0','yielding','lane change','task')
        color_count=self.vehicle.setColor.call_count
        parameter_count=self.vehicle.setParameter.call_count
        for _ in range(20):self.display.npc('npc0','yielding','lane change','task')
        self.assertEqual(self.vehicle.setColor.call_count,color_count)
        self.assertEqual(self.vehicle.setParameter.call_count,parameter_count)
        self.vehicle.setColor.assert_called_once_with('npc0',NPC_COLORS['yielding'])

    def test_recovery_to_normal_restores_original_color(self):
        self.display.npc('npc0','unable','blocked','task')
        self.display.npc('npc0','normal','','')
        self.vehicle.setColor.assert_called_with('npc0',(10,20,30,255))

    def test_emergency_cancel_restores_lincoln_original_color(self):
        self.display.lincoln('lincoln0',True,'manual')
        self.vehicle.setColor.assert_called_with('lincoln0',EMERGENCY_COLOR)
        self.display.lincoln('lincoln0',False,'idle')
        self.vehicle.setColor.assert_called_with('lincoln0',(10,20,30,255))

    def test_close_restores_original_parameter_and_ignores_removed_vehicle(self):
        self.vehicle.getParameter.side_effect=lambda name,key:'old' if key=='vanet.state' else ''
        self.display.npc('npc0','cleared','ok','task')
        self.display.npc('gone','yielding')
        self.display.close(['npc0'])
        self.vehicle.setParameter.assert_any_call('npc0','vanet.state','old')
        self.assertEqual(self.display.saved,{})
        self.assertFalse(any(call.args[0]=='gone' and call.args[1]==(10,20,30,255) for call in self.vehicle.setColor.call_args_list))

    def test_reused_vehicle_id_gets_new_original_color(self):
        self.display.npc('npc0','yielding');self.display.prune([])
        self.vehicle.getColor.return_value=(80,90,100,255)
        self.display.npc('npc0','yielding');self.display.npc('npc0','normal')
        self.vehicle.setColor.assert_called_with('npc0',(80,90,100,255))


class RsuStatusTests(unittest.TestCase):
    def status(self):
        return dict(phase='running',group='cooperative',simulation_time=10.,
                    rsu=dict(id='rsu0',beacon_count=2,selected=['npc0'],feedback={'npc0':dict(status='yielding')},
                             counters=dict(received=3,notice_sent=2,feedback_received=1,clearance_sent=1,backfill_sent=0)),
                    npc_states={'npc0':dict(status='yielding'),'npc1':dict(status='normal')})

    def test_time_alone_does_not_print_duplicate_status(self):
        console=Mock();client=Mock();first=self.status();second=copy.deepcopy(first);second['simulation_time']=11.
        client.command.side_effect=[first,second]
        reader=RsuTerminalStatus(console,'socket',client_factory=Mock(return_value=client))
        reader.poll_once();reader.poll_once();reader.close()
        self.assertEqual(console.info.call_count,1)
        self.assertIn('收包=3',console.info.call_args.args[0])
        client.command.assert_called_with('vanet.status')
        client.close.assert_called_once()

    def test_counter_change_is_printed(self):
        console=Mock();client=Mock();first=self.status();second=copy.deepcopy(first)
        second['rsu']['counters']['received']=4
        client.command.side_effect=[first,second]
        reader=RsuTerminalStatus(console,'socket',client_factory=Mock(return_value=client))
        reader.poll_once();reader.poll_once();reader.close()
        self.assertEqual(console.info.call_count,2)

    def test_reconnect_logs_failure_once_and_then_recovery(self):
        console=Mock();client=Mock();client.command.return_value=self.status()
        factory=Mock(side_effect=[ConnectionError('down'),ConnectionError('down'),client])
        reader=RsuTerminalStatus(console,'socket',client_factory=factory)
        reader.poll_once();reader.poll_once();reader.poll_once();reader.close()
        self.assertEqual(console.info.call_count,3)
        self.assertIn('已恢复',console.info.call_args_list[1].args[0])

    def test_optional_fields_are_compatible(self):
        summary=summarize_status(dict(phase='waiting'))
        self.assertEqual(summary['received'],0)
        self.assertEqual(summary['npc_counts'],{})


if __name__=='__main__':unittest.main()
