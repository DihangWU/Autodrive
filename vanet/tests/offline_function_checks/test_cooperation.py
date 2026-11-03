"""无需启动仿真器，验证无线因果、跨层判断、租期与回补行为。"""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.npc.route_conflict_assessment.route_geometry import conflict
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController,load_scenario


def message(kind='emergency',time=0.,sequence=1,version=1,source='lincoln0',**fields):
    payload=dict(type=kind,task_id='task-a',route_version=version,sequence=sequence,sent_at=time,source=source)
    if kind=='emergency':
        payload.update(route=[dict(x=0,y=0,z=0),dict(x=200,y=0,z=0)],position=dict(x=0,y=0,z=0),speed=10.)
    payload.update(fields)
    return payload


def state(**fields):
    result=dict(id='npc0',x=30.,y=0.,z=0.,heading=0.,speed=5.,width=1.8,lane_id='edge_0')
    result.update(fields)
    return result


def connection():
    c=SimpleNamespace(vehicle=Mock(),edge=Mock(),lane=Mock(),simulation=Mock(),trafficlight=Mock(),route=Mock(),vehicletype=Mock())
    c.vehicle.getSpeedMode.return_value=31
    c.vehicle.getLaneChangeMode.return_value=1621
    c.vehicle.getMaxSpeed.return_value=13.89
    c.vehicle.getLaneIndex.return_value=0
    c.vehicle.getRoadID.return_value='edge'
    c.vehicle.getLaneID.return_value='edge_0'
    c.vehicle.getLateralLanePosition.return_value=0.
    c.vehicle.getIDList.return_value=[]
    c.edge.getLaneNumber.return_value=2
    c.lane.getWidth.return_value=3.5
    c.vehicle.couldChangeLane.return_value=True
    c.vehicletype.getIDList.return_value=[]
    c.route.getIDList.return_value=[]
    c.simulation.getPendingVehicles.return_value=[]
    c.simulation.getTime.return_value=20.
    c.trafficlight.getIDList.return_value=['tl0']
    c.trafficlight.getProgram.return_value='0'
    c.trafficlight.getPhase.return_value=2
    c.trafficlight.getNextSwitch.return_value=27.
    c.trafficlight.getRedYellowGreenState.return_value='Gr'
    return c


class NpcTests(unittest.TestCase):
    def test_no_wireless_message_means_no_cooperative_action(self):
        npc=NpcCooperation('npc0',{});c=connection()
        output=npc.step(0.,state(),c)
        self.assertEqual([v['payload']['type'] for v in output],['beacon'])
        self.assertEqual(c.vehicle.mock_calls,[])

    def test_actual_receive_emits_own_feedback_and_never_forwards_emergency(self):
        npc=NpcCooperation('npc0',{});c=connection()
        self.assertTrue(npc.receive(message(),0.))
        output=npc.step(0.,state(),c)
        self.assertEqual([p['payload']['status'] for p in output if p['payload']['type']=='feedback'],['received','yielding'])
        self.assertTrue(all(p['payload']['type'] in ('beacon','feedback') for p in output))
        c.vehicle.changeLane.assert_called_once_with('npc0',1,3.)

    def test_duplicate_does_not_renew_lease(self):
        npc=NpcCooperation('npc0',{});m=message()
        self.assertTrue(npc.receive(m,0.));self.assertFalse(npc.receive(m,5.))
        self.assertEqual(npc.lease_until,10.)
        c=connection();npc.step(0.,state(),c);npc.step(10.,state(),c)
        self.assertEqual(npc.status,'recovering')
        npc.step(12.1,state(),c)
        self.assertEqual(npc.status,'normal')
        c.vehicle.setSpeed.assert_called_with('npc0',-1.)

    def test_old_cancel_cannot_poison_new_route(self):
        npc=NpcCooperation('npc0',{})
        self.assertTrue(npc.receive(message(version=2),0.))
        self.assertFalse(npc.receive(message('cancel',version=1,sequence=2),0.))
        self.assertTrue(npc.receive(message(version=2,sequence=3,time=1.),1.))
        self.assertFalse(npc.recovering)

    def test_cancel_prevents_delayed_task_resurrection(self):
        npc=NpcCooperation('npc0',{})
        npc.receive(message(),0.)
        npc.receive(message('cancel',time=1.,sequence=2),1.)
        self.assertFalse(npc.receive(message(time=2.,sequence=3),2.))

    def test_crossing_vehicle_holds_until_lincoln_passes(self):
        npc=NpcCooperation('npc0',{});c=connection()
        npc.receive(message(),0.)
        npc.step(0.,state(x=30,y=10,heading=-90),c)
        self.assertEqual(npc.actions.action,'wait_before_conflict')
        npc.step(1.,state(x=30,y=8,heading=-90,speed=0),c)
        self.assertEqual(npc.reason,'holding_until_emergency_passes')
        c.vehicle.setSpeed.assert_called_with('npc0',0.)
        npc.receive(message(time=2.,sequence=2,position=dict(x=50,y=0,z=0)),2.)
        npc.step(2.,state(x=30,y=8,heading=-90,speed=0),c)
        self.assertTrue(npc.recovering)

    def test_junction_vehicle_does_not_stop_inside_conflict(self):
        npc=NpcCooperation('npc0',{});c=connection();npc.receive(message(),0.)
        npc.step(0.,state(in_junction=True),c)
        self.assertEqual(npc.reason,'clear_junction_first')
        c.vehicle.slowDown.assert_not_called()

    def test_overpass_is_not_route_conflict(self):
        self.assertFalse(conflict(state(z=8.),message(),{})['relevant'])

    def test_simulation_period_does_not_depend_on_call_count(self):
        npc=NpcCooperation('npc0',{});c=connection()
        self.assertEqual(len(npc.step(0.,state(),c)),1)
        for _ in range(10):self.assertEqual(npc.step(.2,state(),c),[])
        self.assertEqual(len(npc.step(1.,state(),c)),1)


class RsuTests(unittest.TestCase):
    def test_no_received_beacon_means_no_npc_notice(self):
        rsu=RsuCooperation({});rsu.receive(message(),0.)
        self.assertFalse(any(p['payload']['type']=='notice' for p in rsu.step(0.)))

    def test_notice_is_targeted_and_missing_feedback_is_backfilled(self):
        rsu=RsuCooperation({});rsu.receive(message(),0.)
        rsu.receive(message('beacon',source='npc0',state=state()),0.)
        first=[p for p in rsu.step(0.) if p['payload']['type']=='notice']
        self.assertEqual(first[0]['recipient'],'npc0');self.assertFalse(first[0]['payload']['backfill'])
        later=[p for p in rsu.step(1.) if p['payload']['type']=='notice']
        self.assertTrue(later[0]['payload']['backfill'])

    def test_stale_beacons_do_not_keep_vehicle_selected(self):
        rsu=RsuCooperation({});rsu.receive(message(),0.)
        rsu.receive(message('beacon',source='npc0',state=state()),0.)
        rsu.step(0.)
        outputs=rsu.step(4.)
        self.assertEqual(rsu.selected,{})
        self.assertTrue(any(p['payload']['type']=='cancel' for p in outputs))

    def test_stopped_crossing_vehicle_keeps_lease_until_emergency_passes(self):
        rsu=RsuCooperation({});rsu.receive(message(),0.)
        rsu.receive(message('beacon',source='npc0',state=state(x=30,y=10,heading=-90)),0.)
        rsu.step(0.)
        rsu.receive(message('beacon',source='npc0',time=1.,sequence=2,state=state(x=30,y=8,heading=-90,speed=0)),1.)
        output=rsu.step(1.)
        self.assertIn('npc0',rsu.selected)
        self.assertFalse(any(m['payload']['type']=='cancel' for m in output))
        rsu.receive(message(time=2.,sequence=2,position=dict(x=50,y=0,z=0)),2.)
        output=rsu.step(2.)
        self.assertNotIn('npc0',rsu.selected)
        self.assertTrue(any(m['payload']['type']=='cancel' for m in output))

    def test_feedback_only_counts_current_selected_task(self):
        rsu=RsuCooperation({});rsu.receive(message(version=2),0.)
        rsu.receive(message('beacon',version=0,source='npc0',state=state()),0.)
        rsu.step(0.)
        self.assertFalse(rsu.receive(message('feedback',version=1,source='npc0',status='cleared'),0.))
        self.assertTrue(rsu.receive(message('feedback',version=2,source='npc0',status='cleared'),0.))
        summary=next(p['payload'] for p in rsu.step(1.) if p['payload']['type']=='clearance')
        self.assertTrue(summary['all_cleared'])


class ScenarioTests(unittest.TestCase):
    def test_reset_requires_explicit_emergency_cancel(self):
        c=connection();s=ScenarioController(c)
        with self.assertRaises(RuntimeError):s.reset(emergency_active=True)
        c.vehicle.remove.assert_not_called()

    def test_reset_uses_configured_light_phase_independent_of_startup_delay(self):
        c=connection();s=ScenarioController(c);s.reset()
        c.trafficlight.getPhase.return_value=5
        s.reset()
        c.trafficlight.setPhase.assert_called_with('tl0',0)
        c.trafficlight.setPhaseDuration.assert_called_with('tl0',42.)
        c.trafficlight.getNextSwitch.return_value=101.
        ScenarioController(c).reset()
        c.trafficlight.setPhase.assert_called_with('tl0',0)
        c.trafficlight.setPhaseDuration.assert_called_with('tl0',42.)

    def test_fixed_spawn_waits_for_simulated_depart_offset(self):
        c=connection();s=ScenarioController(c);s.reset()
        s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,2)
        s.start(100.);s.step(134.);s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,2)
        s.step(135.);s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,3)
        s.step(138.);s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,4)
        s.step(151.95);s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,4)
        s.step(152.);s.traffic.spawn_next_vehicles(20)
        self.assertEqual(s.traffic.next_index,5)
        routes=[call.args[1] for call in c.route.add.call_args_list]
        self.assertEqual(routes[0],load_scenario()['npcs'][0]['route'])
        c.vehicle.setRoute.assert_not_called()

    def test_scenario_npcs_are_not_additional_lincolns(self):
        self.assertEqual(load_scenario()['npc_vehicle_type'],'vehicle.audi.a2')


if __name__=='__main__':unittest.main()
