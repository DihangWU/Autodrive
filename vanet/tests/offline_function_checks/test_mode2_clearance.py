"""两种公开模式、收包因果、只向左让行及匝道单一速度控制。"""
import json
from types import SimpleNamespace
import unittest
from vanet.project_paths import EXPERIMENT_DEFAULT_CONFIGURATION_FILE
from vanet.protocol.message_validation.messages import GROUPS,experiment_group
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.lincoln.v2x_data_exchange.wireless_clearance_advice import WirelessClearanceAdvice
from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.highway_ramp_merging.merge_controller import RampMergeController
from vanet.tests.offline_function_checks.test_cooperation import message,state,connection as ordinary_connection

def connection():
    c=ordinary_connection()
    c.vehicle.getMaxSpeedLat.return_value=1.
    c.vehicle.getParameter.return_value='1'
    c.vehicle.getLaneIndex.return_value=1
    c.edge.getLaneNumber.return_value=5
    c.vehicle.getSpeed.return_value=5.
    c.vehicle.getLateralSpeed.return_value=0.
    c.vehicle.getDecel.return_value=4.5
    c.vehicle.getMinGap.return_value=2.5
    c.vehicle.getLeftLeaders.return_value=[]
    c.vehicle.getLeftFollowers.return_value=[]
    c.vehicle.getSecureGap.return_value=5.
    return c


TC=SimpleNamespace(VAR_ROAD_ID=80,VAR_LANE_ID=81,VAR_LANEPOSITION=86,VAR_SPEED=64,VAR_LANEPOSITION_LAT=184)


def config(role='same_direction'):
    result=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    result.update(experiment_mode='mode2',scenario_role=role,yielding_enabled=True)
    result['mode2']['ramp_merge_point']=dict(x=50.,y=0.,z=0.)
    return result


def component(configuration=None):
    npc=NpcCooperation('npc0',configuration or config())
    npc.set_communication_links(['lincoln0'],0.);npc.set_communication_path('direct',0.)
    return npc


class Mode2ClearanceTests(unittest.TestCase):
    def test_only_mode1_and_mode2_are_selectable(self):
        self.assertEqual(GROUPS,('mode1','mode2'))
        for mode in GROUPS:self.assertEqual(experiment_group(mode),mode)
        for mode in ('normal','emergency','cooperative'):
            with self.assertRaises(ValueError):experiment_group(mode)

    def test_mode2_moving_front_feedback_does_not_force_lincoln_to_slow_before_yield_distance(self):
        advice=WirelessClearanceAdvice()
        samples=[SimpleNamespace(center=(0.,0.,0.)),SimpleNamespace(center=(200.,0.,0.))]
        payload=message('feedback',source='npc0',status='received',conflict_point=dict(x=40.,y=0.,z=0.))
        self.assertTrue(advice.receive(payload,0.,'task-a',1))
        self.assertTrue(advice.evaluate(0.,'task-a',1,(0.,0.,0.),samples,[0.,200.],0.)['active'])
        payload.update(sequence=2,experiment_mode='mode2',scenario_role='same_direction')
        self.assertTrue(advice.receive(payload,0.,'task-a',1))
        self.assertFalse(advice.evaluate(0.,'task-a',1,(0.,0.,0.),samples,[0.,200.],0.)['active'])
        self.assertEqual(advice.direct['npc0'].message['vehicles']['npc0'],'received')
        payload.update(sequence=3,status='unable')
        self.assertTrue(advice.receive(payload,0.,'task-a',1))
        self.assertTrue(advice.evaluate(0.,'task-a',1,(0.,0.,0.),samples,[0.,200.],0.)['active'])

    def test_mode2_relay_summary_keeps_real_status_without_a_false_speed_limit(self):
        advice=WirelessClearanceAdvice()
        samples=[SimpleNamespace(center=(0.,0.,0.)),SimpleNamespace(center=(200.,0.,0.))]
        payload=message('clearance',source='rsu0',experiment_mode='mode2',vehicles={'npc0':'yielding'},
            conflict_points={'npc0':dict(x=40.,y=0.,z=0.)},informational_vehicles=['npc0'])
        self.assertTrue(advice.receive(payload,0.,'task-a',1))
        self.assertFalse(advice.evaluate(0.,'task-a',1,(0.,0.,0.),samples,[0.,200.],0.)['active'])
        self.assertEqual(advice.message['vehicles']['npc0'],'yielding')

    def test_range_membership_and_shared_pose_without_reception_never_cause_yield(self):
        npc=component();c=connection();npc.set_emergency_vehicle_state(dict(x=20.,y=0.,z=0.))
        npc.step(0.,state(),c)
        c.vehicle.changeLane.assert_not_called();c.vehicle.slowDown.assert_not_called()

    def test_front_car_receives_at_fifty_and_requests_left_at_forty_five(self):
        npc=component();c=connection();c.vehicle.getLaneIndex.return_value=1;c.edge.getLaneNumber.return_value=5
        self.assertTrue(npc.receive(message(),0.));npc.step(0.,state(x=50.),c)
        c.vehicle.changeSublane.assert_not_called()
        npc.receive(message(time=.1,sequence=2,position=dict(x=4.,y=0.,z=0.)),.1)
        npc.step(.1,state(x=50.),c)
        c.vehicle.changeSublane.assert_not_called()
        npc.receive(message(time=.2,sequence=3,position=dict(x=5.,y=0.,z=0.)),.2)
        npc.step(.2,state(x=50.),c)
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
        c.vehicle.setLaneChangeMode.assert_called_with('npc0',1280)
        c.vehicle.slowDown.assert_not_called();c.vehicle.changeLane.assert_not_called()

    def test_stale_blocked_result_does_not_suppress_native_safe_gap_negotiation(self):
        npc=component();c=connection();c.vehicle.couldChangeLane.return_value=False
        npc.receive(message(),0.);npc.step(0.,state(),c)
        self.assertEqual(npc.reason,'left_lane_requested')
        c.vehicle.couldChangeLane.assert_not_called()
        c.vehicle.setLaneChangeMode.assert_called_with('npc0',1280)
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
        c.vehicle.setSpeed.assert_not_called();c.vehicle.slowDown.assert_not_called()

    def test_third_lane_keeps_driving_without_a_cooperative_lane_change(self):
        npc=component();c=connection();c.vehicle.getLaneIndex.return_value=2
        npc.receive(message(),0.);npc.step(0.,state(y=3.5),c)
        self.assertEqual(npc.reason,'outside_lane4_keep_normal_driving')
        c.vehicle.changeSublane.assert_not_called();c.vehicle.setLaneChangeMode.assert_not_called()
        c.vehicle.setSpeed.assert_not_called();c.vehicle.slowDown.assert_not_called()

    def test_nonblocking_adjacent_car_does_not_report_an_obstruction_when_left_gap_is_blocked(self):
        npc=component();c=connection();c.vehicle.getLaneIndex.return_value=2
        npc.receive(message(),0.);npc.step(0.,state(y=3.5),c)
        self.assertEqual(npc.status,'cleared')
        c.vehicle.setSpeed.assert_not_called();c.vehicle.slowDown.assert_not_called()

    def test_rsu_relays_mode2_to_ramp_even_when_it_is_waiting_off_the_route(self):
        rsu=RsuCooperation(config())
        rsu.set_communication_links(['lincoln0','npc0'],0.);rsu.set_relay_targets(['npc0'],0.)
        self.assertTrue(rsu.receive(message(experiment_mode='mode2'),0.))
        self.assertTrue(rsu.receive(message('beacon',source='npc0',state=state(x=50.,y=30.,speed=0.,scenario_role='ramp_merging')),0.))
        notices=[item for item in rsu.step(0.) if item['payload']['type']=='notice']
        self.assertEqual([item['recipient'] for item in notices],['npc0'])
        self.assertEqual(notices[0]['payload']['experiment_mode'],'mode2')

    def test_left_yield_is_kept_until_lincoln_actually_passes_then_restored(self):
        npc=component();c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        npc.set_emergency_vehicle_state(dict(x=20.,y=0.,z=0.))
        c.vehicle.getLaneIndex.return_value=2
        npc.step(.1,state(y=3.5),c)
        self.assertEqual(npc.status,'cleared');self.assertFalse(npc.recovering)
        npc.set_emergency_vehicle_state(dict(x=50.,y=0.,z=0.))
        npc.step(.2,state(y=3.5),c);npc.step(2.3,state(y=3.5),c)
        self.assertEqual(npc.status,'normal')
        c.vehicle.setLaneChangeMode.assert_called_with('npc0',1621)
        c.vehicle.setMaxSpeedLat.assert_called_with('npc0',1.)
        c.vehicle.setParameter.assert_any_call('npc0','laneChangeModel.lcAccelLat','1')
        c.vehicle.setParameter.assert_any_call('npc0','laneChangeModel.lcAssertive','1')

    def test_centering_requests_continue_after_the_passing_space_is_clear(self):
        npc=component();c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        c.vehicle.getLaneIndex.return_value=2;c.vehicle.getLateralLanePosition.return_value=-.7
        npc.step(.2,state(y=2.8),c)
        c.vehicle.changeSublane.assert_called_with('npc0',.7)
        self.assertEqual(c.vehicle.changeSublane.call_count,2)
        self.assertEqual(npc.status,'cleared')
        self.assertFalse(npc.actions.left_maneuver.centred)

    def test_passing_space_is_reported_without_waiting_for_the_body_to_straighten(self):
        npc=component();c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        c.vehicle.getLaneIndex.return_value=2;c.vehicle.getLateralLanePosition.return_value=-.4
        c.vehicle.getLateralSpeed.return_value=1.3
        npc.step(.2,state(y=2.8,heading=20.),c)
        self.assertEqual(npc.status,'yielding')
        npc.step(.4,state(y=3.1,heading=20.),c)
        self.assertEqual(npc.status,'cleared')
        self.assertFalse(npc.actions.left_maneuver.centred)
        c.vehicle.changeSublane.assert_called_with('npc0',.4)

    def test_previously_merged_ramp_vehicle_yields_left_as_a_main_road_front_car(self):
        c,npc,controller,created,states=self.ramp_fixture()
        controller.completed.add('npc0')
        c.vehicle.getLaneIndex.return_value=1;c.edge.getLaneNumber.return_value=5
        npc.receive(message(),0.);npc.step(0.,state(road_id='-47',lane_id='-47_1'),c)
        self.assertFalse(controller.emergency_hold.requested)
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
        self.assertEqual(npc.current_role,'same_direction')

    def test_leaving_the_ramp_releases_old_hold_before_main_road_left_yield(self):
        c,npc,controller,created,states=self.ramp_fixture()
        npc.receive(message(),0.);npc.step(0.,state(x=40.,y=20.,road_id='31'),c)
        self.assertIn('npc0',controller.emergency_hold.requested)
        controller.completed.add('npc0')
        c.vehicle.getLaneIndex.return_value=1;c.edge.getLaneNumber.return_value=5
        npc.step(.1,state(road_id='-47',lane_id='-47_1'),c)
        self.assertNotIn('npc0',controller.emergency_hold.requested)
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)

    def test_occupied_space_waits_but_any_positive_native_gap_is_used(self):
        for side in ('front','rear'):
            with self.subTest(side=side):
                npc=component();c=connection()
                neighbors=c.vehicle.getLeftLeaders if side=='front' else c.vehicle.getLeftFollowers
                neighbors.return_value=[('other',-3.)]
                npc.receive(message(),0.);npc.step(0.,state(),c)
                self.assertEqual(npc.reason,'waiting_for_unoccupied_lane_space')
                c.vehicle.changeSublane.assert_not_called()
                neighbors.return_value=[('other',.1)]
                npc.step(.2,state(),c)
                c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
                c.vehicle.getSecureGap.assert_not_called()

    def test_native_minimum_gap_is_not_counted_as_vehicle_body(self):
        c=connection();c.vehicle.getLeftFollowers.return_value=[('rear',2.5794)]
        c.vehicle.getSpeed.side_effect=lambda name:18.0199 if name=='rear' else 14.
        npc=component();npc.receive(message(),0.);npc.step(0.,state(),c)
        self.assertEqual(npc.reason,'left_lane_requested')
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
        call=c.vehicle.slowDown.call_args
        self.assertEqual(call.args[:2],('rear',14.))
        self.assertAlmostEqual(call.args[2],4.0199/4.5)
        npc.step(.2,state(),c)
        self.assertEqual(c.vehicle.slowDown.call_count,1)

    def test_large_following_gap_does_not_veto_available_space(self):
        c=connection();c.vehicle.getLeftFollowers.return_value=[('rear',3.)]
        c.vehicle.getSpeed.return_value=20.
        c.vehicle.getSecureGap.return_value=100.
        npc=component();npc.receive(message(),0.);npc.step(0.,state(),c)
        self.assertEqual(npc.reason,'left_lane_requested')
        c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)
        c.vehicle.setLaneChangeMode.assert_called_with('npc0',1280)
        c.vehicle.getSecureGap.assert_not_called()

    def test_rear_assistance_never_controls_lincoln_or_non_network_background(self):
        for other in ('lincoln0','background_highway_001'):
            with self.subTest(vehicle=other):
                c=connection();c.vehicle.getLeftFollowers.return_value=[(other,2.5794)]
                c.vehicle.getSpeed.side_effect=lambda name:18.0199 if name==other else 14.
                npc=component();npc.receive(message(),0.);npc.step(0.,state(),c)
                c.vehicle.slowDown.assert_not_called()
                c.vehicle.setSpeed.assert_not_called()

    def test_only_an_actual_collision_risk_blocks_a_fast_rear(self):
        c=connection();c.vehicle.getLeftFollowers.return_value=[('rear',10.)]
        c.vehicle.getSpeed.side_effect=lambda name:25. if name=='rear' else 15.
        npc=component();npc.receive(message(),0.);npc.step(0.,state(),c)
        self.assertEqual(npc.reason,'waiting_for_unoccupied_lane_space')
        c.vehicle.changeSublane.assert_not_called()
        c.vehicle.getSecureGap.assert_not_called()

    def test_gap_closure_never_requests_a_retreat(self):
        npc=component();c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        c.vehicle.getLateralLanePosition.return_value=1.
        c.vehicle.getLeftFollowers.return_value=[('other',-1.)]
        npc.step(.2,state(),c)
        c.vehicle.changeSublane.assert_called_with('npc0',2.5)
        self.assertEqual(npc.status,'yielding')

    def test_internal_edge_remapping_preserves_left_counted_third_lane(self):
        npc=component();c=connection();npc.receive(message(),0.);npc.step(0.,state(),c)
        c.edge.getLaneNumber.return_value=4;c.vehicle.getLaneIndex.return_value=0
        c.vehicle.getRoadID.return_value=':134_10';c.vehicle.getLateralLanePosition.return_value=.8
        npc.step(.2,state(road_id=':134_10',in_junction=True),c)
        c.vehicle.changeSublane.assert_called_with('npc0',2.7)
        self.assertEqual(npc.actions.left_maneuver.target_lane,1)
        c.vehicle.getLaneIndex.return_value=1;c.vehicle.getLateralLanePosition.return_value=0.
        npc.step(.3,state(y=3.5,road_id=':134_10',in_junction=True),c)
        self.assertTrue(npc.actions.left_maneuver.centred)
        requests=c.vehicle.changeSublane.call_count
        c.edge.getLaneNumber.return_value=5;c.vehicle.getLaneIndex.return_value=2
        c.vehicle.getRoadID.return_value='next_edge'
        npc.step(.4,state(y=3.5,road_id='next_edge'),c)
        self.assertEqual(npc.actions.left_maneuver.target_lane,2)
        self.assertEqual(c.vehicle.changeSublane.call_count,requests)

    def test_internal_fourth_lane_has_no_junction_wait_rule(self):
        for road in (':1061_2',':1452_14'):
            with self.subTest(road=road):
                npc=component();c=connection();c.vehicle.getRoadID.return_value=road
                npc.receive(message(),0.);npc.step(0.,state(road_id=road,lane_id=road+'_1',in_junction=True),c)
                c.vehicle.changeSublane.assert_called_once_with('npc0',3.5)

    def ramp_fixture(self):
        c=connection();c.lane.getMaxSpeed.return_value=13.89
        c.vehicle.getTau.return_value=1.;c.vehicle.getMinGap.return_value=2.5
        vehicles,policy=load_merging_vehicles();entry=dict(vehicles[0],id='npc0')
        created={'npc0':dict(scenario=entry,spec=dict(length='3.7'))}
        controller=RampMergeController(c,policy);controller.start(created,0.)
        npc=component(dict(config('ramp_merging'),ramp_controller=controller))
        states={'npc0':{80:'31',81:'31_1',86:140.,64:5.,184:0.}}
        return c,npc,controller,created,states

    def test_ramp_brakes_without_meter_overwrite_and_resumes_after_passage(self):
        c,npc,controller,created,states=self.ramp_fixture()
        npc.receive(message(),0.);npc.step(0.,state(x=40.,y=20.),c)
        controller.update_lanes(created,states,TC,0.)
        c.vehicle.slowDown.assert_called_once_with('npc0',0.,1.25)
        c.vehicle.setSpeed.assert_not_called()
        states['npc0'][64]=0.;controller.update_lanes(created,states,TC,.2)
        npc.step(.2,state(x=40.,y=20.,speed=0.),c)
        self.assertEqual(npc.status,'cleared');self.assertIn('npc0',controller.emergency_hold.stopped)
        npc.set_emergency_vehicle_state(dict(x=45.,y=0.,z=0.))
        npc.set_communication_links([],1.);npc.step(40.,state(x=40.,y=20.,speed=0.),c)
        self.assertIn('npc0',controller.emergency_hold.requested)
        npc.set_emergency_vehicle_state(dict(x=65.,y=0.,z=0.))
        npc.step(40.1,state(x=40.,y=20.,speed=0.),c)
        controller.update_lanes(created,states,TC,40.1)
        self.assertNotIn('npc0',controller.emergency_hold.requested)
        self.assertEqual(npc.status,'normal')
        c.vehicle.setSpeed.assert_any_call('npc0',-1.)

    def test_explicit_cancel_releases_a_stopped_ramp_even_before_passage(self):
        c,npc,controller,created,states=self.ramp_fixture()
        npc.receive(message(),0.);npc.step(0.,state(x=40.,y=20.),c)
        controller.update_lanes(created,states,TC,0.)
        npc.receive(message('cancel',time=1.,sequence=2),1.)
        npc.step(1.,state(x=40.,y=20.),c)
        self.assertNotIn('npc0',controller.emergency_hold.requested)

    def test_operator_cancel_releases_an_out_of_range_ramp_and_rejects_delayed_task_packets(self):
        c,npc,controller,created,states=self.ramp_fixture()
        npc.receive(message(),0.);npc.step(0.,state(x=40.,y=20.),c)
        controller.update_lanes(created,states,TC,0.)
        npc.set_communication_links([],1.)
        npc.cancel_from_operator('task-a')
        npc.step(1.,state(x=40.,y=20.),c)
        self.assertNotIn('npc0',controller.emergency_hold.requested)
        npc.set_communication_links(['lincoln0'],2.)
        self.assertFalse(npc.receive(message(time=2.,sequence=2),2.))

    def test_committed_merge_is_cleared_without_an_impossible_abrupt_stop(self):
        c,npc,controller,created,states=self.ramp_fixture()
        states['npc0']={80:':1061_0',81:':1061_0_1',86:40.,64:18.,184:0.}
        controller.emergency_hold.request('npc0')
        self.assertEqual(controller.emergency_hold.update(states,TC,0.),set())
        self.assertEqual(controller.emergency_hold.status('npc0'),'committed')
        c.vehicle.slowDown.assert_not_called();c.vehicle.setSpeed.assert_not_called()
        controller.reset();self.assertFalse(controller.emergency_hold.requested)


if __name__=='__main__':unittest.main()
