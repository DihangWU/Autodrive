"""真实 Town04 完整车流及变道途中空隙关闭的回归，不连接用户联仿。"""
import json
import os
import unittest

from vanet.tests.live_cosimulation_checks import test_mode2_yielding as live_support
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController
from vanet.npc.mode2_route_clearance.yielding_actions import Mode2YieldingActions


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated SUMO checks only')
class Mode2DenseLaneChangeTests(unittest.TestCase):
    setUp=live_support.Mode2YieldingLiveTests.setUp
    tearDown=live_support.Mode2YieldingLiveTests.tearDown
    state=live_support.Mode2YieldingLiveTests.state
    load_straight_road=live_support.Mode2YieldingLiveTests.load_straight_road

    def assessment(self,now):
        return dict(kind='same_direction',station=40.,emergency_station=0.,distance=0.,sim_time=now)

    def test_front05_recorded_gap_is_used_with_short_rear_speed_assistance(self):
        self.load_straight_road();c=self.c;name='front05'
        for vehicle,lane,position,speed in ((name,1,200.,14.),('rear',2,190.8,18.01994469),('lead',2,224.,14.)):
            c.vehicle.add(vehicle,'front_route',departLane=str(lane),departPos=str(position),departSpeed=str(speed))
            c.vehicle.setLength(vehicle,3.7186);c.vehicle.setLaneChangeMode(vehicle,1536)
            c.vehicle.setSpeed(vehicle,speed);c.vehicle.setSpeedFactor(vehicle,1.);c.vehicle.setImperfection(vehicle,0.)
        c.vehicle.setTau(name,.45);c.vehicle.setMinGap(name,1.5)
        for _ in range(3):c.simulationStep()
        rear_gap=c.vehicle.getLeftFollowers(name)[0][1]
        self.assertAlmostEqual(rear_gap,2.5794,delta=.01)
        action=Mode2YieldingActions(name,self.config);completed=None;collisions=set();teleports=set();peak_braking=0.
        for index in range(80):
            now=index*.05;action.apply(c,self.state(name),self.assessment(now));c.simulationStep()
            peak_braking=max(peak_braking,-c.vehicle.getAcceleration('rear'))
            self.assertEqual(c.vehicle.getLaneIndex('rear'),2)
            collisions.update(c.simulation.getCollidingVehiclesIDList())
            teleports.update(c.simulation.getStartingTeleportIDList())
            if action.left_maneuver.centred and completed is None:completed=now
        self.assertIsNotNone(completed);self.assertLessEqual(completed,1.5+1e-8)
        self.assertIn('rear',action.left_maneuver.rear_assistance.requested)
        # slowDown requests the normal deceleration; native car following may
        # brake harder as the new leader enters the lane. Keep that protection.
        self.assertLessEqual(peak_braking,c.vehicle.getEmergencyDecel('rear')+.01)
        self.assertFalse(collisions);self.assertFalse(teleports)
        print(json.dumps(dict(scene='recorded_front05',native_rear_gap=rear_gap,
            physical_rear_gap=rear_gap+2.5,completed_seconds=completed,
            rear_peak_braking=peak_braking,collisions=sorted(collisions),teleports=sorted(teleports))))

    def test_available_space_below_eight_metres_completes_in_native_sumo(self):
        self.load_straight_road();c=self.c;name='yielding_front'
        for vehicle,lane,position in ((name,1,100),('lead',2,117.05),('rear',2,86.5)):
            c.vehicle.add(vehicle,'front_route',departLane=str(lane),departPos=str(position),departSpeed=str(80/3.6))
            c.vehicle.setLaneChangeMode(vehicle,1536);c.vehicle.setImperfection(vehicle,0.)
            c.vehicle.setSpeed(vehicle,80/3.6);c.vehicle.setSpeedFactor(vehicle,1.)
        c.vehicle.setTau(name,.45);c.vehicle.setMinGap(name,1.5)
        for _ in range(3):c.simulationStep()
        rear_gap=c.vehicle.getLeftFollowers(name)[0][1]
        self.assertGreater(rear_gap,0.);self.assertLess(rear_gap,8.)
        action=Mode2YieldingActions(name,self.config);completed=None;collisions=set();teleports=set()
        for index in range(80):
            now=index*.05;action.apply(c,self.state(name),self.assessment(now));c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList())
            teleports.update(c.simulation.getStartingTeleportIDList())
            if action.left_maneuver.centred:completed=now;break
        self.assertIsNotNone(completed);self.assertLessEqual(completed,1.5+1e-8)
        self.assertFalse(collisions);self.assertFalse(teleports)
        print(json.dumps(dict(scene='space_below_8m',rear_gap=rear_gap,completed_seconds=completed,
                              collisions=sorted(collisions),teleports=sorted(teleports))))

    def test_full_town04_traffic_completes_the_move_before_faster_rear_closes_the_gap(self):
        c=self.c;tc=self.tc;scene=ScenarioController(c)
        scene.reset();scene.traffic.spawn_next_vehicles(scene.traffic.count)
        for _ in range(3):c.simulationStep()
        fields=[tc.VAR_ROAD_ID,tc.VAR_EDGES,tc.VAR_LANEPOSITION,tc.VAR_LANE_ID,
                tc.VAR_LANEPOSITION_LAT,tc.VAR_POSITION,tc.VAR_SPEED]
        for name in c.vehicle.getIDList():c.vehicle.subscribe(name,fields)
        scene.start(0.)
        names={'vanet_npc_001':6.5,'vanet_npc_highway_lane4_01':15.05,
               'vanet_npc_highway_lane4_02':18.1,'vanet_npc_highway_lane4_03':21.2,
               'vanet_npc_highway_lane4_04':24.1,'vanet_npc_highway_lane4_05':26.1,
               'vanet_npc_highway_lane4_06':27.3}
        actions={};collisions=set();teleports=set();centred=False;finished={};motion_start={}
        stall=0.;max_stall=0.;peak_deceleration=0.
        for index in range(700):
            now=index*.05;states=c.vehicle.getAllSubscriptionResults()
            scene.step(now);scene.traffic.maintain_routes(states,tc)
            for name,start in names.items():
                if now<start or name not in states:continue
                if name not in actions:actions[name]=Mode2YieldingActions(name,self.config)
                actions[name].apply(c,self.state(name),self.assessment(now))
                lane=c.vehicle.getLaneIndex(name);lateral=c.vehicle.getLateralLanePosition(name)
                if abs(lateral)>.001:motion_start.setdefault(name,now-.05)
                if name=='vanet_npc_001':
                    centred|=lane==2 and abs(lateral)<=.2
                else:
                    # The old controller reached 1.57 m and stayed there from
                    # t=17 to t=34. The shorter move fits the real gap safely.
                    if abs(lateral)>.3 and abs(c.vehicle.getLateralSpeed(name))<.05:stall+=.05
                    else:stall=0.
                    max_stall=max(max_stall,stall)
                    if actions[name].left_maneuver.centred:
                        finished.setdefault(name,now-motion_start.get(name,start))
                        self.assertEqual(c.edge.getLaneNumber(c.vehicle.getRoadID(name))-lane,3,(now,name,lane))
            if 15.05<=now<18.:
                peak_deceleration=max(peak_deceleration,-c.vehicle.getAcceleration('vanet_npc_001'))
            c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList())
            teleports.update(c.simulation.getStartingTeleportIDList())
        print(json.dumps(dict(full_scene_motion_start=motion_start,full_scene_completed=finished)),flush=True)
        self.assertTrue(centred);self.assertEqual(set(finished),set(names)-{'vanet_npc_001'})
        for duration in finished.values():self.assertLessEqual(duration,1.5+1e-8)
        self.assertLess(max_stall,.5)
        self.assertLessEqual(peak_deceleration,c.vehicle.getDecel('vanet_npc_001')+.01)
        self.assertFalse(collisions);self.assertFalse(teleports)
        print(json.dumps(dict(scene='full_Town04',vehicles=len(scene.traffic.created),
            first_yield_centred=centred,yield_seconds=finished,
            maximum_midline_stall_seconds=max_stall,follower_peak_deceleration=peak_deceleration,
            collisions=sorted(collisions),teleports=sorted(teleports))))

    def test_accelerating_rear_does_not_trigger_a_retreat_or_collision(self):
        self.load_straight_road();c=self.c;name='yielding_front'
        for vehicle,position in ((name,100),('rear',82.5)):
            c.vehicle.add(vehicle,'front_route',departLane='1' if vehicle==name else '2',
                          departPos=str(position),departSpeed='15')
            c.vehicle.setLaneChangeMode(vehicle,1536);c.vehicle.setImperfection(vehicle,0.)
            c.vehicle.setSpeed(vehicle,15.);c.vehicle.setSpeedFactor(vehicle,1.)
        for _ in range(3):c.simulationStep()
        action=Mode2YieldingActions(name,self.config)
        started=False;complete=None;previous_y=c.vehicle.getPosition(name)[1]
        collisions=set();teleports=set();max_step=0.;min_step=0.
        for index in range(400):
            now=index*.05
            if index==4:
                c.vehicle.setAccel('rear',8.)
                c.vehicle.setSpeed('rear',25.)
            action.apply(c,self.state(name),self.assessment(now));c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList())
            teleports.update(c.simulation.getStartingTeleportIDList())
            lane=c.vehicle.getLaneIndex(name);lateral=c.vehicle.getLateralLanePosition(name)
            y=c.vehicle.getPosition(name)[1];max_step=max(max_step,abs(y-previous_y));min_step=min(min_step,y-previous_y);previous_y=y
            if lane==1 and lateral>.1:started=True
            if action.left_maneuver.centred:complete=now;break
        self.assertTrue(started);self.assertIsNotNone(complete);self.assertGreaterEqual(min_step,-.025)
        self.assertLessEqual(max_step,self.config['mode2']['lateral_speed_mps']*.05+.01)
        self.assertFalse(collisions);self.assertFalse(teleports)
        print(json.dumps(dict(scene='rear_accelerates',minimum_lateral_step=min_step,
            centred_at=complete,max_lateral_step=max_step,collisions=sorted(collisions),teleports=sorted(teleports))))


if __name__=='__main__':unittest.main()
