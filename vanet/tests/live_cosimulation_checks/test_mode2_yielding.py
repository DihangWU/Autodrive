"""独立真实 SUMO 验证 Mode2 驾驶动作；receive 注入代表已送达的无线事件。"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import socket
import subprocess
import sys
import tempfile
import unittest

from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,EXPERIMENT_DEFAULT_CONFIGURATION_FILE
from vanet.npc.highway_ramp_merging.zipper_network import prepare_zipper_configuration
from vanet.npc.highway_ramp_merging.merge_controller import RampMergeController
from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.cooperation import NpcCooperation
from vanet.lincoln.emergency_driving_control.collision_guard import collision_speed_limit


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated SUMO checks only')
class Mode2YieldingLiveTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        import traci.constants as tc
        self.tc=tc;self.folder=tempfile.TemporaryDirectory(prefix='vanet_mode2_')
        configuration=prepare_zipper_configuration(Path(self.folder.name)/'map')
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
        traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(configuration),
            '--step-length','.05','--lateral-resolution','.25','--seed','20261104',
            '--collision.check-junctions','true','--no-step-log','true',
            '--error-log',str(Path(self.folder.name)/'errors.log')],port=port,label='mode2_test')
        self.c=traci.getConnection('mode2_test')
        self.config=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())

    def tearDown(self):
        self.c.close();self.folder.cleanup()

    def state(self,name,role='same_direction'):
        c=self.c;p=c.vehicle.getPosition3D(name);heading=c.vehicle.getAngle(name)-90.;a=math.radians(heading)
        length=c.vehicle.getLength(name)
        return dict(id=name,scenario_role=role,x=p[0]-503.02-length/2*math.cos(a),
                    y=423.76-p[1]-length/2*math.sin(a),z=p[2]+.75,heading=heading,
                    speed=c.vehicle.getSpeed(name),width=c.vehicle.getWidth(name),length=length,
                    lane_id=c.vehicle.getLaneID(name),road_id=c.vehicle.getRoadID(name))

    def received_task(self,route,position,now,sequence):
        return dict(type='emergency',source='lincoln0',experiment_mode='mode2',task_id='mode2-test',
                    route_version=1,sequence=sequence,sent_at=now,route=route,
                    position={k:position[k] for k in ('x','y','z')},speed=position['speed'])

    def load_straight_road(self):
        c=self.c;folder=Path(self.folder.name)
        nodes=folder/'front_nodes.xml';edges=folder/'front_edges.xml';network=folder/'front_test.net.xml'
        nodes.write_text('<nodes><node id="start" x="0" y="0"/><node id="end" x="2500" y="0"/></nodes>')
        edges.write_text('<edges><edge id="approach" from="start" to="end" numLanes="5" speed="27.78"/></edges>')
        subprocess.run([str(SUMO_INSTALLATION_DIRECTORY/'bin/netconvert'),'--node-files',str(nodes),'--edge-files',str(edges),'--output-file',str(network)],check=True,capture_output=True)
        c.load(['-n',str(network),'--step-length','.05','--lateral-resolution','.25','--collision.check-junctions','true','--no-step-log','true','--lanechange.overtake-right','true','--seed','20261104'])
        c.route.add('front_route',['approach'])
        c.lane.setDisallowed('approach_0',['all'])

    def test_multiple_front_cars_move_left_only_after_reception_and_lincoln_passes(self):
        self.load_straight_road();c=self.c
        c.vehicle.add('lincoln_probe','front_route',departLane='1',departPos='40',departSpeed='0')
        c.vehicle.setMaxSpeed('lincoln_probe',100/3.6);c.vehicle.setSpeedFactor('lincoln_probe',1.);c.vehicle.setImperfection('lincoln_probe',0.);c.vehicle.setLaneChangeMode('lincoln_probe',512)
        c.vehicle.setTau('lincoln_probe',.35);c.vehicle.setMinGap('lincoln_probe',3.)
        # Match Lincoln's existing highest-right-of-way controller: overtaking
        # a vehicle that yielded left is allowed while collision checks remain.
        names=['front1','front2','front3'];components={}
        for index,name in enumerate(names):
            c.vehicle.add(name,'front_route',departLane='1',departPos=str(120+index*35),departSpeed='0')
            c.vehicle.setMaxSpeed(name,80/3.6);c.vehicle.setImperfection(name,0.)
            c.vehicle.setLaneChangeMode(name,1536)
            c.vehicle.setParameter(name,'laneChangeModel.lcSpeedGain','0')
            c.vehicle.setParameter(name,'laneChangeModel.lcKeepRight','0')
            components[name]=NpcCooperation(name,dict(self.config,experiment_mode='mode2',scenario_role='same_direction',yielding_enabled=True))
        for _ in range(3):c.simulationStep()
        route=[dict(x=x-503.02,y=423.76-y,z=.75) for x,y in c.lane.getShape('approach_1')]
        next_message={name:0. for name in names};sequence=0;received=set();changed=set();restored=set();collisions=set();teleports=set()
        for index in range(1600):
            now=index*.05;ego=self.state('lincoln_probe')
            for name,npc in components.items():
                own=self.state(name);inside=math.hypot(own['x']-ego['x'],own['y']-ego['y'])<=50.
                npc.set_communication_links(['lincoln0'] if inside else [],now)
                npc.set_communication_path('direct' if inside else None,now)
                npc.set_emergency_vehicle_state(ego)
                if inside and now>=next_message[name]:
                    sequence+=1;npc.receive(self.received_task(route,ego,now,sequence),now)
                    received.add(name);next_message[name]=now+1.
                npc.step(now,own,c)
                if c.vehicle.getLaneIndex(name)==2:changed.add(name)
                if name in changed and npc.status=='normal':restored.add(name)
            c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList());teleports.update(c.simulation.getStartingTeleportIDList())
            if restored==set(names):break
        print(json.dumps(dict(mode='Mode2',front_received=sorted(received),front_left_yielded=sorted(changed),restored=sorted(restored),collisions=sorted(collisions),teleports=sorted(teleports),ego_final=dict(self.state('lincoln_probe'),leader=c.vehicle.getLeader('lincoln_probe',200.),factor=c.vehicle.getSpeedFactor('lincoln_probe'),max_speed=c.vehicle.getMaxSpeed('lincoln_probe')),final={name:dict(status=npc.status,reason=npc.reason,road=c.vehicle.getRoadID(name),lane=c.vehicle.getLaneID(name),lateral=c.vehicle.getLateralLanePosition(name),speed=c.vehicle.getSpeed(name)) for name,npc in components.items()})))
        self.assertEqual(received,set(names));self.assertEqual(changed,set(names));self.assertEqual(restored,set(names))
        self.assertEqual(collisions,set());self.assertEqual(teleports,set())

    def test_compact_gap_is_used_and_overlapping_vehicle_is_waited_for_without_straddling(self):
        for scene in ('compact_gap','overlap_then_gap','tight_gap','fast_rear'):
            with self.subTest(scene=scene):
                self.load_straight_road();c=self.c
                layout=[('yielding_front',1,100)]
                layout+=([('lead',2,126),('follower',2,74)] if scene=='compact_gap' else
                         [('lead',2,119),('follower',2,81)] if scene=='tight_gap' else
                         [('fast_follower',2,80)] if scene=='fast_rear' else [('beside',2,100)])
                for name,lane,position in layout:
                    c.vehicle.add(name,'front_route',departLane=str(lane),departPos=str(position),departSpeed='25' if name=='fast_follower' else '15')
                    c.vehicle.setLaneChangeMode(name,1536);c.vehicle.setImperfection(name,0.)
                    c.vehicle.setSpeed(name,25. if name=='fast_follower' else 15.);c.vehicle.setSpeedFactor(name,1.)
                for _ in range(3):c.simulationStep()
                npc=NpcCooperation('yielding_front',dict(self.config,experiment_mode='mode2',scenario_role='same_direction',yielding_enabled=True))
                route=[dict(x=x-503.02,y=423.76-y,z=.75) for x,y in c.lane.getShape('approach_1')]
                npc.set_communication_links(['lincoln0'],0.);npc.set_communication_path('direct',0.)
                started=c.simulation.getTime();requested=None;centred=None;crossed=None;collisions=set();teleports=set()
                previous_y=c.vehicle.getPosition('yielding_front')[1];max_lateral_step=0.
                for index in range(160):
                    now=index*.05;own=self.state('yielding_front')
                    # A car alongside occupies the destination physically.
                    # Free it after four seconds and verify the waiting car
                    # takes the slot, without crossing through the obstacle.
                    if scene=='overlap_then_gap' and index==80:c.vehicle.remove('beside')
                    ego=dict(own,x=own['x']-40.)
                    npc.set_emergency_vehicle_state(ego)
                    if index%10==0:
                        self.assertTrue(npc.receive(self.received_task(route,ego,now,index+1),now))
                    npc.step(now,own,c)
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList());teleports.update(c.simulation.getStartingTeleportIDList())
                    lane=c.vehicle.getLaneIndex('yielding_front');lateral=c.vehicle.getLateralLanePosition('yielding_front')
                    if requested is None and abs(lateral)>.001:requested=c.simulation.getTime()-started-.05
                    y=c.vehicle.getPosition('yielding_front')[1];max_lateral_step=max(max_lateral_step,abs(y-previous_y));previous_y=y
                    elapsed=c.simulation.getTime()-started
                    if scene=='overlap_then_gap' and index<80:
                        self.assertEqual(lane,1);self.assertLess(abs(lateral),.1)
                    if lane==2 and crossed is None:crossed=elapsed
                    if npc.actions.left_maneuver.centred:centred=elapsed;break
                self.assertIsNotNone(centred)
                self.assertIsNotNone(requested)
                self.assertLessEqual(centred-requested,1.5+1e-8)
                self.assertLess(centred-crossed,1.2)
                self.assertLessEqual(max_lateral_step,self.config['mode2']['lateral_speed_mps']*.05+.01)
                self.assertEqual(collisions,set());self.assertEqual(teleports,set())
                print(json.dumps(dict(mode='Mode2',scene=scene,wait_for_gap_seconds=requested,full_lane_change_seconds=centred-requested,
                    boundary_to_centre_seconds=centred-crossed,max_lateral_step_metres=max_lateral_step,
                    collisions=sorted(collisions),teleports=sorted(teleports))))

    def test_town04_main_internal_link_finishes_four_to_three_without_moving_lane_three(self):
        c=self.c
        c.route.add('main_internal',['-46','-47','-48','-49','-50'])
        names=('lane4_front','lane3_normal')
        for name,lane,position in ((names[0],1,120),(names[1],2,140)):
            c.vehicle.add(name,'main_internal',departLane=str(lane),departPos=str(position),departSpeed='10')
            c.vehicle.setLaneChangeMode(name,1536);c.vehicle.setImperfection(name,0.)
            c.vehicle.setSpeed(name,10.);c.vehicle.setSpeedFactor(name,1.)
        for _ in range(3):c.simulationStep()
        components={name:NpcCooperation(name,dict(self.config,experiment_mode='mode2',scenario_role='same_direction',yielding_enabled=True)) for name in names}
        points=[]
        for lane in ('-46_1',':1061_2_1','-47_1','-48_1'):
            points.extend(c.lane.getShape(lane))
        route=[dict(x=x-503.02,y=423.76-y,z=.75) for x,y in points]
        received=False;completed=False;begin=None;passage_clear=None;collisions=set();teleports=set()
        for index in range(300):
            now=index*.05;front=self.state(names[0]);ego=dict(front,y=front['y']+40.,speed=15.)
            inside=front['road_id']==':1061_2'
            if inside and not received:received=True
            for name,npc in components.items():
                own=self.state(name);npc.set_communication_links(['lincoln0'],now);npc.set_communication_path('direct',now)
                npc.set_emergency_vehicle_state(ego)
                if received and index%5==0:self.assertTrue(npc.receive(self.received_task(route,ego,now,index+1),now))
                npc.step(now,own,c)
                if name==names[0] and begin is None and npc.actions.action=='lane_change':begin=now
                if name==names[0] and begin is not None and npc.status=='cleared' and passage_clear is None:
                    passage_clear=now
                if name==names[1]:self.assertIsNone(npc.actions.saved)
            c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList());teleports.update(c.simulation.getStartingTeleportIDList())
            self.assertEqual(c.vehicle.getLaneIndex(names[1]),2)
            if received and components[names[0]].actions.left_maneuver.centred:
                self.assertEqual(c.vehicle.getRoadID(names[0]),':1061_2')
                self.assertIsNotNone(begin);self.assertLessEqual(now-begin,1.5+1e-8)
                completed=True;break
        self.assertTrue(received);self.assertTrue(completed)
        self.assertIsNotNone(passage_clear);self.assertLess(passage_clear,now)
        self.assertEqual(collisions,set());self.assertEqual(teleports,set())
        # The yielded body must also clear Lincoln's actual physical guard,
        # not just switch the SUMO lane index and remain on the marking.
        front=self.state(names[0]);samples=[];distances=[];station=0.
        for first,second in zip(route,route[1:]):
            dx,dy=second['x']-first['x'],second['y']-first['y'];length=math.hypot(dx,dy)
            if length<1e-6:continue
            steps=max(1,math.ceil(length/.5))
            for index in range(steps):
                fraction=index/steps
                samples.append(SimpleNamespace(center=(first['x']+fraction*dx,first['y']+fraction*dy,0.),forward=(dx/length,dy/length,0.)))
                distances.append(station+fraction*length)
            station+=length
        nearest=min(range(len(samples)),key=lambda i:math.dist(samples[i].center[:2],(front['x'],front['y'])))
        guard=SimpleNamespace(samples=samples,distances=distances,progress=distances[nearest]-10.,front=2.5,half_width=1.1,
            collision_configuration=self.config['lincoln_collision_guard'],wait_reason='')
        obstacle=dict(id=1,kind='vehicle.npc',center=(front['x'],front['y'],front['z']),
            velocity=(0.,-10.,0.),yaw=math.radians(front['heading']),length=2.5,width=1.,height=1.)
        limit=collision_speed_limit(guard,[obstacle],30.,20.)
        self.assertEqual(limit,30.,dict(front=front,route_point=samples[nearest].center,guard=guard.collision_guard_state))
        self.assertIsNone(guard.collision_guard_state)
        print(json.dumps(dict(mode='Mode2',scene='Town04_internal_highway',lane4_to_lane3=completed,
            lane3_unchanged=True,seconds_to_passage=passage_clear-begin,
            seconds_to_centre=now-begin,collisions=sorted(collisions),teleports=sorted(teleports))))

    def test_ramp_stays_stopped_despite_zipper_updates_then_rejoins_after_passage(self):
        c=self.c;tc=self.tc;vehicles,policy=load_merging_vehicles()
        name=vehicles[0]['id'];entry=vehicles[0]
        c.route.add('ramp_route',entry['route']);c.vehicle.add(name,'ramp_route',departLane='1',departPos='140',departSpeed='0')
        c.vehicle.setMaxSpeed(name,80/3.6);c.vehicle.setSpeed(name,0.);c.vehicle.setLaneChangeMode(name,1536)
        for _ in range(3):c.simulationStep()
        controller=RampMergeController(c,policy);created={name:dict(scenario=entry,spec=dict(length=str(c.vehicle.getLength(name))))}
        config=dict(self.config,experiment_mode='mode2',scenario_role='ramp_merging',yielding_enabled=True,ramp_controller=controller)
        npc=NpcCooperation(name,config);controller.start(created,0.)
        point=config['mode2']['ramp_merge_point'];before=dict(x=point['x'],y=point['y']+40,z=0.,speed=10.)
        after=dict(x=point['x'],y=point['y']-20,z=0.,speed=10.)
        route=[dict(x=point['x'],y=point['y']+200,z=0.),dict(x=point['x'],y=point['y']-200,z=0.)]
        npc.set_communication_links(['lincoln0'],0.);npc.set_communication_path('direct',0.)
        fields=[tc.VAR_ROAD_ID,tc.VAR_EDGES,tc.VAR_LANEPOSITION,tc.VAR_LANE_ID,tc.VAR_LANEPOSITION_LAT,tc.VAR_SPEED]
        c.vehicle.subscribe(name,fields);stopped_samples=0;merged=False;collisions=set();teleports=set();receipt_speed=0.
        for index in range(1000):
            now=index*.05;own=self.state(name,'ramp_merging');npc.set_emergency_vehicle_state(before if now<8. else after)
            if index==40:
                receipt_speed=own['speed']
                self.assertTrue(npc.receive(self.received_task(route,before,now,1),now))
            npc.step(now,own,c)
            controller.update_lanes(created,c.vehicle.getAllSubscriptionResults(),tc,now)
            c.simulationStep()
            collisions.update(c.simulation.getCollidingVehiclesIDList());teleports.update(c.simulation.getStartingTeleportIDList())
            if 4.<=now<8.:
                self.assertEqual(c.vehicle.getRoadID(name),'31');self.assertLess(c.vehicle.getSpeed(name),.1);stopped_samples+=1
            if c.vehicle.getRoadID(name)=='-47':merged=True;break
        print(json.dumps(dict(mode='Mode2',ramp_receipt_speed_mps=receipt_speed,ramp_stopped_samples=stopped_samples,ramp_resumed=merged,collisions=sorted(collisions),teleports=sorted(teleports))))
        self.assertGreater(receipt_speed,3.);self.assertGreater(stopped_samples,50);self.assertTrue(merged)
        self.assertEqual(collisions,set());self.assertEqual(teleports,set())


if __name__=='__main__':unittest.main()
