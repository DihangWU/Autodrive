"""独立 SUMO 验证 Town04 路线切绿与原配时恢复，不接管用户联仿。"""
import os
import math
from pathlib import Path
import socket
import sys
import tempfile
import unittest

from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE,SUMO_TOWN04_NETWORK_FILE
from vanet.rsu.traffic_signal_priority.route_signals import RouteSignalIndex
from vanet.rsu.traffic_signal_priority.priority_controller import TrafficSignalPriority


def town_route(index):
    lanes=['-7_1','-8_1','-9_1','-18_1','-19_1'];route=[]
    for current,following in zip(lanes,lanes[1:]):
        route.extend(index.shapes[current])
        outgoing,lane=following.rsplit('_',1)
        link=next(link for link in index.successors[current] if link.get('to')==outgoing and link.get('toLane')==lane)
        via=link.get('via')
        while via:
            route.extend(index.shapes[via])
            link=next(link for link in index.successors[via] if link.get('to')==outgoing and link.get('toLane')==lane)
            via=link.get('via')
    route.extend(index.shapes[lanes[-1]])
    return route


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated SUMO check only')
class SignalPriorityLiveTests(unittest.TestCase):
    def test_route_lights_green_together_and_restore_after_each_junction(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        with tempfile.TemporaryDirectory(prefix='rsu_signals_') as directory:
            with socket.socket() as listener:
                listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
            traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(SUMO_TOWN04_CONFIGURATION_FILE),
                         '--step-length','.05','--no-step-log','true','--error-log',str(Path(directory)/'errors.log')],
                        port=port,label='rsu_signal_test')
            c=traci.getConnection('rsu_signal_test')
            try:
                index=RouteSignalIndex(SUMO_TOWN04_NETWORK_FILE);route=town_route(index)
                matches=index.match(route)
                self.assertEqual([item['controller'] for item in matches],['1197','1103','1008'])
                self.assertEqual(index.match(list(reversed(route))),[])
                self.assertEqual(index.match([{**p,'z':p['z']+10.} for p in route]),[])
                originals={name:(c.trafficlight.getProgram(name),c.trafficlight.getPhase(name),
                                 c.trafficlight.getNextSwitch(name)-c.simulation.getTime(),
                                 c.trafficlight.getRedYellowGreenState(name)) for name in c.trafficlight.getIDList()}
                events=[];control=TrafficSignalPriority({},lambda event,now,**fields:events.append((event,fields)))
                message=dict(type='emergency',source='lincoln0',task_id='test',route_version=1,route=route,position=route[0])
                control.receive(message,0.,c,SUMO_TOWN04_NETWORK_FILE,route[0])
                for target in matches:
                    name=target['controller'];state=c.trafficlight.getRedYellowGreenState(name)
                    self.assertEqual(state,target['green_state'])
                    self.assertEqual(state[target['link_index']],'G')
                    # Every group rendered by one CARLA lamp has one colour.
                    for group in index.groups[name]:self.assertEqual(len({state[i] for i in group}),1)
                    self.assertIn('r',state)
                for name,saved in originals.items():
                    if name not in control.active:self.assertEqual(c.trafficlight.getRedYellowGreenState(name),saved[3])
                # Run an ordinary SUMO-controlled probe along the actual road;
                # position updates are only used to observe passed junctions.
                c.route.add('probe_route',['-7','-8','-9','-18','-19'])
                c.vehicle.add('signal_probe','probe_route',departLane='1',departPos='5',departSpeed='0')
                c.vehicle.setLaneChangeMode('signal_probe',512)
                c.vehicle.setSpeedFactor('signal_probe',1.)
                restored=[];collisions=set();teleports=set()
                for _ in range(1400):
                    c.simulationStep();now=c.simulation.getTime()
                    collisions.update(c.simulation.getCollidingVehiclesIDList());teleports.update(c.simulation.getStartingTeleportIDList())
                    if 'signal_probe' not in c.vehicle.getIDList():break
                    x,y,z=c.vehicle.getPosition3D('signal_probe');length=c.vehicle.getLength('signal_probe')
                    yaw=math.radians(c.vehicle.getAngle('signal_probe')-90.)
                    position=dict(x=x-index.offset[0]-length/2*math.cos(yaw),
                                  y=index.offset[1]-y-length/2*math.sin(yaw),z=z,length=length)
                    before=set(control.active);control.update(position,now)
                    for name in sorted(before-set(control.active)):
                        restored.append(name)
                        saved=originals[name]
                        self.assertEqual(c.trafficlight.getProgram(name),saved[0])
                        self.assertEqual(c.trafficlight.getPhase(name),saved[1])
                        self.assertAlmostEqual(c.trafficlight.getNextSwitch(name)-now,saved[2],places=3)
                    for name,record in control.active.items():
                        self.assertEqual(c.trafficlight.getRedYellowGreenState(name),record['target']['green_state'])
                    # Repeated radio delivery cannot re-lock restored signals.
                    if int(now*20)%20==0:control.receive(message,now,c,SUMO_TOWN04_NETWORK_FILE,position)
                    if not control.active:break
                self.assertEqual(restored,['1197','1103','1008'])
                self.assertFalse(collisions);self.assertFalse(teleports)
                # A new task starts afresh; cancellation restores all at once.
                message['task_id']='second_task';control.receive(message,100.,c,SUMO_TOWN04_NETWORK_FILE,route[0])
                control.release_all(101.,'operator_cancelled')
                self.assertFalse(control.active)
                self.assertTrue(all(c.trafficlight.getProgram(t['controller'])=='0' for t in matches))
                print('RSU signals: 1197/1103/1008 green together; restored in route order; '
                      'shared CARLA lamps consistent; normal probe passed; cancellation restored all; no collisions/teleports')
            finally:c.close()


if __name__=='__main__':unittest.main()
