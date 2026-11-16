"""独立 SUMO 复现右转入口保空等待，验证弯道跟进且仍遵守红灯。"""
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest

from modules.npcvehicules.traffic.vehicle_types import install_vehicle_type
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import load_scenario
from vanet.npc.highway_background_traffic.right_turn_queueing import RightTurnQueueing,KEEP_CLEAR_PARAMETER
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated real SUMO test only')
class RightTurnQueueingLiveTests(unittest.TestCase):
    def test_follow_queue_into_curve_without_crossing_red_light(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        import traci.constants as tc
        with tempfile.TemporaryDirectory(prefix='right_turn_queue_') as directory:
            with socket.socket() as listener:
                listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
            traci.start([str(SUMO_INSTALLATION_DIRECTORY/'bin/sumo'),'-c',str(SUMO_TOWN04_CONFIGURATION_FILE),
                         '--step-length','.05','--lateral-resolution','.25','--no-step-log','true',
                         '--error-log',str(Path(directory)/'errors.log')],port=port,label='right_turn_queue_test')
            c=traci.getConnection('right_turn_queue_test')
            try:
                config=load_scenario();spec=config['vehicle_type_spec']
                install_vehicle_type(c,spec,set(c.vehicletype.getIDList()))
                c.route.add('blue',['-7','-8']);c.route.add('turn',['-48','-7','-8'])
                c.trafficlight.setPhase('1197',0);c.trafficlight.setPhaseDuration('1197',200.)
                for index,position in enumerate([21.84,15.62,9.40]):
                    c.vehicle.add('blue'+str(index),'blue',typeID=spec['id'],departLane='1',
                                  departPos=str(position),departSpeed='0')
                c.vehicle.add('candidate','turn',typeID=spec['id'],departLane='1',departPos='45',departSpeed='0')
                collisions=set();teleports=set()
                for _ in range(400):
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    teleports.update(c.simulation.getStartingTeleportIDList())
                self.assertEqual(c.vehicle.getLaneID('candidate'),'-48_1')
                self.assertEqual(c.vehicle.getSpeed('candidate'),0.)
                shared_original=c.vehicletype.getParameter(spec['id'],KEEP_CLEAR_PARAMETER)
                policy=RightTurnQueueing(c,config['right_turn_queueing'])
                created={'candidate':dict(scenario={})}
                c.vehicle.subscribe('candidate',[tc.VAR_ROAD_ID,tc.VAR_EDGES])
                seen=set()
                for _ in range(400):
                    policy.update(created,c.vehicle.getAllSubscriptionResults(),tc,c.simulation.getTime())
                    c.simulationStep()
                    seen.add(c.vehicle.getRoadID('candidate'))
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    teleports.update(c.simulation.getStartingTeleportIDList())
                    self.assertEqual(c.trafficlight.getRedYellowGreenState('1197')[2],'r')
                    self.assertNotIn('-8',seen)
                self.assertTrue({':890_2',':890_13','-7'}<=seen)
                self.assertEqual(c.vehicle.getLaneID('candidate'),'-7_1')
                self.assertEqual(c.vehicle.getSpeed('candidate'),0.)
                self.assertAlmostEqual(c.vehicle.getLanePosition('candidate'),3.181121,places=3)
                self.assertEqual(c.vehicle.getSpeedMode('candidate'),31)
                self.assertEqual(policy.active,{})
                private_type=c.vehicle.getTypeID('candidate')
                self.assertNotEqual(private_type,spec['id'])
                self.assertEqual(c.vehicletype.getParameter(private_type,KEEP_CLEAR_PARAMETER),'-1')
                self.assertEqual(c.vehicletype.getParameter(spec['id'],KEEP_CLEAR_PARAMETER),shared_original)
                for name in ('blue0','blue1','blue2'):
                    self.assertEqual(c.vehicle.getLaneID(name),'-7_1')
                    self.assertEqual(c.vehicle.getSpeed(name),0.)
                c.trafficlight.setPhase('1197',6);c.trafficlight.setPhaseDuration('1197',100.)
                for _ in range(200):
                    c.simulationStep()
                    collisions.update(c.simulation.getCollidingVehiclesIDList())
                    teleports.update(c.simulation.getStartingTeleportIDList())
                    if c.vehicle.getRoadID('candidate')=='-8':break
                self.assertEqual(c.vehicle.getRoadID('candidate'),'-8')
                self.assertEqual(collisions,set());self.assertEqual(teleports,set())
                print('Right-turn queue: baseline waits before curve; adjusted vehicle follows red-light queue; '
                      'green resumes travel; speedMode=31; shared type unchanged; collisions=0; teleports=0')
            finally:
                c.close()


if __name__=='__main__':unittest.main()
