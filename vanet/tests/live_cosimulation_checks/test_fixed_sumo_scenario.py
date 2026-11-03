"""独占临时 SUMO 生命周期测试；VANET_RUN_LIVE_SUMO=1 时运行。"""
import os
import math
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY,SUMO_TOWN04_CONFIGURATION_FILE
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController
from vanet.npc.yielding_action_control.sumo_actions import YieldingActions


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO')=='1','Explicit isolated live SUMO test only')
class FixedSumoScenarioTests(unittest.TestCase):
    def test_prepare_depart_yield_restore_and_reset(self):
        sys.path.insert(0,str(SUMO_INSTALLATION_DIRECTORY/'tools'))
        import traci
        connection=None
        with tempfile.TemporaryDirectory(prefix='vanet_scenario_live_') as folder:
            log_file=Path(folder)/'sumo.log'
            error_log_file=Path(folder)/'sumo-errors.log'
            sumo_binary_file=SUMO_INSTALLATION_DIRECTORY/'bin/sumo'
            try:
                with socket.socket() as listener:
                    listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
                label='vanet_scenario_live_test'
                traci.start([str(sumo_binary_file),'-c',str(SUMO_TOWN04_CONFIGURATION_FILE),
                             '--step-length','.05','--lateral-resolution','.25','--seed','20261104',
                             '--no-step-log','true','--log',str(log_file),'--error-log',str(error_log_file)],
                            port=port,label=label)
                connection=traci.getConnection(label)
                scenario=ScenarioController(connection);scenario.reset()
                scenario.traffic.spawn_next_vehicles(20)
                for _ in range(3):connection.simulationStep()
                initial=set(connection.vehicle.getIDList())
                self.assertEqual(len(initial),2)
                self.assertTrue(all(connection.vehicle.getSpeed(name)==0. for name in initial))
                name='vanet_npc_front_01'
                actions=YieldingActions(name,{})
                own=dict(speed=connection.vehicle.getSpeed(name),width=connection.vehicle.getWidth(name),
                         lane_id=connection.vehicle.getLaneID(name))
                status,reason=actions.apply(connection,own,dict(kind='same_direction',inside_corridor=True))
                self.assertEqual((status,reason),('yielding','safe_adjacent_lane'))
                actions.restore(connection,own,0.);actions.restore(connection,own,3.)
                self.assertEqual(connection.vehicle.getSpeedMode(name),31)
                self.assertEqual(connection.vehicle.getLaneChangeMode(name),1621)
                scenario.start(0.)
                seen = set()
                positions = {}
                maximum_travel = {}
                for index in range(1,1601):
                    scenario.step(index*.05);scenario.traffic.spawn_next_vehicles(20);connection.simulationStep()
                    for vehicle_id in connection.vehicle.getIDList():
                        seen.add(vehicle_id)
                        position = connection.vehicle.getPosition(vehicle_id)
                        origin = positions.setdefault(vehicle_id,position)
                        maximum_travel[vehicle_id] = max(maximum_travel.get(vehicle_id,0.),math.dist(origin,position))
                expected = {item['id'] for item in scenario.config['npcs']}
                self.assertEqual(scenario.traffic.next_index,len(expected))
                # 完成固定路线的车辆可以正常驶离，不能要求第80秒仍有五辆车。
                self.assertEqual(seen,expected)
                for entry in scenario.config['npcs']:
                    if entry.get('depart_offset',0.) > 0:
                        self.assertGreater(maximum_travel[entry['id']],10.,entry['id'])
                scenario.reset();scenario.traffic.spawn_next_vehicles(20)
                for _ in range(3):connection.simulationStep()
                self.assertEqual(set(connection.vehicle.getIDList()),initial)
                self.assertTrue(all(connection.vehicle.getSpeed(name)==0. for name in initial))
                self.assertEqual(error_log_file.read_text(),'')
            finally:
                if connection is not None:connection.close()


if __name__=='__main__':unittest.main()
