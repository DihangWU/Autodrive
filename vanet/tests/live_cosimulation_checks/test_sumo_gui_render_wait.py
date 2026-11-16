"""独立 GUI 回归：车道外投影等待 TraCI 时，重绘不能阻塞或推进时钟。"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from foundation.sumo.init.gui.build_sumo_gui import GUI_BINARY
from foundation.sumo.init.gui.traci_render_sumo_gui import (
    DISABLE_READ_UNLOCK_ENVIRONMENT_VARIABLE, PAINT_TRACE_ENVIRONMENT_VARIABLE,
)
from vanet.project_paths import SUMO_INSTALLATION_DIRECTORY, SUMO_TOWN04_CONFIGURATION_FILE


@unittest.skipUnless(os.environ.get('VANET_RUN_LIVE_SUMO_GUI') == '1',
                     'Explicit isolated live SUMO GUI test only')
class SumoGuiRenderWaitTests(unittest.TestCase):
    def measure(self, legacy_lock, directory):
        sys.path.insert(0, str(SUMO_INSTALLATION_DIRECTORY / 'tools'))
        import traci

        env = dict(os.environ)
        env[PAINT_TRACE_ENVIRONMENT_VARIABLE] = '1'
        env[DISABLE_READ_UNLOCK_ENVIRONMENT_VARIABLE] = '1' if legacy_lock else '0'
        log_path = directory / ('legacy.log' if legacy_lock else 'fixed.log')
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port = listener.getsockname()[1]
        connection = None
        with log_path.open('w') as log:
            process = subprocess.Popen([
                str(GUI_BINARY), '-c', str(SUMO_TOWN04_CONFIGURATION_FILE),
                '--remote-port', str(port), '--step-length', '.05',
                '--start', 'true', '--quit-on-end', 'true',
                '--no-step-log', 'true', '--delay', '0',
            ], stdout=log, stderr=log, env=env)
            try:
                connection = traci.connect(port=port, numRetries=20, proc=process)
                connection._socket.settimeout(3)
                connection.route.add('render_check', ['-41', '-46', '-47'])
                connection.vehicle.add('lincoln0', 'render_check', departLane='1',
                                       departPos='200', departSpeed='0')
                connection.vehicle.add('npc_render_check', 'render_check', departLane='2',
                                       departPos='240', departSpeed='0')
                for _ in range(4):
                    connection.simulationStep()
                position = connection.vehicle.getPosition('lincoln0')
                # CARLA proxy uses keepRoute=2. Force its off-network drawing
                # path rather than testing tracking alone (which needs no net lock).
                connection.vehicle.moveToXY('lincoln0', '', -1, position[0] + 25,
                                            position[1] + 25, keepRoute=2, matchThreshold=0)
                connection.simulationStep()
                self.assertEqual(connection.vehicle.getLaneID('lincoln0'), '')
                connection.gui.trackVehicle('View #0', 'lincoln0')
                connection.gui.setZoom('View #0', 600)
                time.sleep(.2)
                before_time = connection.simulation.getTime()
                before_paints = self.paint_count(log_path)
                centre = connection.gui.getOffset('View #0')
                for index in range(12):
                    connection.gui.setOffset('View #0', centre[0] + index * .5, centre[1])
                    time.sleep(.1)
                idle_paints = self.paint_count(log_path) - before_paints
                after_time = connection.simulation.getTime()
                # Restore to a lane and verify that the GUI and clock still work.
                connection.vehicle.moveTo('lincoln0', '-41_1', 200)
                connection.vehicle.setSpeed('lincoln0', 15)
                connection.vehicle.setSpeed('npc_render_check', 14)
                moving_before = self.paint_count(log_path)
                for _ in range(24):
                    start = time.monotonic()
                    connection.simulationStep()
                    time.sleep(max(0, .05 - (time.monotonic() - start)))
                result = dict(idle_completed_paints=idle_paints,
                              simulation_time_before=before_time,
                              simulation_time_after=after_time,
                              moving_completed_paints=self.paint_count(log_path) - moving_before)
                self.assertEqual(before_time, after_time)
                self.assertGreater(result['moving_completed_paints'], 10)
                return result
            finally:
                if connection is not None:
                    try:
                        connection.close(wait=False)
                    except (OSError, traci.TraCIException):
                        pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)

    @staticmethod
    def paint_count(path):
        return sum(line.startswith('AUTODRIVE_GUI_PAINT ') for line in path.read_text().splitlines())

    def test_off_network_proxy_does_not_block_idle_paint(self):
        if not os.environ.get('DISPLAY'):
            self.skipTest('An X11 display is required')
        self.assertTrue(GUI_BINARY.is_file(), 'Build the project SUMO GUI first')
        with tempfile.TemporaryDirectory(prefix='sumo_gui_render_') as directory:
            directory = Path(directory)
            baseline = self.measure(True, directory)
            fixed = self.measure(False, directory)
            print(json.dumps(dict(legacy_lock=baseline, idle_unlock=fixed), ensure_ascii=False))
            self.assertGreater(fixed['idle_completed_paints'], baseline['idle_completed_paints'] + 3)
