"""Launch the external-clock Veins network in Qtenv (or explicit Cmdenv checks)."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
from datetime import datetime, timezone
from vanet.project_paths import (
    OMNET_COBRIDGE_COMMUNICATION_SOCKET_FILE, OMNET_INSTALL_DIRECTORY,
    VEINS_INSTALL_DIRECTORY, OMNET_NETWORK_DEFINITIONS_DIRECTORY,
    OMNET_SIMULATION_CONFIGURATION_DIRECTORY, OMNET_RUNTIME_DIRECTORY,
    EXPERIMENT_RESULTS_DIRECTORY, OMNET_EXECUTABLE_FILE, OMNET_LIBRARY_DIRECTORY,
    VEINS_SOURCE_DIRECTORY, VEINS_NED_DIRECTORY, VEINS_WIRELESS_LIBRARY_FILE, OMNET_RADIO_PARAMETERS_FILE,
)
from vanet.omnet.build_and_launch.build_network import build_network
from vanet.omnet.map_and_communication_display.scene_configuration import generate_scene


def generate_configuration(directory, socket_path, seed=None):
    directory.mkdir(parents=True, exist_ok=True)
    parameters = json.loads(OMNET_RADIO_PARAMETERS_FILE.read_text())
    scene,scene_path=generate_scene(directory)
    if seed is not None:
        parameters['seed'] = seed
    model = directory / 'radio_models.xml'
    model.write_text(f'''<?xml version="1.0"?>
<root><AnalogueModels><AnalogueModel type="SimplePathlossModel" thresholding="true">
<parameter name="alpha" type="double" value="{float(parameters['pathloss_exponent'])}"/>
</AnalogueModel></AnalogueModels><Decider type="Decider80211p">
<parameter name="centerFrequency" type="double" value="5.890e9"/>
</Decider></root>''')
    quote = lambda value: json.dumps(str(value))
    text = f'''[General]
network = vanet.radio.ExternalNetwork
scheduler-class = "ExternalClockScheduler"
qtenv-default-config = "General"
qtenv-default-run = "0"
cmdenv-express-mode = true
cmdenv-autoflush = true
cmdenv-status-frequency = 10s
**.cmdenv-log-level = warn
seed-set = {parameters['seed']}
**.scalar-recording = true
**.vector-recording = true
output-scalar-file = {quote(directory / 'network.sca')}
output-vector-file = {quote(directory / 'network.vec')}
*.gateway.socketPath = {quote(socket_path)}
*.gateway.eventFile = {quote(directory / 'wireless_events.jsonl')}
*.gateway.stepStatisticsFile = {quote(directory / 'network_steps.jsonl')}
*.gateway.allowedResultsRoot = {quote(EXPERIMENT_RESULTS_DIRECTORY.resolve())}
*.gateway.vehicleAntennaOffsetZ = {parameters['vehicle_antenna_offset_z_m']}m
*.gateway.coordinateOffsetX = {scene['coordinate_offset_x']}
*.gateway.coordinateOffsetY = {scene['coordinate_offset_y']}
*.gateway.v2vRange = {scene['communication_ranges']['v2v_metres']}m
*.gateway.rsuRange = {scene['communication_ranges']['rsu_metres']}m
*.gateway.sceneFile = {quote(scene_path)}
*.connectionManager.sendDirect = true
*.connectionManager.maxInterfDist = {parameters['maximum_interference_distance_m']}m
*.connectionManager.drawMaxIntfDist = false
*.radio*.veinsmobility.x = 0
*.radio*.veinsmobility.y = 0
*.radio*.veinsmobility.z = 0
*.radio*.nic.mac1609_4.useServiceChannel = false
*.radio*.nic.mac1609_4.txPower = {parameters['tx_power_mw']}mW
*.radio*.nic.mac1609_4.bitrate = {parameters['bitrate_mbps']}Mbps
*.radio*.nic.mac1609_4.queueSize = 512
*.radio*.nic.phy80211p.minPowerLevel = {parameters['minimum_power_dbm']}dBm
*.radio*.nic.phy80211p.useNoiseFloor = true
*.radio*.nic.phy80211p.noiseFloor = {parameters['noise_floor_dbm']}dBm
*.radio*.nic.phy80211p.decider = xmldoc({quote(model)})
*.radio*.nic.phy80211p.analogueModels = xmldoc({quote(model)})
*.radio*.nic.phy80211p.usePropagationDelay = true
*.radio*.nic.phy80211p.collectCollisionStatistics = true
*.radio*.nic.phy80211p.antennaOffsetZ = 0m
'''
    configuration = directory / 'omnetpp.ini'
    configuration.write_text(text)
    (directory / 'radio_parameters.json').write_text(json.dumps(parameters, indent=2))
    return configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', type=Path, default=OMNET_COBRIDGE_COMMUNICATION_SOCKET_FILE)
    parser.add_argument('--results', type=Path)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--ui',choices=('Qtenv','Cmdenv'),default='Qtenv')
    args = parser.parse_args()
    library = build_network()
    if args.build_only:
        return
    directory = args.results or EXPERIMENT_RESULTS_DIRECTORY / ('network_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')) / 'omnet'
    directory = directory.resolve()
    args.socket = args.socket.resolve()
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    OMNET_RUNTIME_DIRECTORY.mkdir(parents=True, exist_ok=True)
    lock_path = args.socket.with_suffix('.lock')
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'OMNeT++ already owns {args.socket}')
        args.socket.unlink(missing_ok=True)
        configuration = generate_configuration(directory, args.socket, args.seed)
        command = [str(OMNET_EXECUTABLE_FILE), '-u', args.ui,
                   '-n', os.pathsep.join(map(str, (OMNET_NETWORK_DEFINITIONS_DIRECTORY, VEINS_NED_DIRECTORY))),
                   '-l', str(VEINS_WIRELESS_LIBRARY_FILE), '-l', str(library), '-f', str(configuration)]
        env = dict(os.environ)
        env['LD_LIBRARY_PATH'] = os.pathsep.join([str(OMNET_LIBRARY_DIRECTORY), str(VEINS_SOURCE_DIRECTORY), env.get('LD_LIBRARY_PATH', '')])
        (directory / 'launch.json').write_text(json.dumps({'command': command, 'socket': str(args.socket)}, indent=2))
        print('[OMNeT++] Starting Veins IEEE 802.11p; CoBridge owns simulation time', flush=True)
        print('[OMNeT++] Results: ' + str(directory), flush=True)
        process = subprocess.Popen(command, env=env, cwd=directory)
        def stop(*_):
            process.terminate()
        previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            code = process.wait()
            if code:
                raise SystemExit(code)
        finally:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            for sig, handler in previous.items(): signal.signal(sig, handler)
            args.socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
