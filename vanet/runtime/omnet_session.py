"""Launch OMNeT++; live RSUs are owned by the separate run_rsu.sh command."""
import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.paths import LINCOLN_DIR
sys.path.insert(0, str(LINCOLN_DIR))
from shared_sumo import control_folder, request
from vanet.common.config import CONFIG, load_config, rsu_definitions, radio_arguments


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--carla-host', default='127.0.0.1')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--no-scene-markers', action='store_true', help='Use static RSUs for offline tests only')
    parser.add_argument('--vanet-config', default=str(CONFIG))
    args, command = parser.parse_known_args()
    cfg = load_config(args.vanet_config)
    if any(arg.startswith(('--*.rsu', '--*.manager.rsuDefinitions=')) for arg in command):
        raise SystemExit('RSU coordinates must be changed in vanet/town04.json.')
    if any(arg.startswith('--') and any(name in arg.split('=', 1)[0] for name in
           ('vehicleRange', 'rsuRange', 'communicationRange', 'maxInterfDist')) for arg in command):
        raise SystemExit('Radio ranges must be changed in the JSON communication configuration.')
    command.extend(radio_arguments(cfg))
    configs = [command[i + 1] for i, arg in enumerate(command[:-1]) if arg in ('-c', '--config-name')]
    offline = args.no_scene_markers or (configs and configs[-1] == 'Smoke') or any(a in command for a in ('-h', '--help', '-v'))
    # Synthetic/offline tests can still supply static nodes. Normal live runs
    # receive exclusively the RSUs actually created by the shared bridge.
    command.append('--*.manager.rsuDefinitions=' + json.dumps(rsu_definitions(cfg) if offline else ''))
    if not any(arg.startswith('--*.manager.port=') for arg in command):
        command.append(f'--*.manager.port={cfg["network"]["port"]}')
    lock = None
    child = None
    try:
        if not offline:
            folder = control_folder(args.carla_host, args.carla_port)
            lock = (folder / 'vanet-omnet-session.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another OMNeT++ session is already running.')
            status = request(args.carla_host, args.carla_port, {'action': 'status'}, timeout=8)
            if status.get('vanet_scene', {}).get('protocol') != 5:
                raise RuntimeError('共享 SUMO 桥仍是旧版本。请正常关闭车辆后，执行 bash run_lincoln.sh --stop-sumo，再重新启动车辆。')
            print('Live RSUs are controlled independently by bash vanet/run_rsu.sh.', flush=True)
        child = subprocess.Popen(command, close_fds=True, start_new_session=True)
        def forward(signum, frame):
            if child.poll() is None:
                os.killpg(child.pid, signum)
        signal.signal(signal.SIGINT, forward)
        signal.signal(signal.SIGTERM, forward)
        return child.wait()
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as error:
        raise SystemExit(str(error))
