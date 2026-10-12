"""Read town04.json and own scene markers for exactly one OMNeT++ process."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'lincoln'))
from shared_sumo import atomic_json, control_folder, identity, request
from vanet.config import CONFIG, load_config, rsu_definitions


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--carla-host', default='127.0.0.1')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--no-scene-markers', action='store_true')
    parser.add_argument('--vanet-config', default=str(CONFIG))
    args, command = parser.parse_known_args()
    cfg = load_config(args.vanet_config)
    if any(arg.startswith(('--*.rsu', '--*.manager.rsuDefinitions=')) for arg in command):
        raise SystemExit('RSU coordinates must be changed in vanet/town04.json.')
    command.append('--*.manager.rsuDefinitions=' + json.dumps(rsu_definitions(cfg)))
    if not any(arg.startswith('--*.manager.port=') for arg in command):
        command.append(f'--*.manager.port={cfg["network"]["port"]}')
    configs = [command[i + 1] for i, arg in enumerate(command[:-1]) if arg in ('-c', '--config-name')]
    offline = args.no_scene_markers or (configs and configs[-1] == 'Smoke') or any(a in command for a in ('-h', '--help', '-v'))
    folder = control_folder(args.carla_host, args.carla_port)
    lease, reply = folder / 'vanet-scene.json', folder / 'vanet-scene-status.json'
    lock = None
    child = None
    session = uuid.uuid4().hex
    try:
        if not offline:
            lock = (folder / 'vanet-scene-session.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another OMNeT++ scene session is already running.')
            status = request(args.carla_host, args.carla_port, {'action': 'status'}, timeout=8)
            if status.get('vanet_scene', {}).get('protocol') != 2:
                raise RuntimeError('共享 SUMO 桥仍是旧版本。请正常关闭车辆后，在工程根目录执行 bash run_lincoln.sh --stop-sumo，再重新启动车辆。')
        print(f'RSUs from {args.vanet_config}: {rsu_definitions(cfg)}', flush=True)
        child = subprocess.Popen(command, close_fds=True, start_new_session=True)
        def forward(signum, frame):
            if child.poll() is None:
                os.killpg(child.pid, signum)
        signal.signal(signal.SIGINT, forward)
        signal.signal(signal.SIGTERM, forward)
        if not offline:
            atomic_json(lease, dict(session=session, owner=identity(), child=identity(child.pid), config=cfg))
            deadline = time.monotonic() + 10
            while child.poll() is None:
                state = json.loads(reply.read_text()) if reply.exists() else {}
                if state.get('session') == session:
                    if not state['ok']:
                        raise RuntimeError(state['error'])
                    print('CARLA + SUMO RSU markers enabled for this OMNeT++ session.', flush=True)
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('共享桥未确认 RSU 显示；请确认 SUMO 正在运行。')
                time.sleep(.1)
        return child.wait()
    finally:
        if lease.exists() and json.loads(lease.read_text()).get('session') == session:
            lease.unlink()
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
