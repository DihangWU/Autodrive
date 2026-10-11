"""Reuse an existing bridge or start exactly one local SUMO bridge."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import carla

PROJECT = Path(__file__).resolve().parents[1]


def bridges(host, port):
    matches = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            args = (directory / 'cmdline').read_bytes().decode().strip('\0').split('\0')
            if not any(Path(arg).name == 'spawn_npc_sumo.py' for arg in args):
                continue
            peer_host = args[args.index('--host') + 1] if '--host' in args else '127.0.0.1'
            peer_port = int(args[args.index('--port') + 1]) if '--port' in args else 2000
            local_hosts = {'localhost', '127.0.0.1'}
            if peer_port == port and (peer_host == host or {peer_host, host} <= local_hosts):
                matches.append(int(directory.name))
        except (OSError, UnicodeError, ValueError, IndexError):
            pass
    return matches


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--npcs', type=int, default=0)
    p.add_argument('--stop', action='store_true')
    args = p.parse_args()
    host_key = 'localhost' if args.host in ('localhost', '127.0.0.1') else args.host
    key = hashlib.sha256(f'{os.getuid()}:{host_key}:{args.port}'.encode()).hexdigest()[:16]
    folder = Path('/tmp') / ('carla-lincoln-' + key)
    folder.mkdir(mode=0o700, exist_ok=True)
    with (folder / 'lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        state = folder / 'bridge.json'
        if args.stop:
            if state.exists():
                pid = json.loads(state.read_text())['pid']
                if pid in bridges(args.host, args.port):
                    os.kill(pid, signal.SIGINT)
                    print(f'Stopping managed SUMO bridge {pid}; waiting for cleanup.', flush=True)
                    deadline = time.monotonic() + 45
                    while pid in bridges(args.host, args.port):
                        if time.monotonic() >= deadline:
                            raise SystemExit('SUMO bridge cleanup timed out; do not start a second bridge yet.')
                        time.sleep(.2)
                    print('Managed SUMO bridge stopped.', flush=True)
                state.unlink()
            return
        client = carla.Client(args.host, args.port)
        client.set_timeout(10)
        world = client.get_world()
        if not world.get_map().name.rsplit('/', 1)[-1].startswith('Town04'):
            raise SystemExit('Please load Town04 first. This command does not change maps.')
        existing = bridges(args.host, args.port)
        if len(existing) > 1:
            raise SystemExit('Multiple SUMO bridges found. Stop duplicate tick owners first.')
        if not existing:
            if world.get_settings().synchronous_mode:
                raise SystemExit('World already synchronous but no local SUMO bridge found. Use --no-sumo only if an external bridge owns ticks.')
            if args.host not in ('127.0.0.1', 'localhost'):
                raise SystemExit('Auto-start supports a local CARLA server. Start the remote SUMO bridge separately and use --no-sumo.')
            sumo = Path(os.environ.get('SUMO_HOME', str(Path.home() / 'sumo-1.22.0')))
            if not (sumo / 'bin/sumo-gui').is_file():
                raise SystemExit('SUMO not found. Set SUMO_HOME to its installation directory.')
            env = dict(os.environ, SUMO_HOME=str(sumo))
            env['PATH'] = str(sumo / 'bin') + ':' + env['PATH']
            script = PROJECT / 'carla-source/Co-Simulation/Sumo/spawn_npc_sumo.py'
            with (folder / 'sumo.log').open('w') as log:
                process = subprocess.Popen([str(Path.home() / 'carla-venv/bin/python'), '-u',
                    str(script), '--host', args.host, '--port', str(args.port), '-n', str(args.npcs),
                    '--sumo-gui', '--sumo-auto-start', '--sync-vehicle-color',
                    '--step-length', '0.05', '--tls-manager', 'carla'],
                    cwd=script.parent, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            state.write_text(json.dumps(dict(pid=process.pid)))
            existing = [process.pid]
            print(f'Started shared SUMO bridge {process.pid}; log: {folder / "sumo.log"}', flush=True)
        deadline = time.monotonic() + 90
        last_frame = world.get_snapshot().frame
        advancing_frames = 0
        while time.monotonic() < deadline:
            if not bridges(args.host, args.port):
                raise SystemExit(f'SUMO bridge exited. See {folder / "sumo.log"}')
            frame = world.get_snapshot().frame
            settings = world.get_settings()
            if settings.synchronous_mode and frame > last_frame:
                advancing_frames += 1
            else:
                advancing_frames = 0
            last_frame = frame
            if advancing_frames >= 3:
                print(f'Reusing SUMO bridge {existing[0]}. It alone advances simulation.', flush=True)
                return
            time.sleep(.5)
        raise SystemExit(f'SUMO startup timed out. See {folder / "sumo.log"}')


if __name__ == '__main__':
    main()
