"""Prepare the optional endpoint, then invoke the existing Lincoln launcher."""
import argparse
import json
import os
import gc
import signal
import subprocess
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'lincoln'))
sys.path.insert(0, str(PROJECT))

def main():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    position = p.add_mutually_exclusive_group()
    position.add_argument('--vanet-slot', choices=['auto', 'A', 'B'], default='auto')
    position.add_argument('--pose', type=float, nargs=4)
    position.add_argument('--spawn-index', type=int)
    args, extra = p.parse_known_args()
    if '--help' in sys.argv or '-h' in sys.argv:
        print('VANET: --vanet-slot auto|A|B selects a fixed Town04 slot; all other options use the root launcher.', flush=True)
        os.execvp('bash', ['bash', str(PROJECT/'run_lincoln.sh'), '--help'])
    from ensure_sumo import bridges
    import carla
    cfg = json.loads(Path(__file__).with_name('town04.json').read_text())
    os.chdir(PROJECT)
    c = carla.Client(args.host,args.port)
    c.set_timeout(30)
    world = c.get_world()
    from vanet.spawn import reserve_slot
    slot_name = 'custom'
    if args.pose is not None:
        position_args = ['--pose', *map(str, args.pose)]
    elif args.spawn_index is not None:
        position_args = ['--spawn-index', str(args.spawn_index)]
    else:
        # Resolve the experiment slot here; ego_vehicle.py only sees its existing --pose option.
        slot_name, transform = reserve_slot(world, args.host, args.port,
                                           None if args.vanet_slot == 'auto' else args.vanet_slot)
        position_args = ['--pose', str(transform.location.x), str(transform.location.y),
                         str(transform.location.z), str(transform.rotation.yaw)]
    os.environ['VANET_PORT'] = str(cfg['network']['port'])
    active = bridges(args.host, args.port)
    expected = ('VANET_PORT=' + str(cfg['network']['port'])).encode()
    if active and expected not in Path('/proc', str(active[0]), 'environ').read_bytes().split(b'\0'):
        raise SystemExit('Existing SUMO bridge has no matching VANET endpoint. Close vehicle clients, then run bash run_lincoln.sh --stop-sumo and retry. No bridge was stopped automatically.')
    print(f'VANET junction 924, slot={slot_name}, position={position_args}; OBU name follows SUMO lincoln ID.', flush=True)
    # CARLA's C++ sockets must not survive an exec into the vehicle process.
    # Release the query client, then use close_fds in the child. This parent
    # remains alive to own its slot reservation for the vehicle's lifetime.
    del world, c
    gc.collect()
    child = subprocess.Popen(['bash', str(PROJECT/'run_lincoln.sh'),
                              '--host', args.host, '--port', str(args.port),
                              *position_args, *extra], close_fds=True)
    def stop_child(signum, frame):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT, stop_child)
    signal.signal(signal.SIGTERM, stop_child)
    raise SystemExit(child.wait())

if __name__ == '__main__':
    main()
