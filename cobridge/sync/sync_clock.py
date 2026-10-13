"""Own CARLA's synchronous clock independently of vehicles and SUMO display."""
import argparse
import fcntl
import hashlib
import math
import os
from pathlib import Path
import signal
import tempfile
import threading
import time

import carla


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--step-length', type=float, default=.05)
    parser.add_argument('--duration', type=float, default=0,
                        help='Stop after this many wall seconds; 0 runs until Ctrl+C.')
    args = parser.parse_args()
    if not math.isfinite(args.step_length) or not 0 < args.step_length <= .1:
        parser.error('--step-length must be finite and in (0, 0.1].')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative.')

    # Hold this descriptor until settings have been restored. Different local
    # launchers cannot accidentally become two tick owners on the same server.
    host = 'localhost' if args.host in ('localhost', '127.0.0.1') else args.host
    key = hashlib.sha256(f'{os.getuid()}:{host}:{args.port}'.encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / f'autodrive-sync-{key}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(1, 'A synchronous clock is already running for this server.\n')
        client = carla.Client(args.host, args.port)
        client.set_timeout(10)
        world = client.get_world()
        original = world.get_settings()
        if original.synchronous_mode:
            parser.exit(1, 'CARLA is already synchronous; keep its existing tick owner.\n')
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.step_length
        if settings.substepping:
            settings.max_substeps = max(settings.max_substeps,
                                        math.ceil(args.step_length / settings.max_substep_delta_time))
        stop = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
        started = time.monotonic()
        changed = False
        frames = 0
        try:
            # Mark before the RPC: if it times out after applying, still restore.
            changed = True
            world.apply_settings(settings)
            print(f'SYNC_READY pid={os.getpid()} step={args.step_length} '
                  f'map={world.get_map().name}', flush=True)
            while not stop.is_set():
                if args.duration and time.monotonic() - started >= args.duration:
                    break
                begin = time.monotonic()
                world.tick(10.)
                frames += 1
                # Pace to at most real time, leaving clients time between ticks.
                # Slow frames are not followed by catch-up bursts.
                stop.wait(max(0., args.step_length - (time.monotonic() - begin)))
        finally:
            if changed:
                # Loading another map ends ownership of the old world's settings.
                if client.get_world().id == world.id:
                    world.apply_settings(original)
                    print(f'SYNC_STOPPED frames={frames}; previous world settings restored', flush=True)
                else:
                    print('SYNC_STOPPED; world changed, new world settings left untouched', flush=True)


if __name__ == '__main__':
    main()
