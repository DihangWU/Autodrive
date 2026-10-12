"""Real isolated SUMO POI + launcher cleanup test; no connection to live CARLA."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'lincoln'), str(Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0')) / 'tools')]
import traci
from shared_sumo import RequestService, control_folder
from vanet.config import load_config, sumo_rsu
from vanet.scene import SceneMarkers


class World:
    def __init__(self):
        self.frame = 0
        self.draws = []
        def draw(*args, **kwargs):
            self.draws.append(kwargs)
        self.debug = SimpleNamespace(draw_line=draw, draw_box=draw, draw_string=draw)

    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(delta_seconds=.05))

    def get_map(self):
        return SimpleNamespace(name='Town04', get_waypoint=lambda *a, **kw:
                               SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(z=0))))


def main():
    cfg = load_config()
    world = World()
    host = 'vanet-test-' + str(os.getpid())
    folder = control_folder(host, 2000)
    scene = SceneMarkers(world, host, 2000)
    service = RequestService(host, 2000, None, None, lambda: dict(vanet_scene=scene.status()))
    binary = Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0')) / 'bin/sumo'
    traci.start([str(binary), '-n', str(ROOT / 'carla-source/Co-Simulation/Sumo/examples/net/Town04.net.xml'),
                 '--no-step-log', 'true', '--step-length', '.05'])
    child = None
    orphan = None
    try:
        with tempfile.TemporaryDirectory(prefix='vanet-scene-test-') as tmp:
            tmp = Path(tmp)
            for mode in ('normal', 'terminate', 'crash'):
                argv_file, exit_file = tmp / (mode + '.json'), tmp / (mode + '.exit')
                code = ('import json,sys,time;from pathlib import Path;'
                        'Path(sys.argv[1]).write_text(json.dumps(sys.argv[3:]));'
                        '\nwhile not Path(sys.argv[2]).exists(): time.sleep(.05)')
                log = (tmp / (mode + '.log')).open('w')
                child = subprocess.Popen([sys.executable, str(ROOT / 'vanet/omnet_session.py'),
                                          '--carla-host', host, sys.executable, '-c', code,
                                          str(argv_file), str(exit_file)], stdout=log, stderr=log)
                def advance():
                    service.process()
                    scene.poll()
                    traci.simulationStep()
                    world.frame += 1
                    time.sleep(.01)
                deadline = time.monotonic() + 10
                while scene.session is None or not argv_file.exists():
                    advance()
                    assert child.poll() is None, (tmp / (mode + '.log')).read_text()
                    assert time.monotonic() < deadline, 'Marker creation timeout'
                assert traci.poi.getPosition(scene.POI) == sumo_rsu(cfg)[:2]
                assert traci.poi.getType(scene.POI) == 'RSU'
                passed = json.loads(argv_file.read_text())
                for axis, value in zip('xyz', sumo_rsu(cfg)):
                    assert f'--*.rsu.veinsmobility.{axis}={value}' in passed
                assert world.draws and all(d['life_time'] == .1 for d in world.draws)
                count = len(world.draws)
                scene.poll()  # One new frame since last advance; subsequent same-frame calls don't redraw.
                count = len(world.draws)
                scene.poll()
                assert len(world.draws) == count
                if mode == 'normal':
                    exit_file.touch()
                elif mode == 'terminate':
                    child.send_signal(signal.SIGTERM)
                else:
                    orphan = json.loads(scene.lease.read_text())['child']['pid']
                    child.kill()
                child.wait(timeout=5)
                scene.poll()
                assert scene.POI not in traci.poi.getIDList(), f'{mode}: stale SUMO POI'
                assert scene.session is None
                if orphan:
                    os.kill(orphan, signal.SIGTERM)
                    orphan = None
                log.close()
                print(f'PASS: {mode} exit removes RSU; JSON coordinates agree across launcher / SUMO / CARLA drawing.')
    finally:
        if child and child.poll() is None:
            child.kill()
            child.wait()
        if orphan:
            os.kill(orphan, signal.SIGTERM)
        scene.clear()
        traci.close()
        for path in folder.iterdir():
            path.unlink()
        folder.rmdir()


if __name__ == '__main__':
    main()
