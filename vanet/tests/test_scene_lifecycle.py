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

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.paths import PROJECT_DIR as ROOT, LINCOLN_DIR, VANET_DIR
sys.path[:0] = [str(LINCOLN_DIR), str(Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0')) / 'tools')]
import traci
from shared_sumo import RequestService, control_folder
from vanet.common.config import load_config, sumo_rsu, rsu_definitions
from vanet.scene.markers import SceneMarkers


class World:
    def __init__(self):
        self.frame = 0
        self.draws = []
        self.actors = []
        self.fail_after = None
        def draw(*args, **kwargs):
            self.draws.append(kwargs)
        self.debug = SimpleNamespace(draw_line=draw, draw_box=draw, draw_string=draw)

    def get_blueprint_library(self):
        class Blueprint(dict):
            def set_attribute(self, name, value):
                self[name] = value
        return SimpleNamespace(find=lambda _: Blueprint())

    def spawn_actor(self, bp, transform):
        if self.fail_after is not None and len(self.actors) >= self.fail_after:
            raise RuntimeError('injected spawn failure')
        class Actor:
            is_alive = True
            def set_simulate_physics(self, value):
                self.physics = value
            def set_enable_gravity(self, value):
                self.gravity = value
            def set_collisions(self, value):
                self.collisions = value
            def destroy(self):
                self.is_alive = False
                return True
        a = Actor()
        a.transform = transform
        a.role = bp['role_name']
        size = float(bp['scale']) / 2
        a.bounding_box = SimpleNamespace(extent=SimpleNamespace(x=size, y=size, z=size))
        self.actors.append(a)
        return a

    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(delta_seconds=.05))

    def get_map(self):
        return SimpleNamespace(name='Town04', get_waypoint=lambda *a, **kw:
                               SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(z=0))))


def main():
    cfg = load_config()
    cfg['rsus'].append(dict(cfg['rsus'][0], id='rsu_test_second', x=235.0, y=-150.0))
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
            config_path = tmp / 'two-rsus.json'
            config_path.write_text(json.dumps(cfg))
            for mode in ('normal', 'terminate', 'crash'):
                argv_file, exit_file = tmp / (mode + '.json'), tmp / (mode + '.exit')
                code = ('import json,sys,time;from pathlib import Path;'
                        'Path(sys.argv[1]).write_text(json.dumps(sys.argv[3:]));'
                        '\nwhile not Path(sys.argv[2]).exists(): time.sleep(.05)')
                log = (tmp / (mode + '.log')).open('w')
                child = subprocess.Popen([sys.executable, str(VANET_DIR / 'runtime/omnet_session.py'),
                                          '--carla-host', host, '--vanet-config', str(config_path), sys.executable, '-c', code,
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
                for p in cfg['rsus']:
                    assert traci.poi.getPosition(p['id']) == sumo_rsu(cfg, p)[:2]
                    assert traci.poi.getType(p['id']) == 'RSU'
                assert len(scene.markers) == 2
                assert set(scene.bodies) == {p['id'] for p in cfg['rsus']}
                for p in cfg['rsus']:
                    actors = scene.bodies[p['id']].actors
                    assert actors and all(a.collisions and not a.physics and not a.gravity for a in actors)
                    bottom, top = actors[0], actors[-1]
                    assert abs(bottom.transform.location.z - bottom.bounding_box.extent.z - p['base_z']) < 1e-5
                    assert abs(top.transform.location.z + top.bounding_box.extent.z - p['z']) < 1e-5
                passed = json.loads(argv_file.read_text())
                assert '--*.manager.rsuDefinitions=' + json.dumps(rsu_definitions(cfg)) in passed
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
                assert not set(p['id'] for p in cfg['rsus']).intersection(traci.poi.getIDList()), f'{mode}: stale SUMO POI'
                assert scene.session is None
                assert not scene.bodies and not any(a.is_alive for a in world.actors)
                if orphan:
                    os.kill(orphan, signal.SIGTERM)
                    orphan = None
                log.close()
                print(f'PASS: {mode} exit removes both RSUs; JSON coordinates agree across launcher / SUMO / CARLA drawing.')
            # Fail halfway through building a mast: previously spawned pieces
            # must be removed, and the launcher must receive a failure reply.
            from shared_sumo import identity
            lease = dict(session='partial-failure', owner=identity(os.getpid()),
                         child=identity(os.getpid()), config=cfg)
            scene.lease.write_text(json.dumps(lease))
            world.fail_after = len(world.actors) + 2
            scene.poll()
            assert 'injected spawn failure' in json.loads(scene.reply.read_text())['error']
            assert not scene.bodies and not any(a.is_alive for a in world.actors)
            assert not set(p['id'] for p in cfg['rsus']).intersection(traci.poi.getIDList())
            scene.lease.unlink()
            print('PASS: partial CARLA spawn failure rolls back all physical actors.')
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
