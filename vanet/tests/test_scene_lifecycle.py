"""Real isolated SUMO POI + launcher cleanup test; no connection to live CARLA."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import json
import os
import signal
import subprocess
import tempfile
import time
from types import SimpleNamespace

from cobridge.paths import PROJECT_DIR as ROOT, OMNET_DIR
sys.path.insert(0, str(Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0')) / 'tools'))
import traci
from cobridge.bridge.shared_sumo import RequestService, control_folder, atomic_json, identity
from vanet.common.config import load_config, sumo_rsu, rsu_definitions
from cobridge.bridge.scene_markers import SceneMarkers


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
    traci.start([str(binary), '-n', str(ROOT / 'cobridge/carla-source-official/Co-Simulation/Sumo/examples/net/Town04.net.xml'),
                 '--no-step-log', 'true', '--step-length', '.05'])
    child = None
    omnet = None
    atomic_json(folder / 'ready.json', dict(owner=identity()))
    try:
        with tempfile.TemporaryDirectory(prefix='vanet-scene-test-') as tmp:
            tmp = Path(tmp)
            config_path = tmp / 'two-rsus.json'
            config_path.write_text(json.dumps(cfg))
            def advance():
                service.process()
                scene.poll()
                traci.simulationStep()
                world.frame += 1
                time.sleep(.01)

            argv_file, exit_file = tmp / 'omnet-argv.json', tmp / 'omnet-exit'
            code = ('import json,sys,time;from pathlib import Path;'
                    'Path(sys.argv[1]).write_text(json.dumps(sys.argv[3:]));'
                    '\nwhile not Path(sys.argv[2]).exists(): time.sleep(.05)')
            omnet_log = (tmp / 'omnet.log').open('w')
            omnet = subprocess.Popen([sys.executable, str(OMNET_DIR / 'runtime/omnet_session.py'),
                                      '--carla-host', host, '--vanet-config', str(config_path),
                                      sys.executable, '-c', code, str(argv_file), str(exit_file)],
                                     stdout=omnet_log, stderr=omnet_log)
            deadline = time.monotonic() + 10
            while not argv_file.exists():
                advance()
                assert omnet.poll() is None, (tmp / 'omnet.log').read_text()
                assert time.monotonic() < deadline
            passed = json.loads(argv_file.read_text())
            assert '--*.manager.rsuDefinitions=' + json.dumps('') in passed
            assert not scene.markers and not scene.lease.exists()
            print('PASS: opening OMNeT++ alone creates no RSU.')

            for mode in ('interrupt', 'terminate', 'crash'):
                log = (tmp / (mode + '.log')).open('w')
                command = [sys.executable, str(ROOT / 'carla/rsu/rsu_session.py'),
                           '--carla-host', host, '--vanet-config', str(config_path)]
                child = subprocess.Popen(command, stdout=log, stderr=log)
                deadline = time.monotonic() + 10
                while scene.session is None:
                    advance()
                    assert child.poll() is None, (tmp / (mode + '.log')).read_text()
                    assert time.monotonic() < deadline, 'RSU creation timeout'
                for p in cfg['rsus']:
                    assert traci.poi.getPosition(p['id']) == sumo_rsu(cfg, p)[:2]
                    assert traci.poi.getType(p['id']) == 'RSU'
                assert scene.network_nodes() == [(p['id'], *sumo_rsu(cfg, p)) for p in cfg['rsus']]
                assert len(scene.markers) == 2
                assert set(scene.bodies) == {p['id'] for p in cfg['rsus']}
                for p in cfg['rsus']:
                    actors = scene.bodies[p['id']].actors
                    assert actors and all(a.collisions and not a.physics and not a.gravity for a in actors)
                    bottom, top = actors[0], actors[-1]
                    assert abs(bottom.transform.location.z - bottom.bounding_box.extent.z - p['base_z']) < 1e-5
                    assert abs(top.transform.location.z + top.bounding_box.extent.z - p['z']) < 1e-5
                assert world.draws and all(d['life_time'] == .1 for d in world.draws)
                scene.poll()
                count = len(world.draws)
                scene.poll()
                assert len(world.draws) == count
                duplicate = subprocess.run(command, capture_output=True, text=True, timeout=3)
                assert duplicate.returncode != 0 and 'RSU 命令已在运行' in duplicate.stderr
                if mode == 'terminate':
                    # OMNeT++ exits while the independent RSU command stays alive.
                    exit_file.touch()
                    omnet.wait(timeout=3)
                    advance()
                    assert len(scene.markers) == 2 and child.poll() is None
                    print('PASS: closing OMNeT++ does not remove independently owned RSUs.')
                if mode == 'interrupt':
                    child.send_signal(signal.SIGINT)
                elif mode == 'terminate':
                    child.send_signal(signal.SIGTERM)
                else:
                    child.kill()
                deadline = time.monotonic() + 8
                while child.poll() is None:
                    advance()
                    assert time.monotonic() < deadline, 'RSU command cleanup timeout'
                scene.poll()
                assert not set(p['id'] for p in cfg['rsus']).intersection(traci.poi.getIDList()), f'{mode}: stale SUMO POI'
                assert scene.session is None and not scene.network_nodes()
                assert not scene.bodies and not any(a.is_alive for a in world.actors)
                if mode == 'interrupt':
                    assert omnet.poll() is None, 'RSU stop unexpectedly closes OMNeT++'
                log.close()
                print(f'PASS: {mode} exit removes both RSUs independently of OMNeT++.')
            omnet_log.close()
            # Fail halfway through building a mast: previously spawned pieces
            # must be removed, and the launcher must receive a failure reply.
            lease = dict(session='partial-failure', owner=identity(os.getpid()),
                         config=cfg)
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
        if omnet and omnet.poll() is None:
            omnet.terminate()
            omnet.wait()
        scene.clear()
        traci.close()
        for path in folder.iterdir():
            path.unlink()
        folder.rmdir()


if __name__ == '__main__':
    main()
