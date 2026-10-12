"""Actual Veins/TCP tests: independent nodes, hard radius edges and mobility."""
import csv
import json
import logging
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.config import load_config
from vanet.common.paths import VANET_DIR
from vanet.runtime.bridge import VanetBridge


class World:
    def __init__(self, cfg, positions):
        self.frame = 0
        self.cfg, self.positions = cfg, positions

    def get_snapshot(self):
        self.frame += 1
        positions = self.positions(self.frame)
        ox, oy = self.cfg['network']['offset_x'], self.cfg['network']['offset_y']
        def find(actor):
            x, y = positions[actor - 101]
            return SimpleNamespace(get_transform=lambda: SimpleNamespace(
                location=SimpleNamespace(x=x - ox, y=oy - y, z=.2)))
        return SimpleNamespace(frame=self.frame, find=find)

    def get_actors(self):
        actors = [SimpleNamespace(id=101 + i, attributes={'role_name': f'lincoln_range_{i}'})
                  for i in range(len(self.positions(self.frame)))]
        return SimpleNamespace(filter=lambda _: actors)


def run_case(tmp, name, cfg, positions, duration=6, rsu_provider=None):
    results = tmp / name
    results.mkdir()
    config = results / 'config.json'
    config.write_text(json.dumps(cfg))
    bridge = VanetBridge(0)
    world = World(cfg, positions)
    if rsu_provider is not None:
        bridge.rsu_provider = lambda: rsu_provider(world.frame)
    stop = threading.Event()
    errors = []
    def serve():
        try:
            while not stop.is_set():
                bridge.tick(world, .05)
                time.sleep(.001)
        except Exception as error:
            errors.append(error)
    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    command = ['bash', str(VANET_DIR / 'run_omnet.sh'), '--no-scene-markers',
               '--vanet-config', str(config), '-u', 'Cmdenv', '-c', 'Live',
               '--cmdenv-express-mode=false', '--cmdenv-event-banners=false',
               f'--sim-time-limit={duration}s',
               f'--*.manager.port={bridge.listener.getsockname()[1]}',
               '--result-dir=' + str(results), '--**.appl.csvDir=' + json.dumps(str(results))]
    try:
        run = subprocess.run(command, capture_output=True, text=True, timeout=30)
        assert run.returncode == 0, run.stdout + run.stderr
        assert not errors, errors
        rows = []
        for file in results.glob('*.csv'):
            with file.open() as stream:
                rows.extend(csv.DictReader(stream))
        scalars = '\n'.join(f.read_text() for f in results.glob('*.sca'))
        return [r for r in rows if r['event'] == 'RX'], run.stdout, scalars
    finally:
        stop.set()
        bridge.close()
        worker.join(timeout=2)


def received(rows, source, target, path='direct', origin=None):
    return [r for r in rows if r['sender'] == source and r['receiver'] == target
            and r['path'] == path and (origin is None or r['origin'] == origin)]


def main():
    logging.basicConfig(level=logging.ERROR)
    cfg = load_config()
    p = dict(id='rsu0', x=700 - cfg['network']['offset_x'],
             y=cfg['network']['offset_y'] - 625, z=60, base_z=.39, beacon_radius=5)
    with tempfile.TemporaryDirectory(prefix='vanet-radio-range-') as folder:
        tmp = Path(folder)
        no_rsu = dict(cfg, rsus=[])
        for name, distance, allowed in [('v2v_inside', 39.99, True), ('v2v_edge', 40, True),
                                        ('v2v_outside', 40.01, False)]:
            rows, log, scalars = run_case(tmp, name, no_rsu,
                                         lambda _, d=distance: [(700, 625), (700 + d, 625)])
            for source, target in [('lincoln0', 'lincoln1'), ('lincoln1', 'lincoln0')]:
                assert bool(received(rows, source, target)) == allowed, (name, source, rows)
            assert ('COVERAGE LINK CREATED lincoln0 <-> lincoln1' in log) == allowed
            if not allowed:
                assert any('outsideRangeFrames' in row and float(row.split()[-1]) > 0
                           for row in scalars.splitlines()), 'No PHY range filtering'
            print('PASS:', name, 'V2V without RSUs')

        with_rsu = dict(cfg, rsus=[p])
        for name, distance, allowed in [('v2i_inside', 219.99, True), ('v2i_edge', 220, True),
                                        ('v2i_outside', 220.01, False)]:
            rows, log, _ = run_case(tmp, name, with_rsu,
                                   lambda _, d=distance: [(700 + d, 625)])
            assert bool(received(rows, 'lincoln0', 'rsu0')) == allowed, (name, rows)
            assert bool(received(rows, 'rsu0', 'lincoln0')) == allowed, (name, rows)
            assert ('COVERAGE LINK CREATED rsu0 <-> lincoln0' in log) == allowed
            print('PASS:', name, 'single vehicle and RSU, both directions')

        rows, _, _ = run_case(tmp, 'relay_only', with_rsu, lambda _: [(580, 625), (820, 625)])
        assert not received(rows, 'lincoln0', 'lincoln1')
        assert not received(rows, 'lincoln1', 'lincoln0')
        for source, target in [('lincoln0', 'lincoln1'), ('lincoln1', 'lincoln0')]:
            assert received(rows, 'rsu0', target, 'relay', source), rows
        print('PASS: vehicles 240m apart communicate through their shared RSU')

        rows, _, _ = run_case(tmp, 'direct_and_relay', with_rsu, lambda _: [(685, 625), (715, 625)])
        for source, target in [('lincoln0', 'lincoln1'), ('lincoln1', 'lincoln0')]:
            assert received(rows, source, target)
            assert received(rows, 'rsu0', target, 'relay', source)
        print('PASS: V2V direct and RSU relay coexist')

        def moving(frame):
            distance = 30 if frame < 60 else (40.01 if frame < 120 else 20)
            return [(700, 625), (700 + distance, 625)]
        rows, log, _ = run_case(tmp, 'leave_and_return', no_rsu, moving, duration=9)
        for source, target in [('lincoln0', 'lincoln1'), ('lincoln1', 'lincoln0')]:
            packets = received(rows, source, target)
            assert any(float(r['time']) < 2.9 for r in packets)
            assert not any(3.05 <= float(r['time']) < 5.9 for r in packets)
            assert any(float(r['time']) >= 6.1 for r in packets)
        assert log.count('COVERAGE LINK CREATED lincoln0 <-> lincoln1') == 2
        assert 'COVERAGE LINK REMOVED' in log
        print('PASS: movement removes the link outside 40m and restores it on return')

        # Every added vehicle gets independent links; an uncovered third car
        # cannot receive RSU traffic or inject packets into that RSU's relay.
        rows, _, _ = run_case(tmp, 'three_vehicles', with_rsu,
                             lambda _: [(685, 625), (715, 625), (970, 625)])
        assert received(rows, 'lincoln0', 'lincoln1')
        assert received(rows, 'rsu0', 'lincoln1', 'relay', 'lincoln0')
        assert not any(r['sender'] == 'lincoln2' or r['receiver'] == 'lincoln2'
                       or r['origin'] == 'lincoln2' for r in rows)
        print('PASS: independent links for three vehicles; uncovered car is disconnected')


if __name__ == '__main__':
    main()
