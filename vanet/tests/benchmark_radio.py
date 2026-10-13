"""Compare hard-range filtering paths with 200 real Veins OBUs + one RSU.

Synthetic snapshots on an isolated TCP port; never connects to live CARLA/SUMO.
Both runs disable event logging. Timing includes process startup and teardown.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import csv
import json
import logging
import resource
import tempfile
import time

from test_radio_range import load_config, run_case


def packets(folder):
    rows = []
    for path in folder.glob('*.csv'):
        with path.open() as stream:
            rows.extend(tuple(row) for row in csv.reader(stream) if row[0] != 'time')
    return sorted(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nodes', type=int, default=200)
    parser.add_argument('--duration', type=float, default=3)
    args = parser.parse_args()
    if not 2 <= args.nodes <= 200 or args.duration <= 0:
        parser.error('nodes must be 2..200 and duration must be positive')
    logging.basicConfig(level=logging.ERROR)
    cfg = dict(load_config(), rsus=[])
    points = [(560+15*(i%20), 560+15*(i//20)) for i in range(args.nodes)]
    measurements, traces = [], []
    with tempfile.TemporaryDirectory(prefix='vanet-benchmark-') as folder:
        root = Path(folder)
        for early in (False, True):
            name = 'early' if early else 'reference'
            visuals = []
            start = time.perf_counter()
            cpu = resource.getrusage(resource.RUSAGE_CHILDREN)
            rows, _, _ = run_case(root, name, cfg, lambda _: points, duration=args.duration,
                rsu_provider=lambda _: [('rsu0',700,625,60)], roles=['sumo_driver']*args.nodes,
                vehicle_names={101+i: f'npc_bench_{i}' for i in range(args.nodes)},
                visualization=visuals, timeout=max(120, args.duration*60),
                extra_args=['--cmdenv-express-mode=true', '--record-eventlog=false',
                            '--**.nic.phy80211p.earlyRangeFilter='+str(early).lower()])
            finish = resource.getrusage(resource.RUSAGE_CHILDREN)
            measurements.append(dict(path=name, nodes=args.nodes, sim_seconds=args.duration,
                wall_seconds=round(time.perf_counter()-start,3),
                cpu_seconds=round(finish.ru_utime+finish.ru_stime-cpu.ru_utime-cpu.ru_stime,3),
                received=len(rows)))
            # Include HELLO/ACK, data TX/RX, sequence, timestamps and delays.
            traces.append((packets(root/name), [(v['frame'],v['time'],v['links']) for v in visuals]))
            print(json.dumps(measurements[-1]), flush=True)
        assert traces[0] == traces[1], 'Filtering changed packet traces or handshake/activity feedback'
        assert measurements[-1]['received'] > 0
        assert any(v[2] for v in traces[-1][1]), 'No confirmed animation links'
        print('PASS: every packet CSV row and every frame of handshake/color feedback is identical.')
        print('CPU speedup: %.2fx (reference / early filtering)' %
              (measurements[0]['cpu_seconds']/measurements[1]['cpu_seconds']))


if __name__ == '__main__':
    main()
