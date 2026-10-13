"""One real Veins run: add, remove, re-add and move RSUs while V2V continues."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import logging
import tempfile

from test_radio_range import run_case, received, load_config


def main():
    logging.basicConfig(level=logging.ERROR)
    cfg = dict(load_config(), rsus=[])
    def rsus(frame):
        if 60 <= frame < 120:
            return [('rsu0', 700, 625, 60)]
        if 180 <= frame < 240:
            return [('rsu0', 700, 625, 60), ('rsu1', 715, 650, 30)]
        if 240 <= frame < 300:
            return [('rsu0', 1100, 625, 60)]
        return []
    with tempfile.TemporaryDirectory(prefix='vanet-dynamic-rsu-') as folder:
        rows, log, _ = run_case(Path(folder), 'dynamic', cfg,
                               lambda _: [(685, 625), (715, 625)], duration=18,
                               rsu_provider=rsus)
        assert log.count('RSU CREATED rsu0 ') == 2
        assert log.count('RSU DELETED rsu0') == 2
        assert log.count('RSU CREATED rsu1 ') == 1
        assert log.count('RSU DELETED rsu1') == 1
        for source, target in [('lincoln0', 'lincoln1'), ('lincoln1', 'lincoln0')]:
            direct = received(rows, source, target)
            for start in (0, 3, 6, 9, 12, 15):
                assert any(start + .1 < float(r['time']) < start + 2.9 for r in direct), (source, target, start)
            relay = received(rows, 'rsu0', target, 'relay', source)
            assert any(3.1 < float(r['time']) < 5.9 for r in relay)
            assert any(9.1 < float(r['time']) < 11.9 for r in relay)
        for r in rows:
            if r['sender'] == 'rsu0' or r['receiver'] == 'rsu0':
                t = float(r['time'])
                assert (2.9 < t < 6) or (8.9 < t < 12), r
        assert any(r['sender'] == 'rsu1' and r['path'] == 'relay' for r in rows)
        assert 'COVERAGE LINK REMOVED' in log
        print('PASS: a single running OMNeT++ instance starts without RSUs, adds/removes/re-adds multiple RSUs,')
        print('      updates their coordinates and coverage, and preserves V2V throughout all six phases.')


if __name__ == '__main__':
    main()
