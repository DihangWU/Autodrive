"""Two real Veins RSUs, plus empty/invalid lists, with isolated result files."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import csv
import json
import subprocess
import tempfile
from vanet.common.config import load_config

from cobridge.paths import OMNET_DIR as ROOT


def main():
    config = load_config()
    config['rsus'] = [dict(config['rsus'][0], id='rsu0'),
                      dict(config['rsus'][0], id='rsu1', x=235, y=-150, z=30)]
    with tempfile.TemporaryDirectory(prefix='vanet-multi-rsu-') as tmp:
        tmp = Path(tmp)
        path = tmp / 'config.json'
        for name, rsus in (('two', config['rsus']), ('empty', [])):
            cfg = dict(config, rsus=rsus)
            path.write_text(json.dumps(cfg))
            results = tmp / name
            results.mkdir()
            command = ['bash', str(ROOT / 'run_omnet.sh'), '--vanet-config', str(path),
                       '-u', 'Cmdenv', '-c', 'Smoke', '--sim-time-limit=10s',
                       '--result-dir=' + str(results), '--**.appl.csvDir=' + json.dumps(str(results))]
            run = subprocess.run(command, capture_output=True, text=True, timeout=30)
            assert run.returncode == 0, run.stdout + run.stderr
            rows = []
            for file in results.glob('*.csv'):
                with file.open() as stream:
                    rows.extend(csv.DictReader(stream))
            for p in rsus:
                assert list(results.glob(p['id'] + '-*.csv')), 'Missing RSU module output'
                assert any(r['event'] == 'RX' and r['receiver'] == p['id'] and r['sender'].startswith('lincoln') for r in rows)
                assert any(r['event'] == 'RX' and r['sender'] == p['id'] and r['receiver'].startswith('lincoln') and r['path'] == 'relay' for r in rows), p['id'] + ' relay reception missing'
            if not rsus:
                assert not list(results.glob('rsu*.csv'))
                assert any(r['event'] == 'RX' and r['sender'].startswith('lincoln')
                           and r['receiver'].startswith('lincoln') for r in rows), 'No V2V without RSUs'
            print('PASS:', name, 'RSU configuration; separate nodes and wireless results.')
        for invalid in ([config['rsus'][0], config['rsus'][0]],
                        [dict(config['rsus'][0], id='lincoln0')],
                        [dict(config['rsus'][0], x=float('nan'))]):
            path.write_text(json.dumps(dict(config, rsus=invalid)))
            try:
                load_config(path)
            except ValueError:
                continue
            raise AssertionError('Invalid RSU configuration accepted')
        print('PASS: duplicate names, conflicting names and nonfinite coordinates rejected.')


if __name__ == '__main__':
    main()
