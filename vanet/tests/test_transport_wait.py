"""Keep one Veins run alive across a slow, fragmented RSU removal snapshot."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import json
import socket
import subprocess
import tempfile
import time

from vanet.common.config import load_config
from cobridge.common.paths import OMNET_DIR


def main():
    with tempfile.TemporaryDirectory(prefix='vanet-wait-') as folder, socket.socket() as listener:
        folder = Path(folder)
        config = folder / 'config.json'
        config.write_text(json.dumps(dict(load_config(), rsus=[])))
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(10)
        log = (folder / 'network.log').open('w')
        command = ['bash', str(OMNET_DIR / 'run_omnet.sh'), '--no-scene-markers',
                   '--vanet-config', str(config), '-u', 'Cmdenv', '--sim-time-limit=7s',
                   f'--*.manager.port={listener.getsockname()[1]}',
                   '--result-dir=' + str(folder), '--**.appl.csvDir=' + json.dumps(str(folder))]
        child = subprocess.Popen(command, stdout=log, stderr=log)
        states = []
        try:
            with listener.accept()[0] as peer:
                peer.settimeout(20)
                reader = peer.makefile('r')
                for frame in range(1, 142):
                    # Remove, re-add, then remove the same RSU in one run.
                    has_rsu = frame < 40 or 80 <= frame < 110
                    header = f'FRAME2 {frame} 0.05 2 {int(has_rsu)}\n'
                    cars = '101 lincoln0 685 625 2\n102 lincoln1 715 625 2\n'
                    if frame == 40:
                        peer.sendall(header[:9].encode())
                        time.sleep(.05)
                        peer.sendall((header[9:] + cars.splitlines(True)[0]).encode())
                        # Exceeds the old socket's fatal 10-second timeout.
                        time.sleep(11)
                        assert child.poll() is None, 'Network quit during scene cleanup'
                        peer.sendall(cars.splitlines(True)[1].encode())
                    else:
                        peer.sendall((header + cars + ('rsu0 700 625 60\n' if has_rsu else '')).encode())
                    line = reader.readline()
                    if not line:  # The 7s time limit precedes the final ACK.
                        assert frame == 141, (frame, line)
                        break
                    fields = line.split()
                    assert fields[:2] == ['VIS', str(frame)], fields
                    assert abs(float(fields[2]) - frame * .05) < 1e-8, fields
                    links = [reader.readline().split() for _ in range(int(fields[3]))]
                    assert reader.readline().strip() == f'ACK {frame}'
                    if not has_rsu:
                        assert all('rsu0' not in row[:2] for row in links), links
                    states.append((frame, links))
            assert child.wait(timeout=5) == 0, (folder / 'network.log').read_text()
            assert len(states) == 140, len(states)
            for first, last in [(1, 39), (80, 109)]:
                assert any('rsu0' in link[:2] for f, links in states if first <= f <= last for link in links)
            for first, last in [(40, 79), (110, 139)]:
                assert any(set(link[:2]) == {'lincoln0', 'lincoln1'}
                           for f, links in states if first <= f <= last for link in links)
            print('PASS: 11-second partial RSU removal, re-add/removal, continuous V2V, '
                  'exactly one ACK per frame and no skipped simulation time.')
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            log.close()


if __name__ == '__main__':
    main()
