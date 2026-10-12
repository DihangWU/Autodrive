"""Exercise the real TCP/ACK adapter against Veins using synthetic actor snapshots.
This verifies synchronization and lifecycle, not CARLA road geometry.
"""
import csv
import logging
from pathlib import Path
import subprocess
import threading
import tempfile
import json
import time
from types import SimpleNamespace
from bridge import VanetBridge

ROOT = Path(__file__).resolve().parent

class World:
    def __init__(self):
        self.frame = 0
    def get_snapshot(self):
        self.frame += 1
        def find(actor):
            x,y = (196.82,-204.60) if actor==101 else (235.62,-176.69)
            return SimpleNamespace(get_transform=lambda: SimpleNamespace(location=SimpleNamespace(x=x,y=y,z=.2)))
        return SimpleNamespace(frame=self.frame,find=find)
    def get_actors(self):
        ids=[101] if 15<=self.frame<25 else [101,102]
        actors=[SimpleNamespace(id=i,attributes={'role_name':f'lincoln_test_{i}'}) for i in ids]
        return SimpleNamespace(filter=lambda pattern:actors)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    bridge=VanetBridge(0)
    port=bridge.listener.getsockname()[1]
    world=World()
    stop=threading.Event()
    def serve():
        while not stop.is_set():
            bridge.tick(world,.05)
            time.sleep(.001)
    worker=threading.Thread(target=serve,daemon=True)
    worker.start()
    output=tempfile.TemporaryDirectory(prefix='vanet-transport-')
    command=['bash',str(ROOT/'run_omnet.sh'),'--no-scene-markers','-u','Cmdenv','-c','Live','--sim-time-limit=2s',f'--*.manager.port={port}',
             '--result-dir='+output.name, '--**.appl.csvDir='+json.dumps(output.name)]
    try:
        started=time.time()
        completed=subprocess.run(command,capture_output=True,text=True,timeout=20)
        if completed.returncode:
            raise RuntimeError(completed.stdout+completed.stderr)
        assert world.frame>=35, world.frame
        rows=[]
        files=[f for f in Path(output.name).glob('*.csv') if f.stat().st_mtime>=started]
        assert len([f for f in files if f.name.startswith('lincoln1-')])==2, 'lincoln1 OBU was not recreated'
        for file in files:
            rows.extend(csv.DictReader(file.open()))
        assert any(r['event']=='RX' and r['receiver']=='rsu0' for r in rows)
        assert any(r['event']=='RX' and r['path']=='relay' and r['receiver'].startswith('lincoln') for r in rows)
        assert any(r['event']=='RX' and r['path']=='direct' and r['sender'].startswith('lincoln') and r['receiver'].startswith('lincoln') for r in rows)
        print(f'PASS: TCP frame ACK, {world.frame} snapshots, V2V, V2I, RSU relay and actor removal/recreation.')
    finally:
        stop.set()
        bridge.close()
        worker.join(timeout=2)
        output.cleanup()
