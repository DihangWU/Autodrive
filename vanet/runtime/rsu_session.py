"""Own configured CARLA / SUMO / OMNeT++ RSUs independently of OMNeT++."""
import argparse
import fcntl
import json
from pathlib import Path
import signal
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.paths import LINCOLN_DIR
sys.path.insert(0, str(LINCOLN_DIR))
from shared_sumo import alive, atomic_json, control_folder, identity, request
from vanet.common.config import CONFIG, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--carla-host', default='127.0.0.1')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--vanet-config', default=str(CONFIG))
    args = parser.parse_args()
    cfg = load_config(args.vanet_config)
    if not cfg['rsus']:
        raise RuntimeError('town04.json 的 rsus 列表为空，没有可生成的 RSU。')
    folder = control_folder(args.carla_host, args.carla_port)
    lease, reply = folder / 'vanet-rsu.json', folder / 'vanet-rsu-status.json'
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.set())
    session = uuid.uuid4().hex
    with (folder / 'vanet-rsu-session.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('RSU 命令已在运行；请先在原终端按 Ctrl+C 退出。')
        status = request(args.carla_host, args.carla_port, {'action': 'status'}, timeout=8)
        if status.get('vanet_scene', {}).get('protocol') != 3:
            raise RuntimeError('共享 SUMO 桥仍是旧版本。请正常关闭车辆后，执行 bash run_lincoln.sh --stop-sumo，再重新启动车辆。')
        try:
            atomic_json(lease, dict(session=session, owner=identity(), config=cfg))
            deadline = time.monotonic() + 10
            while not stopped.is_set():
                state = json.loads(reply.read_text()) if reply.exists() else {}
                if state.get('session') == session:
                    if not state['ok']:
                        raise RuntimeError(state['error'])
                    print('RSU 已创建：' + ', '.join(p['id'] for p in cfg['rsus']) +
                          '；Ctrl+C 移除，OMNeT++ 可以继续运行。', flush=True)
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('共享桥未确认 RSU 创建；请检查 SUMO 和共享桥日志。')
                stopped.wait(.1)
            while not stopped.wait(.2):
                ready = folder / 'ready.json'
                if not ready.exists() or not alive(json.loads(ready.read_text()).get('owner')):
                    raise RuntimeError('共享桥已退出，RSU 会话结束。')
        finally:
            if lease.exists() and json.loads(lease.read_text()).get('session') == session:
                lease.unlink()
            # The bridge's idle callback also cleans up while Qtenv is paused.
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                state = json.loads(reply.read_text()) if reply.exists() else {}
                if state.get('session') != session or not (folder / 'ready.json').exists():
                    break
                time.sleep(.1)
            print('RSU 命令已退出；共享桥清理实体、SUMO 图标及网络节点。', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError) as error:
        raise SystemExit(str(error))
