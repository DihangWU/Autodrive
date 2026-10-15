"""持有唯一的 TraCI 连接，串行处理模块请求。当前只管理地图和 POI。"""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PROJECT_DIR = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_DIR))
from simuenv.sumo.traci.client import RUNTIME_DIR, SOCKET_FILE, owner_alive, request


def start_sumo(config_file, headless=False):
    """启动后台管理端，等待就绪；同一配置已运行时直接复用。"""
    try:
        status = request({'action': 'status'}, timeout=1)
    except RuntimeError:
        status = None
    if status:
        if status['config'] != str(config_file) or status.get('headless', False) != headless:
            raise RuntimeError('SUMO 已使用其他配置或显示模式，请先关闭旧管理端。')
        print(f'复用 SUMO 管理端（PID {status["pid"]}）：{config_file}')
        return

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    log_file = RUNTIME_DIR / 'service.log'
    command = [sys.executable, '-u', str(Path(__file__).resolve()), str(config_file)]
    if headless:
        command.append('--headless')
    with log_file.open('a') as log:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            status = request({'action': 'status'}, timeout=1)
        except RuntimeError:
            status = None
        if status:
            if status['config'] != str(config_file):
                raise RuntimeError('另一个 SUMO 管理端已加载其他配置。')
            print(f'加载 SUMO 配置：{os.path.relpath(config_file, PROJECT_DIR.parent)}')
            print(f'TraCI 管理端已就绪（PID {status["pid"]}），终端可继续使用。')
            print(f'管理日志：{log_file.relative_to(PROJECT_DIR.parent)}')
            return
        if process.poll() is not None:
            break
        time.sleep(.1)
    if process.poll() is None:
        process.terminate()
    raise RuntimeError(f'SUMO 管理端启动失败，请查看 {log_file}')


def read_network(config_file):
    """从实际 SUMO 配置及路网读取地图身份和坐标偏移，不手填偏移。"""
    config = ET.parse(config_file).getroot()
    value = config.find('input/net-file').get('value')
    network_file = (config_file.parent / value).resolve()
    network = ET.parse(network_file).getroot()
    offset = [float(number) for number in network.find('location').get('netOffset').split(',')]
    map_name = network_file.name.removesuffix('.net.xml')
    xodr_file = network_file.with_name(map_name + '.xodr')
    fingerprint = hashlib.sha256(xodr_file.read_text().strip().encode()).hexdigest() if xodr_file.is_file() else None
    return dict(config=str(config_file), network=str(network_file), map=map_name,
                offset=offset, xodr_sha256=fingerprint)


class SumoService:
    def __init__(self, connection, network):
        self.connection = connection
        self.network = network
        self.sessions = {}
        self.stopped = False

    def remove_session(self, session):
        owned = self.sessions.get(session, {})
        existing = set(self.connection.poi.getIDList())
        for name in owned.get('ids', []):
            if name in existing:
                self.connection.poi.remove(name, layer=100)
        self.sessions.pop(session, None)

    def handle(self, command):
        action = command['action']
        if action == 'status':
            pois = list(self.connection.poi.getIDList())
            return dict(self.network, pid=os.getpid(), pois=pois,
                        poi_positions={name: self.connection.poi.getPosition(name) for name in pois},
                        sessions=list(self.sessions), time=self.connection.simulation.getTime())
        if action == 'stop':
            self.stopped = True
            return 'SUMO 管理端正在关闭。'
        if action == 'add_pois':
            session, pois = command['session'], command['pois']
            if session in self.sessions:
                raise ValueError('该 RSU 会话已经注册。')
            if not owner_alive(command['owner']):
                raise ValueError('RSU 请求进程已经退出。')
            names = [poi['id'] for poi in pois]
            if len(names) != len(set(names)) or set(names) & set(self.connection.poi.getIDList()):
                raise ValueError('RSU ID 重复或已被 SUMO 中的其他对象使用。')
            self.sessions[session] = dict(owner=command['owner'], ids=[])
            try:
                for poi in pois:
                    self.connection.poi.add(
                        poi['id'], poi['x'], poi['y'], (255, 255, 255, 255),
                        poiType='RSU', layer=100, imgFile=poi['image'], width=8, height=8)
                    self.sessions[session]['ids'].append(poi['id'])
                    self.connection.poi.setParameter(poi['id'], 'name', poi['id'])
            except Exception:
                self.remove_session(session)
                raise
            return names
        if action == 'remove_pois':
            self.remove_session(command['session'])
            return 'RSU POI 已清理。'
        raise ValueError(f'未知 SUMO 请求：{action}')

    def poll(self):
        # 不调用 simulationStep：编辑 POI 不需要接管联仿时钟。
        self.connection.getVersion()
        for session, state in list(self.sessions.items()):
            if not owner_alive(state['owner']):
                self.remove_session(session)


def serve(config_file, headless=False):
    sumo_home = os.environ.get('SUMO_HOME')
    if not sumo_home:
        raise RuntimeError('请先设置 SUMO_HOME，使管理端能导入 tools/traci。')
    sys.path.insert(0, str(Path(sumo_home) / 'tools'))
    import traci

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    with (RUNTIME_DIR / 'service.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        SOCKET_FILE.unlink(missing_ok=True)
        network = read_network(config_file)
        network['headless'] = headless
        binary = 'sumo' if headless else 'sumo-gui'
        # SUMO 日志仍放在地图目录；管理端自身的输出另存 service.log。
        command = [binary, '-c', str(config_file), '--quit-on-end', 'false',
                   '--log', str(config_file.parent / 'sumo.log')]
        if not headless:
            command += ['--start']
        connection = None
        try:
            traci.start(command, label='managed', numRetries=5)
            connection = traci.getConnection('managed')
            service = SumoService(connection, network)
            for signum in (signal.SIGINT, signal.SIGTERM):
                signal.signal(signum, lambda *_: setattr(service, 'stopped', True))
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
                server.bind(str(SOCKET_FILE))
                os.chmod(SOCKET_FILE, 0o600)
                server.listen(8)
                server.settimeout(.2)
                print(f'TraCI 管理端已就绪：{config_file}', flush=True)
                next_poll = 0
                while not service.stopped:
                    if time.monotonic() >= next_poll:
                        service.poll()
                        next_poll = time.monotonic() + 1
                    try:
                        client, _ = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(3)
                        try:
                            with client.makefile('rb') as stream:
                                command = json.loads(stream.readline(262144))
                            reply = dict(ok=True, result=service.handle(command))
                        except (ValueError, KeyError, OSError, traci.TraCIException) as error:
                            reply = dict(ok=False, error=str(error))
                        try:
                            client.sendall((json.dumps(reply) + '\n').encode())
                        except OSError:
                            pass  # 请求方断开不影响其他模块。
        finally:
            if connection is not None:
                try:
                    connection.close(wait=False)
                except Exception as error:
                    print(f'TraCI 关闭信息：{error}', flush=True)
            SOCKET_FILE.unlink(missing_ok=True)


def main():
    """管理命令留在服务模块，不放进 sumo_init.py。"""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path, help='要加载的 .sumocfg')
    operations = parser.add_mutually_exclusive_group()
    operations.add_argument('--status', action='store_true', help='查询管理端状态')
    operations.add_argument('--stop', action='store_true', help='关闭管理端及其 SUMO')
    parser.add_argument('--headless', action='store_true', help='无窗口测试模式')
    args = parser.parse_args()
    if args.status:
        print(json.dumps(request({'action': 'status'}), ensure_ascii=False, indent=2))
    elif args.stop:
        request({'action': 'stop'})
        deadline = time.monotonic() + 10
        while SOCKET_FILE.exists() and time.monotonic() < deadline:
            time.sleep(.1)
        if SOCKET_FILE.exists():
            raise RuntimeError('SUMO 管理端尚未退出，请查看 service.log。')
        print('SUMO 管理端已关闭。')
    else:
        if args.config is None:
            parser.error('启动服务时需要填写 .sumocfg 路径。')
        serve(args.config.resolve(), args.headless)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ImportError, ValueError) as error:
        raise SystemExit(f'错误：{error}')
