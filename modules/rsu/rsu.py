"""Independent foreground RSU process; no clock or simulator lifetime ownership."""
import argparse
import fcntl
import hashlib
import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

# 路径与可调参数
PROJECT_DIR = Path(__file__).resolve().parents[2]
CONFIG_FILE = PROJECT_DIR / 'foundation' / 'cobridge' / 'rsu' / 'config' / 'town04.json'
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'rsu'
STATE_FILE = RUNTIME_DIR / 'session.json'
LEGACY_STATE_FILE = PROJECT_DIR / '.runtime' / 'cobridge' / 'rsu-session.json'
LOCK_FILE = RUNTIME_DIR / 'session.lock'
LOG_FILE = RUNTIME_DIR / 'rsu.log'
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2000
DEFAULT_DURATION = 0.0
DEFAULT_TIMEOUT = 2.0
DEFAULT_RPC_TIMEOUT = 10.0
STATUS_INTERVAL = 1.0
DRAW_INTERVAL = 0.05
RETRY_INTERVAL = 1.0
COMPONENT_NAME = 'RSU'

sys.path.insert(0, str(PROJECT_DIR))
from foundation.console import Console
from foundation.module_status import ModuleReporter, DEFAULT_SOCKET as STATUS_SOCKET_FILE
from foundation.cobridge.rsu.rsu import RsuBridge, read_configuration
from foundation.cobridge.network import read_network
from foundation.sumo.client import SumoClient, SOCKET_FILE


CONSOLE = Console(COMPONENT_NAME)


def open_session(args):
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    sumo = SumoClient(args.sumo_socket, args.rpc_timeout)
    component = None
    try:
        status = sumo.status()
        network = read_network(status['config'])
        fingerprint = hashlib.sha256(world.get_map().to_opendrive().strip().encode()).hexdigest()
        if fingerprint != network['xodr_sha256']:
            raise RuntimeError('RSU 等待地图一致的 CARLA 与 SUMO。')
        component = RsuBridge(world, sumo.connection, network, args.config, STATE_FILE)
        component.setup()
        return client, world, sumo, component
    except BaseException:
        if component is not None:
            try:
                component.close()
            except Exception:
                logging.exception('RSU 创建失败后的清理未完成')
        sumo.close()
        raise


def run(args, stopped, reporter):
    started = time.monotonic()
    last_error = None
    while not stopped.is_set():
        if args.duration and time.monotonic() - started >= args.duration:
            break
        try:
            client, world, sumo, component = open_session(args)
        except Exception as error:
            message = str(error)
            reporter.set_state('waiting', message)
            if message != last_error:
                CONSOLE.info(f'等待依赖就绪：{message}')
                last_error = message
            stopped.wait(RETRY_INTERVAL)
            continue
        reporter.set_state('running', '实体已创建')
        CONSOLE.info(f'实体已创建，会话 {component.session}。Ctrl+C 清理本次实体并退出。')
        last_error = None
        next_check = 0
        try:
            while not stopped.is_set():
                now = time.monotonic()
                if args.duration and now - started >= args.duration:
                    return
                if now >= next_check:
                    if client.get_world().id != world.id:
                        raise RuntimeError('CARLA 世界已切换。')
                    sumo.status()
                    next_check = now + STATUS_INTERVAL
                component.after_step(now - started, DRAW_INTERVAL)
                stopped.wait(max(0, DRAW_INTERVAL - (time.monotonic() - now)))
        except Exception as error:
            reporter.set_state('recovering', str(error))
            CONSOLE.warning(f'依赖断开：{error}；等待恢复。', exc_info=True)
        finally:
            try:
                component.close()
                CONSOLE.info('本次实体已移除。')
            except Exception as error:
                reporter.set_state('error', f'清理未完成：{error}')
                CONSOLE.warning(f'清理未完成，保留恢复记录：{error}', exc_info=True)
            finally:
                sumo.close()
        stopped.wait(RETRY_INTERVAL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--config', type=Path, default=CONFIG_FILE)
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION)
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument('--rpc-timeout', type=float, default=DEFAULT_RPC_TIMEOUT)
    parser.add_argument('--sumo-socket', type=Path, default=SOCKET_FILE)
    parser.add_argument('--status-socket', type=Path, default=STATUS_SOCKET_FILE)
    args = parser.parse_args()
    if (not 1 <= args.port <= 65535 or not math.isfinite(args.duration) or args.duration < 0
            or not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.rpc_timeout))):
        parser.error('端口、duration 或超时无效。')
    args.config = args.config.resolve()
    read_configuration(args.config)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=LOG_FILE, level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.set())
    with LOCK_FILE.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有 RSU 命令运行，请在原终端退出。')
        if LEGACY_STATE_FILE.exists() and not STATE_FILE.exists():
            LEGACY_STATE_FILE.replace(STATE_FILE)
        reporter = ModuleReporter(COMPONENT_NAME, args.status_socket)
        try:
            run(args, stopped, reporter)
        finally:
            reporter.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        CONSOLE.error(f'运行失败：{error}', exc_info=True)
        raise SystemExit(1)
