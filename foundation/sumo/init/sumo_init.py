"""Foreground SUMO runtime: owns TraCI, autonomous clock and bridge attachment."""
import argparse
import fcntl
import logging
import math
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time

# 路径与可调参数
PROJECT_DIR = Path(__file__).resolve().parents[3]
MAPS_DIR = PROJECT_DIR / '.runtime' / 'maps'
CONFIG_PATTERN = '*/*.sumocfg'
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'sumo'
LOG_FILE = RUNTIME_DIR / 'sumo.log'
ERROR_LOG_FILE = RUNTIME_DIR / 'sumo-error.log'
LAUNCH_LOG_FILE = RUNTIME_DIR / 'launcher.log'
RUNTIME_LOG_FILE = RUNTIME_DIR / 'runtime.log'
DEFAULT_CONFIG = None
DEFAULT_PORT = 8813
DEFAULT_STEP_LENGTH = 0.05
SUMO_LATERAL_RESOLUTION = 0.25
DEFAULT_HEADLESS = False
SUMO_BINARY = 'sumo'
SUMO_START = 'true'
SUMO_QUIT_ON_END = 'false'
STARTUP_TIMEOUT = 20.0
POLL_INTERVAL = 0.005
SHUTDOWN_TIMEOUT = 3.0
PROC_DIR = Path('/proc')
PROC_TCP_FILES = (PROC_DIR / 'net' / 'tcp', PROC_DIR / 'net' / 'tcp6')
COMPONENT_NAME = 'SUMO'

sys.path.insert(0, str(PROJECT_DIR))
from foundation.console import Console
from foundation.ipc import JsonServer
from foundation.sumo.client import SOCKET_FILE
from foundation.sumo.traci import connect
from foundation.sumo.runtime import SumoRuntime
from foundation.sumo.init.gui.build_sumo_gui import ensure_sumo_gui


CONSOLE = Console(COMPONENT_NAME)


def find_configuration():
    files = list(MAPS_DIR.glob(CONFIG_PATTERN))
    if not files:
        raise RuntimeError('没有地图配置，请先运行 map_carla2sumo，或指定已有 .sumocfg。')
    return max(files, key=lambda path: path.stat().st_mtime)


def process_identity(pid):
    try:
        fields = (PROC_DIR / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] in ('Z', 'X') else fields[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def process_listening(pid, port):
    try:
        inodes = set()
        for descriptor in (PROC_DIR / str(pid) / 'fd').iterdir():
            try:
                target = str(descriptor.readlink())
            except FileNotFoundError:
                continue
            if target.startswith('socket:['):
                inodes.add(target[len('socket:['):-1])
        for table in PROC_TCP_FILES:
            for row in table.read_text().splitlines()[1:]:
                fields = row.split()
                if (fields[3] == '0A' and int(fields[1].rsplit(':', 1)[1], 16) == port
                        and fields[9] in inodes):
                    return True
    except (FileNotFoundError, ProcessLookupError):
        pass
    return False


def wait_for_control_requests(server, stopped):
    """请求到达立即唤醒；空闲时最多等待 5 ms，继续检查退出和自主步进。"""
    if stopped.is_set() or any(b'\n' in buffer for buffer in server.clients.values()):
        return  # 缓冲区已有完整请求时，不必再等 socket 收到新数据。
    readers = [server.socket, *server.clients]
    select.select(readers, [], [], POLL_INTERVAL)


def run(args, stopped):
    config = (args.config or find_configuration()).resolve()
    if not config.is_file() or config.suffix != '.sumocfg':
        raise ValueError(f'无效 SUMO 配置：{config}')
    connection, server, process = None, None, None
    lock_file = Path(str(args.control_socket) + '.lock')
    lock_file.parent.mkdir(parents=True, exist_ok=True)
    with lock_file.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('该 SUMO 控制入口已在运行。')
        try:
            binary = SUMO_BINARY if args.headless else str(ensure_sumo_gui())
            command = [binary, '-c', str(config),
                       '--remote-port', str(args.port), '--step-length', str(args.step_length),
                       '--lateral-resolution', str(SUMO_LATERAL_RESOLUTION),
                       '--start', SUMO_START, '--quit-on-end', SUMO_QUIT_ON_END,
                       '--log', str(LOG_FILE), '--error-log', str(ERROR_LOG_FILE)]
            with LAUNCH_LOG_FILE.open('w') as output:
                process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                           stdin=subprocess.DEVNULL, start_new_session=True)
            deadline = time.monotonic() + STARTUP_TIMEOUT
            while not process_listening(process.pid, args.port):
                if stopped.is_set():
                    return
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f'SUMO 启动失败或超时，请查看 {LAUNCH_LOG_FILE} 和 {ERROR_LOG_FILE}')
                stopped.wait(POLL_INTERVAL)
            connection = connect(port=args.port, retries=0)
            runtime = SumoRuntime(connection, config, args.step_length,
                                  on_clock_change=lambda external: CONSOLE.info(
                                      '时钟已由 cobridge 接管，自主时钟步进已暂停。' if external
                                      else 'cobridge 时钟已释放，恢复自主时钟步进。'),
                                  on_operation_confirmed=CONSOLE.info)
            server = JsonServer(args.control_socket)
            CONSOLE.info('已启动，等待统一时钟接入；未接入时自主推进。Ctrl+C 退出本进程。')
            while not stopped.is_set():
                if process.poll() is not None:
                    raise RuntimeError(f'SUMO 已退出，请查看 {ERROR_LOG_FILE}')
                server.poll(runtime.handle, runtime.disconnected)
                runtime.autonomous_step()
                wait_for_control_requests(server, stopped)
            CONSOLE.info('正在退出并释放本次仿真进程。')
        finally:
            if server is not None:
                server.close()
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    logging.exception('关闭本次 SUMO TraCI 连接失败')
            if process is not None and process.poll() is None:
                try:
                    process.wait(timeout=SHUTDOWN_TIMEOUT)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--step-length', type=float, default=DEFAULT_STEP_LENGTH)
    parser.add_argument('--headless', action='store_true', default=DEFAULT_HEADLESS)
    parser.add_argument('--control-socket', type=Path, default=SOCKET_FILE)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.step_length) or args.step_length <= 0:
        parser.error('端口无效，或步长不是有限正数。')
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=RUNTIME_LOG_FILE, level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.set())
    run(args, stopped)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        CONSOLE.error(f'运行失败：{error}', exc_info=True)
        raise SystemExit(1)
