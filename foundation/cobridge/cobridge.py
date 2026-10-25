"""Independent clock: discover simulators, attach dynamically, release on exit."""
import argparse
import fcntl
import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

# 路径与可调参数
PROJECT_DIR = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'cobridge'
LOG_FILE = RUNTIME_DIR / 'bridge.log'
LOCK_FILE = RUNTIME_DIR / 'clock.lock'
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2000
DEFAULT_STEP_LENGTH = 0.05
DEFAULT_TIMEOUT = 1.0
DEFAULT_RPC_TIMEOUT = 10.0
DEFAULT_STEPS = 0
DEFAULT_FAST = False
DISCOVERY_INTERVAL = 1.0
COMPONENT_NAME = 'CoBridge'

sys.path.insert(0, str(PROJECT_DIR))
from foundation.console import Console
from foundation.module_status import ModuleRegistry
from foundation.cobridge.adapters.carla import CarlaAdapter
from foundation.cobridge.adapters.sumo import SumoAdapter
from foundation.cobridge.control import ControlServer, SOCKET_FILE
from foundation.cobridge.lincoln.vehicle_registry import LincolnVehicleRegistry
from foundation.cobridge.npcvehicules.npcvehicules import NpcVehicleRegistry
from foundation.cobridge.traffic_lights.sumo_traffic_lights import SumoTrafficLights
from foundation.sumo.client import SOCKET_FILE as SUMO_SOCKET_FILE


CONSOLE = Console(COMPONENT_NAME)


class BridgeRuntime:
    def __init__(self, args):
        self.args = args
        self.carla = None
        self.sumo = None
        self.next_discovery = 0
        self.last_errors = {}
        self.frames = 0
        self.modules = ModuleRegistry(CONSOLE.info)
        self.lincoln = LincolnVehicleRegistry()
        self.npc = NpcVehicleRegistry()
        self.traffic_lights = SumoTrafficLights(CONSOLE.info)

    def report(self, name, error):
        message = str(error)
        if self.last_errors.get(name) != message:
            CONSOLE.info(f'{name} 等待接入：{message}')
            self.last_errors[name] = message

    def discover(self):
        now = time.monotonic()
        if now < self.next_discovery:
            return
        self.next_discovery = now + DISCOVERY_INTERVAL
        if self.carla is None:
            try:
                fingerprint = self.sumo.network['xodr_sha256'] if self.sumo else None
                self.carla = CarlaAdapter(self.args.host, self.args.port, self.args.timeout,
                                          self.args.step_length, fingerprint)
                self.last_errors.pop('CARLA', None)
                CONSOLE.info('CARLA 已接入时钟。')
            except Exception as error:
                self.report('CARLA', error)
        if self.sumo is None:
            try:
                fingerprint = self.carla.fingerprint if self.carla else None
                self.sumo = SumoAdapter(self.args.sumo_socket, self.args.rpc_timeout,
                                        self.args.step_length, fingerprint,
                                        on_operation=lambda message: CONSOLE.info(f'SUMO：{message}'))
                self.last_errors.pop('SUMO', None)
                CONSOLE.info('SUMO 已接入时钟。')
            except Exception as error:
                self.report('SUMO', error)

    def drop(self, name, error):
        CONSOLE.warning(f'{name} 已断开：{error}；时钟继续运行。')
        field = name.lower()
        adapter = getattr(self, field)
        setattr(self, field, None)
        try:
            try:
                self.traffic_lights.close()
            finally:
                adapter.close()
        except Exception:
            logging.exception('释放 %s 接入失败', name)
        self.next_discovery = 0

    def step(self):
        self.npc.synchronize(self.carla, self.sumo)
        self.traffic_lights.synchronize(self.carla, self.sumo)
        self.lincoln.prepare_traffic_managers(self.carla)
        for name in ('CARLA', 'SUMO'):
            if name == 'SUMO':
                try:
                    self.lincoln.synchronize_before_sumo_step(self.carla, self.sumo)
                except Exception:
                    logging.exception('Lincoln 投影同步失败；保留车辆并在下帧重试')
            adapter = getattr(self, name.lower())
            if adapter is not None:
                try:
                    adapter.step()
                except Exception as error:
                    self.drop(name, error)
        self.frames += 1

    def handle(self, command, owner):
        if command['action'] == 'status':
            return dict(carla_connected=self.carla is not None, sumo_connected=self.sumo is not None,
                        frame=self.frames, time=self.frames * self.args.step_length,
                        modules=self.modules.snapshot(), lincolns=self.lincoln.snapshot(),
                        npc_batches=self.npc.snapshot(),
                        traffic_lights=dict(self.traffic_lights.details))
        if command['action'] in ('lincoln.gta.set', 'lincoln.gta.status'):
            return self.lincoln.navigation.handle_gta_mode(command, self.carla, self.sumo)
        if command['action'] in ('lincoln.navigate', 'lincoln.navigation.status'):
            return self.lincoln.navigation.handle(command, self.carla, self.sumo)
        if command['action'].startswith('lincoln.'):
            return self.lincoln.handle_request(command, owner)
        if command['action'].startswith('npc.'):
            return self.npc.handle_request(command, owner, self.carla, self.sumo)
        if command['action'] == 'module.status':
            return self.modules.update(command, owner)
        raise ValueError(f"未知时钟控制请求：{command['action']}")

    def disconnected(self, owner):
        self.modules.disconnected(owner)
        self.lincoln.disconnected(owner)
        self.npc.disconnected(owner)

    def close(self):
        errors = []
        try:
            self.npc.close(self.carla, self.sumo)
        except Exception as error:
            errors.append(f'清理 NPC：{error}')
        try:
            self.traffic_lights.close()
        except Exception as error:
            errors.append(f'恢复 CARLA 信号灯：{error}')
        try:
            self.lincoln.close(self.sumo)
        except Exception as error:
            errors.append(f'清理 Lincoln 投影：{error}')
        for name in ('carla', 'sumo'):
            adapter = getattr(self, name)
            if adapter is not None:
                try:
                    adapter.close()
                except Exception as error:
                    errors.append(f'释放 {name}：{error}')
                setattr(self, name, None)
        if errors:
            raise RuntimeError('; '.join(errors))


def run_clock(args, stopped):
    runtime = BridgeRuntime(args)
    control, failure = None, None
    try:
        control = ControlServer(args.control_socket)
        CONSOLE.info(f'时钟已启动，步长 {args.step_length}s；等待仿真器接入。Ctrl+C 退出本进程。')
        while not stopped.is_set() and (not args.steps or runtime.frames < args.steps):
            started = time.monotonic()
            control.poll(runtime.handle, runtime.disconnected)
            runtime.discover()
            runtime.step()
            if not args.fast:
                stopped.wait(max(0, args.step_length - (time.monotonic() - started)))
    except BaseException as error:
        failure = error
        raise
    finally:
        errors = []
        if control is not None:
            try:
                control.close()
            except Exception as error:
                errors.append(str(error))
        try:
            runtime.close()
        except Exception as error:
            errors.append(str(error))
        logging.getLogger(COMPONENT_NAME).info('时钟完成 %s 帧', runtime.frames)
        if errors:
            details = '; '.join(errors)
            if failure is not None:
                CONSOLE.warning(f'退出清理异常：{details}')
            else:
                raise RuntimeError(details)
        CONSOLE.info('时钟已退出，已释放仿真器控制权。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--sumo-socket', type=Path, default=SUMO_SOCKET_FILE)
    parser.add_argument('--step-length', type=float, default=DEFAULT_STEP_LENGTH)
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument('--rpc-timeout', type=float, default=DEFAULT_RPC_TIMEOUT)
    parser.add_argument('--steps', type=int, default=DEFAULT_STEPS)
    parser.add_argument('--fast', action='store_true', default=DEFAULT_FAST)
    parser.add_argument('--control-socket', type=Path, default=SOCKET_FILE)
    args = parser.parse_args()
    if (not 1 <= args.port <= 65535 or args.steps < 0
            or not all(math.isfinite(v) and v > 0 for v in
                       (args.timeout, args.rpc_timeout, args.step_length))):
        parser.error('端口、步数、超时或步长无效。')
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
            raise RuntimeError('已有 CoBridge 正在运行。')
        run_clock(args, stopped)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        CONSOLE.error(f'运行失败：{error}', exc_info=True)
        raise SystemExit(1)
