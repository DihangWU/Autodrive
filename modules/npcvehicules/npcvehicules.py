"""通过已运行的 CoBridge 添加 SUMO 随机 NPC，并同步到 CARLA。

本入口只管理一批车流的生命周期；业务代码位于 traffic/。
Ctrl+C 删除本批车辆，不关闭仿真器、共享桥或其他车辆。
"""
import argparse
import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

PROJECT_DIR = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'npcvehicules'
LOG_FILE = RUNTIME_DIR / 'npcvehicules.log'
DEFAULT_COUNT = 10
DEFAULT_SEED = 0
DEFAULT_FILTER = 'vehicle.*'
DEFAULT_DURATION = 0.
STATUS_INTERVAL = 1.
DEFAULT_TIMEOUT = 10.

sys.path.insert(0, str(PROJECT_DIR))
from foundation.console import Console
from foundation.cobridge.control import ControlClient, SOCKET_FILE

CONSOLE = Console('NPC')


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-n', '--number-of-vehicles', type=int, default=DEFAULT_COUNT, help='本批车辆数量，默认 10')
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED, help='车型、出生道路与路线随机种子')
    parser.add_argument('--filter', default=DEFAULT_FILTER, help='CARLA 车型通配符，默认 vehicle.*')
    parser.add_argument('--bridge-socket', type=Path, default=SOCKET_FILE, help='选择要使用的 CoBridge')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION, help='运行秒数；0 表示直到 Ctrl+C')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT, help='桥控制请求超时秒数')
    args = parser.parse_args()
    if (args.number_of_vehicles < 1 or not args.filter or not math.isfinite(args.duration)
            or args.duration < 0 or not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error('数量、车型筛选、运行时长或超时无效。')
    return args


def display_status(client, batch_id, previous):
    status = client.command('npc.status', batch_id=batch_id)
    if status['state'] == 'error':
        raise RuntimeError(status['detail'])
    summary = (status['added'], status['active'], status['carla'], status['detail'], status['teleports'])
    if summary != previous:
        message = (f"本批 {status['requested']} 辆，已提交 {status['added']}，"
                   f"SUMO 上路 {status['active']}，CARLA 实体 {status['carla']}，"
                   f"等待插入 {status['pending']}，已驶离 {status['finished']}")
        if status['teleports']:
            message += f"，SUMO 传送累计 {status['teleports']} 次"
        if status['detail']:
            message += f"；{status['detail']}"
        CONSOLE.info(message)
        client.command('module.status', name=f'NPC {batch_id}', state='running', detail=message)
    return summary


def run(args, stopped):
    client = ControlClient(args.bridge_socket, args.timeout)
    batch_id = None
    try:
        result = client.command('npc.start', count=args.number_of_vehicles, seed=args.seed, filter=args.filter)
        batch_id = result['batch_id']
        CONSOLE.info(f'车流批次 {batch_id} 已登记，分帧创建。Ctrl+C 清理本批车辆。')
        started, previous = time.monotonic(), None
        while not stopped.is_set():
            previous = display_status(client, batch_id, previous)
            if args.duration and time.monotonic() - started >= args.duration:
                break
            stopped.wait(STATUS_INTERVAL)
    finally:
        try:
            if batch_id is not None:
                client.command('npc.stop', batch_id=batch_id)
                client.command('module.status', name=f'NPC {batch_id}', state='stopped')
                CONSOLE.info('已请求清理本批 NPC；共享桥和其他模块保持运行。')
        except (OSError, RuntimeError) as error:
            CONSOLE.warning(f'退出请求未确认，连接断开后由桥清理本批车辆：{error}')
        finally:
            client.close()


def main():
    args = parse_arguments()
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=LOG_FILE, level=logging.INFO,
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
        CONSOLE.error(str(error), exc_info=True)
        raise SystemExit(1)
