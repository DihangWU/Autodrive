"""统一推进 CARLA 和 SUMO，每帧 0.05 秒；Ctrl+C 停止时钟，保留仿真程序。"""

import argparse
import fcntl
import hashlib
import logging
from pathlib import Path
import signal
import sys
import threading
import time

PROJECT_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_DIR))
from simuenv.sumo.traci.client import STEP_LENGTH, process_identity, request

RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'cobridge'


def run_clock(args, stopped):
    """唯一的步进循环；以后在这里加入车辆状态交换，不另建时钟。"""
    import carla

    status = request({'action': 'status'}, timeout=args.timeout)
    if abs(status.get('step_length', 0) - STEP_LENGTH) > 1e-9:
        raise RuntimeError('SUMO 管理端版本或步长不匹配，请关闭 SUMO 后重新运行 run_sumo。')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    current_map = world.get_map()
    fingerprint = hashlib.sha256(current_map.to_opendrive().strip().encode()).hexdigest()
    if fingerprint != status['xodr_sha256']:
        raise RuntimeError('CARLA 与 SUMO 地图不一致，请重新转换地图并重启 SUMO。')
    original = world.get_settings()
    if original.synchronous_mode:
        raise RuntimeError('CARLA 已在同步模式，请先确认并停止其他时钟，再恢复异步模式后启动。')

    owner = process_identity()
    sumo_time = request(dict(action='claim_clock', owner=owner), timeout=args.timeout)['time']
    configured = False
    frames = 0
    started = time.monotonic()
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = STEP_LENGTH
        configured = True  # apply_settings 超时时也尝试恢复，避免 CARLA 留在同步模式。
        world.apply_settings(settings)
        snapshot = world.get_snapshot()
        first_frame = snapshot.frame
        carla_time = snapshot.timestamp.elapsed_seconds
        print(f'CoBridge 已启动：CARLA {args.host}:{args.port}，地图 {Path(current_map.name).name}')
        print(f'固定步长 {STEP_LENGTH} 秒；Ctrl+C 停止时钟，保留 CARLA 和 SUMO。')
        logging.info('启动 world=%s CARLA起点=%s SUMO起点=%s 步长=%s',
                     world.id, carla_time, sumo_time, STEP_LENGTH)
        while not stopped.is_set() and (not args.steps or frames < args.steps):
            frame_started = time.monotonic()
            if client.get_world().id != world.id:
                raise RuntimeError('CARLA 世界已切换，请停止后为新地图重新启动联仿。')
            # 以后先应用本帧控制，再 tick；在两端步进之间加入车辆状态同步。
            frame = world.tick(args.timeout)
            snapshot = world.get_snapshot()
            if frame != first_frame + frames + 1 or snapshot.frame != frame:
                raise RuntimeError('检测到其他客户端推进 CARLA，停止统一时钟。')
            result = request(dict(action='step', owner=owner, sequence=frames + 1),
                             timeout=args.timeout)
            frames += 1
            # 逐帧比较，避免 CARLA 浮点时间在长时间运行后累计微小误差。
            if (abs(snapshot.timestamp.elapsed_seconds - carla_time - STEP_LENGTH) > 1e-6
                    or abs(result['time'] - sumo_time - STEP_LENGTH) > 1e-6):
                raise RuntimeError('两端仿真时间增量不一致，停止统一时钟。')
            carla_time = snapshot.timestamp.elapsed_seconds
            sumo_time = result['time']
            # 以 20 Hz 为目标，只等待本帧剩余预算；处理超时不再额外 sleep。
            stopped.wait(max(0, STEP_LENGTH - (time.monotonic() - frame_started)))
    finally:
        # 只恢复本次接管的世界，不能把旧地图的设置应用到用户新加载的世界。
        if configured:
            try:
                if client.get_world().id == world.id:
                    world.apply_settings(original)
            except RuntimeError as error:
                logging.exception('恢复 CARLA 设置失败')
                print(f'恢复 CARLA 设置失败：{error}', file=sys.stderr)
        try:
            request(dict(action='release_clock', owner=owner), timeout=args.timeout)
        except RuntimeError as error:
            logging.warning('释放 SUMO 时钟：%s', error)
        logging.info('停止：完成 %s 帧，仿真增量 %.3f 秒，实际耗时 %.3f 秒',
                     frames, frames * STEP_LENGTH, time.monotonic() - started)
        print(f'CoBridge 已停止：完成 {frames} 帧，仿真推进 {frames * STEP_LENGTH:.2f} 秒。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='CARLA 地址，默认 127.0.0.1')
    parser.add_argument('--port', type=int, default=2000, help='CARLA RPC 端口，默认 2000')
    parser.add_argument('--timeout', type=float, default=10, help='请求超时秒数，默认 10')
    parser.add_argument('--steps', type=int, default=0, help='运行指定帧数后退出，0 表示持续运行')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.timeout <= 0 or args.steps < 0:
        parser.error('port 必须为 1～65535，timeout 大于 0，steps 不小于 0。')
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=RUNTIME_DIR / 'clock.log', level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    with (RUNTIME_DIR / 'clock.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('CoBridge 已在运行，不能启动第二个统一时钟。') from None
        stopped = threading.Event()
        for signum in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signum, lambda *_: stopped.set())
        run_clock(args, stopped)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ImportError) as error:
        logging.exception('CoBridge 异常退出')
        raise SystemExit(f'错误：{error}')
