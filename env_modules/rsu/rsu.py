"""从同一份配置在 CARLA 创建 RSU 实体，在 SUMO 创建图标；Ctrl+C 同时清理。"""

import argparse
import fcntl
import hashlib
import json
import logging
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time
import uuid


MODULE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = MODULE_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))
from carla_rsu import RsuBody
from rsu_marker import draw_marker
from rsu_label import draw_label, label_text
from sumo_rsu import SumoRsu
from simuenv.sumo.traci.client import owner_alive, process_identity

RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'rsu'


def save_state(path, state):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2))
    temporary.replace(path)


def clean_previous_session(world, sumo, state_file):
    """上次异常退出后，再次启动时按会话前缀回收遗留实体。"""
    if not state_file.exists():
        return
    state = json.loads(state_file.read_text())
    if owner_alive(state['owner']):
        raise RuntimeError('上一个 RSU 进程仍在运行。')
    if state.get('world_id') == world.id:
        world.wait_for_tick(3)
        prefix = f'rsu_{state["session"]}_'
        for actor in world.get_actors():
            if actor.attributes.get('role_name', '').startswith(prefix):
                if not actor.destroy():
                    raise RuntimeError(f'无法清理上次遗留的 RSU actor {actor.id}')
    sumo.remove(state['session'])
    state_file.unlink()
    logging.info('已回收上次遗留的 RSU 会话：%s', state['session'])


def read_configuration(path):
    config = json.loads(path.read_text())
    if config.get('coordinate_system', 'carla') != 'carla':
        raise ValueError('RSU 配置应使用 CARLA 世界坐标，单位为米。')
    rsus = config['rsus']
    if not rsus:
        raise ValueError('rsus 列表不能为空。')
    names = set()
    for rsu in rsus:
        name = rsu['id']
        if not re.fullmatch(r'[A-Za-z0-9_]+', name) or name in names:
            raise ValueError(f'RSU ID 无效或重复：{name}')
        names.add(name)
        for key in ('x', 'y', 'z', 'base_z'):
            rsu[key] = float(rsu[key])
            if not math.isfinite(rsu[key]):
                raise ValueError(f'{name}.{key} 必须为有限数值。')
        if not 0 < rsu['z'] - rsu['base_z'] <= 200:
            raise ValueError(f'{name} 塔身高度必须在 0～200 米之间。')
        radius = float(rsu.get('beacon_radius', 5))
        if not math.isfinite(radius) or not 0 < radius <= 50:
            raise ValueError(f'{name}.beacon_radius 必须在 0～50 米之间。')
        rsu['beacon_radius'] = radius
    return config


def run(args, stopped):
    import carla

    config = read_configuration(args.config)
    sumo = SumoRsu()
    client = carla.Client(args.host, args.port)
    client.set_timeout(5)
    world = client.get_world()
    current_map = world.get_map()
    map_name = Path(current_map.name).name
    if config['map'] != map_name or sumo.status['map'] != map_name:
        raise RuntimeError(f'地图不一致：CARLA={map_name}，SUMO={sumo.status["map"]}，RSU={config["map"]}')
    fingerprint = hashlib.sha256(current_map.to_opendrive().strip().encode()).hexdigest()
    if fingerprint != sumo.status['xodr_sha256']:
        raise RuntimeError('SUMO 路网所用 .xodr 与当前 CARLA 地图不一致，请重新转换并重启 SUMO。')

    session = uuid.uuid4().hex
    bodies = []
    state_file = RUNTIME_DIR / 'session.json'
    clean_previous_session(world, sumo, state_file)
    state = dict(session=session, owner=process_identity(), host=args.host, port=args.port,
                 world_id=world.id, map=map_name, rsus=config['rsus'], actors=[])
    save_state(state_file, state)
    try:
        # 同名 POI 会报错，避免覆盖其他模块创建的对象。
        sumo.add(session, config['rsus'])
        for rsu in config['rsus']:
            body = RsuBody(session)
            bodies.append(body)  # 立即记录，生成到一半失败也能清理。
            body.spawn(world, rsu, rsu['base_z'])
            state['actors'].extend(actor.id for actor in body.actors)
            save_state(state_file, state)
        logging.info('已创建 %s，实体 actor IDs=%s', [r['id'] for r in config['rsus']], state['actors'])
        print('RSU 已同步到 CARLA 和 SUMO：' + ', '.join(r['id'] for r in config['rsus']))
        print('Ctrl+C 删除本次 RSU；CARLA 和 SUMO 保持运行。')
        started = time.monotonic()
        next_check = 0
        next_marker = 0
        while not stopped.is_set():
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            if time.monotonic() >= next_check:
                sumo.check()
                if world.id != client.get_world().id:
                    raise RuntimeError('CARLA 已切换世界，结束本次 RSU 会话。')
                next_check = time.monotonic() + 1
            for rsu in config['rsus']:
                if now >= next_marker:
                    draw_marker(world, rsu, rsu['base_z'], lifetime=.8)
                label_z = rsu['z'] + max(2, rsu['beacon_radius'] * 1.6) + 5
                draw_label(world, rsu['x'], rsu['y'], label_z, label_text(rsu['id']),
                           lifetime=.06, elapsed=time.monotonic() - started)
            if now >= next_marker:
                next_marker = now + .25
            # 文字约 20 Hz 更新；短寿命减少旋转时旧角度文字的残影。
            stopped.wait(max(0, .05 - (time.monotonic() - now)))
    finally:
        errors = []
        for body in reversed(bodies):
            try:
                body.destroy()
            except Exception as error:
                errors.append(str(error))
        try:
            sumo.remove(session)
        except RuntimeError as error:
            logging.warning('SUMO POI 清理：%s', error)
            errors.append(str(error))
        if errors:
            logging.error('清理未完全确认：%s', errors)
            print(f'RSU 清理未完全确认，状态保留在 {state_file}，下次启动时重试。')
        else:
            state_file.unlink(missing_ok=True)
            print('本次 CARLA RSU 实体和 SUMO 图标已移除。')
        logging.info('会话结束：%s，清理错误=%s', session, errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='CARLA 地址')
    parser.add_argument('--port', type=int, default=2000, help='CARLA RPC 端口')
    parser.add_argument('--config', type=Path, default=MODULE_DIR / 'config' / 'town04.json',
                        help='RSU 配置，默认使用旧版 Town04 位置')
    parser.add_argument('--duration', type=float, default=0, help='运行指定秒数后清理；0 表示直到 Ctrl+C')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error('端口必须在 1～65535，duration 不能为负数。')
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=RUNTIME_DIR / 'rsu.log', level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.set())
    with (RUNTIME_DIR / 'session.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有 RSU 命令运行，请先在原终端退出。')
        run(args, stopped)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError, ImportError) as error:
        logging.exception('RSU 会话失败')
        raise SystemExit(f'错误：{error}')
