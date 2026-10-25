"""SUMO 自动接管 CARLA 红绿灯：校验完整映射、逐帧同步、断开恢复。

复用官方 SumoTLLogic 的 linkSignalID 映射和 BridgeHelper 灯色转换。
不创建仿真器、不推进时钟、不修改官方源码或 SUMO 配时程序。
"""
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

PROJECT_DIR = Path(__file__).resolve().parents[3]
OFFICIAL_SUMO_DIR = PROJECT_DIR / 'foundation' / 'carla' / 'carla-official' / 'Co-Simulation' / 'Sumo'
SUMO_TOOLS_DIR = Path(os.environ.get('SUMO_HOME', '/usr/share/sumo')) / 'tools'
RETRY_INTERVAL = 1.0
SIGNAL_STATE_VARIABLE = 0x20  # SUMO TraCI 的 TL_RED_YELLOW_GREEN_STATE。


def load_official_signal_helpers():
    """仅导入官方映射和转换工具，不实例化官方联仿服务。"""
    for folder in (OFFICIAL_SUMO_DIR, SUMO_TOOLS_DIR):
        if str(folder) not in sys.path:
            sys.path.append(str(folder))
    from sumo_integration.sumo_simulation import SumoTLLogic
    from sumo_integration.bridge_helper import BridgeHelper
    return SumoTLLogic, BridgeHelper.get_carla_traffic_light_state


def read_signal_connections(network_file, active_ids, logic_class):
    """读取本次路网的连接索引；同一连接在不同程序中的映射必须一致。"""
    connections = {}
    mapped_controllers = set()
    for logic in ET.parse(network_file).getroot().findall('tlLogic'):
        controller = logic.get('id')
        if controller not in active_ids:
            continue
        parameters = {p.get('key'): p.get('value') for p in logic.findall('param')
                      if p.get('key', '').startswith('linkSignalID:')}
        official = logic_class(controller, [p.get('state') for p in logic.findall('phase')], parameters)
        for landmark in official.get_all_landmarks():
            if not landmark:
                continue
            for signal in official.get_associated_signals(landmark):
                if signal in connections and connections[signal] != landmark:
                    raise ValueError(f'SUMO 连接 {signal} 在不同程序中的信号灯映射不一致。')
                connections[signal] = landmark
                mapped_controllers.add(controller)
    missing = sorted(set(active_ids) - mapped_controllers)
    if missing:
        raise ValueError(f'SUMO 信号灯缺少 linkSignalID 映射：{missing}')
    return connections


class SumoTrafficLights:
    def __init__(self, report=print):
        self.report = report
        self.endpoints = None
        self.carla = self.sumo = None
        self.saved = {}
        self.signals = {}
        self.last_states = {}
        self.previous_subscriptions = {}
        self.next_retry = 0.
        self.last_error = None
        self.details = dict(state='waiting', carla_lights=0, mapped_lights=0)

    def read_carla_landmarks(self, world):
        """按官方方案通过 OpenDRIVE 地标关联实体，同时检查未关联的实体。"""
        actors = {a.id: a for a in world.get_actors().filter('traffic.traffic_light')}
        landmarks = {}
        for landmark in world.get_map().get_all_landmarks_of_type('1000001'):
            if landmark.id:
                actor = world.get_traffic_light(landmark)
                if actor is not None:
                    landmarks[str(landmark.id)] = actor.id
        return actors, landmarks

    def attach(self, carla, sumo):
        """完整校验成功、读到灯色后才冻结；写入失败则恢复原状态。"""
        logic_class, self.convert_state = load_official_signal_helpers()
        if carla.fingerprint != sumo.network['xodr_sha256']:
            raise ValueError('CARLA 与 SUMO 地图不一致，不能接管信号灯。')
        active_ids = list(sumo.connection.trafficlight.getIDList())
        connections = read_signal_connections(sumo.network['network_file'], active_ids, logic_class)
        actors, landmarks = self.read_carla_landmarks(carla.world)
        self.signals = {}
        missing_landmarks = set()
        for signal, landmark in connections.items():
            actor_id = landmarks.get(landmark)
            if actor_id is None:
                missing_landmarks.add(landmark)
            else:
                self.signals.setdefault(actor_id, set()).add(signal)
        missing_actors = sorted(set(actors) - self.signals.keys())
        self.details = dict(state='waiting', carla_lights=len(actors), mapped_lights=len(self.signals),
                            sumo_controllers=len(active_ids), missing_carla_actor_ids=missing_actors,
                            missing_carla_landmarks=sorted(missing_landmarks))
        if missing_actors or missing_landmarks:
            raise ValueError(f'信号灯映射不完整：未匹配 CARLA actor={missing_actors}，'
                             f'SUMO 地标在 CARLA 中不存在={sorted(missing_landmarks)}；不冻结、不接管。')
        if not actors:
            self.details['state'] = 'no_lights'
            return
        frozen = {actor.is_frozen() for actor in actors.values()}
        if len(frozen) != 1:
            raise ValueError('CARLA 信号灯冻结状态不一致，无法通过全场接口准确恢复。')
        self.original_frozen = frozen.pop()
        self.carla, self.sumo = carla, sumo
        self.saved = {key: (actor, actor.get_state()) for key, actor in actors.items()}
        states = self.read_states()  # 不先冻结再等待 SUMO 回复。
        try:
            self.subscribe_states(active_ids)
            carla.world.freeze_all_traffic_lights(True)
            self.apply_states(states)
        except Exception:
            self.close()
            raise
        self.details['state'] = 'active'
        self.last_error = None
        self.report(f'SUMO 已接管全部 {len(actors)} 个 CARLA 红绿灯（{len(active_ids)} 个 SUMO 控制器）。')

    def subscribe_states(self, controllers):
        """订阅实际灯色；保留原有订阅变量，退出时还原。"""
        domain = self.sumo.connection.trafficlight
        previous = domain.getAllSubscriptionResults()
        for controller in controllers:
            variables = {int(key) for key in previous.get(controller, {})}
            self.previous_subscriptions[controller] = variables
            domain.subscribe(controller, sorted(variables | {SIGNAL_STATE_VARIABLE}))

    def read_states(self):
        """读取实际灯色，支持 SUMO 手动设相位、切程序和直接设灯色。"""
        controllers = {controller for signals in self.signals.values() for controller, _ in signals}
        domain = self.sumo.connection.trafficlight
        if self.previous_subscriptions:
            # 本地 JSON 协议会将整数键变为字符串；每帧只做一次管理端请求。
            latest = domain.getAllSubscriptionResults()
            states = {key: latest[key][str(SIGNAL_STATE_VARIABLE)] for key in controllers}
        else:
            states = {key: domain.getRedYellowGreenState(key) for key in sorted(controllers)}
        result = {}
        conflicts = []
        for actor_id, signals in self.signals.items():
            values = {states[controller][index] for controller, index in signals}
            # 同一 CARLA 灯不能表示不同车道的多个灯色，沿用官方保守取红灯规则。
            if len(values) > 1:
                conflicts.append(actor_id)
            result[actor_id] = self.convert_state(next(iter(values)) if len(values) == 1 else 'r')
        conflicts.sort()
        if conflicts and conflicts != self.details.get('conflicting_actor_ids'):
            self.report(f'同一 CARLA 灯对应的 SUMO 连接灯色冲突，按官方规则保守取红灯：{conflicts}')
        self.details['conflicting_actor_ids'] = conflicts
        return result

    def apply_states(self, states):
        """只写发生变化的灯，避免每帧向所有 CARLA actor 重复发 RPC。"""
        for actor_id, state in states.items():
            if self.last_states.get(actor_id) != state:
                self.saved[actor_id][0].set_state(state)
                self.last_states[actor_id] = state

    def synchronize(self, carla, sumo):
        """由桥主循环调用；缺一端时恢复，完整接入后自动重试。"""
        endpoints = (id(carla), id(sumo))
        if endpoints != self.endpoints:
            self.close()
            self.endpoints = endpoints
            self.next_retry = 0.
        if carla is None or sumo is None:
            self.details['state'] = 'waiting'
            return
        if time.monotonic() < self.next_retry:
            return
        try:
            if self.carla is None:
                self.attach(carla, sumo)
                if self.details['state'] == 'no_lights':
                    self.next_retry = time.monotonic() + RETRY_INTERVAL
            else:
                if carla.client.get_world().id != carla.world.id:
                    raise RuntimeError('CARLA 世界已切换，释放旧地图的信号灯控制。')
                self.apply_states(self.read_states())
        except Exception as error:
            self.close()
            self.details['state'] = 'error'
            self.details['error'] = str(error)
            self.next_retry = time.monotonic() + RETRY_INTERVAL
            if self.last_error != str(error):
                self.report(f'SUMO 控灯未接管：{error}')
                self.last_error = str(error)

    def close(self):
        """只恢复本模块保存的灯色和原冻结状态，不改配时、不关闭仿真器。"""
        if self.carla is None:
            return
        errors = []
        try:
            if self.carla.client.get_world().id == self.carla.world.id:
                for actor, state in self.saved.values():
                    try:
                        actor.set_state(state)
                    except Exception as error:
                        errors.append(str(error))
                try:
                    self.carla.world.freeze_all_traffic_lights(self.original_frozen)
                except Exception as error:
                    errors.append(str(error))
        finally:
            if self.previous_subscriptions:
                try:
                    domain = self.sumo.connection.trafficlight
                    # 若 SUMO 已失联，第一项失败后直接结束，不逐项等待超时。
                    for controller, variables in self.previous_subscriptions.items():
                        if variables:
                            domain.subscribe(controller, sorted(variables))
                        else:
                            domain.unsubscribe(controller)
                except Exception as error:
                    self.report(f'SUMO 灯色订阅清理未完成：{error}')
                self.previous_subscriptions.clear()
            self.carla = self.sumo = None
            self.saved.clear()
            self.signals.clear()
            self.last_states.clear()
            self.details['state'] = 'waiting'
        if errors:
            raise RuntimeError('信号灯恢复失败：' + '; '.join(errors))
