"""固定场景控制：只操作本批 NPC 与保存过的信号状态，不推进时钟。"""
import copy
import json
import logging
from vanet.project_paths import SCENARIO_TOWN04_CONFIGURATION_FILE
from modules.npcvehicules.traffic.vehicle_types import install_vehicle_type

LOG = logging.getLogger(__name__)
OWNER_PARAMETER = 'autodrive.npc.batch'


def load_scenario(path=None):
    selected = path or SCENARIO_TOWN04_CONFIGURATION_FILE
    with open(selected,encoding='utf-8') as stream:
        return json.load(stream)


class FixedNpcTraffic:
    """兼容既有 NpcSumoAdapter 的固定批次，不调用随机出生或随机续路。"""
    def __init__(self, connection, config, batch_id='vanet_scenario'):
        self.connection, self.config, self.batch_id = connection, config, batch_id
        self.specifications = sorted(copy.deepcopy(config['npcs']),key=lambda v:(v.get('depart_offset',0.),v['id']))
        self.count = len(self.specifications)
        self.initial_count = sum(float(v.get('depart_offset',0.)) <= 0 for v in self.specifications)
        self.next_index = 0
        self.created = {}
        self.start_time = None
        self.sim_time = 0.
        self.existing_types = set(connection.vehicletype.getIDList())
        self.type_spec = copy.deepcopy(config['vehicle_type_spec'])
        self.type_spec.setdefault('colors',[])
        self.route_generation = 0

    def spawn_next_vehicles(self, limit):
        installed_routes = set(self.connection.route.getIDList())
        for _ in range(max(0,int(limit))):
            if self.next_index >= self.count:
                break
            entry = self.specifications[self.next_index]
            offset = float(entry.get('depart_offset',0.))
            if offset > 0 and (self.start_time is None or self.sim_time < self.start_time+offset):
                break
            name = entry['id']
            spec = self.type_spec
            install_vehicle_type(self.connection,spec,self.existing_types)
            route_id = f'{self.batch_id}_{self.route_generation}_{name}'
            if route_id not in installed_routes:
                self.connection.route.add(route_id,entry['route'])
            self.connection.vehicle.add(name,route_id,typeID=spec['id'],depart='now',
                                        departLane=str(entry['lane_index']),departPos=str(entry['position']),
                                        departSpeed=str(entry.get('speed',0.)))
            color = list(entry.get('color',[80,120,190,255]))
            self.created[name] = dict(spec=copy.deepcopy(spec),color=color,scenario=copy.deepcopy(entry))
            self.next_index += 1
            vehicle = self.connection.vehicle
            vehicle.setParameter(name,OWNER_PARAMETER,self.batch_id)
            vehicle.setParameter(name,'vanet.scenario.role',entry.get('role','traffic'))
            vehicle.setColor(name,color)
            vehicle.setSpeedFactor(name,float(self.config.get('npc_speed_factor',1.)))
            vehicle.setImperfection(name,0.)
            vehicle.setSpeed(name,0. if self.start_time is None else -1.)
            LOG.info('Fixed NPC added id=%s sim_time=%.3f route=%s offset=%s',name,self.sim_time,entry['route'],offset)

    def maintain_routes(self, states, constants):
        """固定路线驶完即正常离场，不消耗共享随机数重新选择道路。"""

    def remove_vehicles(self, subscribed=None):
        vehicle = self.connection.vehicle
        active = set(vehicle.getIDList())
        pending = set(self.connection.simulation.getPendingVehicles())
        for name in self.created.keys() & (active | pending):
            owner = vehicle.getParameter(name,OWNER_PARAMETER)
            if owner != self.batch_id:
                raise RuntimeError(f'固定场景 NPC 归属改变，拒绝删除 {name}')
            if subscribed is not None and name in active and name in subscribed:
                vehicle.unsubscribe(name)
                subscribed.discard(name)
            vehicle.remove(name)

    def reset(self):
        self.remove_vehicles()
        self.created.clear()
        self.next_index = 0
        self.start_time = None
        self.route_generation += 1

    def start(self, sim_time):
        if self.start_time is not None:
            raise RuntimeError('固定场景已经开始；新一轮须先恢复初始场景。')
        self.start_time = self.sim_time = float(sim_time)
        active = set(self.connection.vehicle.getIDList())
        for name in self.created.keys() & active:
            self.connection.vehicle.setSpeed(name,-1.)


class ScenarioController:
    def __init__(self, connection, config=None, event_sink=None):
        self.connection = connection
        self.config = copy.deepcopy(config if config is not None else load_scenario())
        self.event_sink = event_sink
        self.traffic = FixedNpcTraffic(connection,self.config)
        self.light_snapshot = None
        self.prepared = False
        self.started = False

    def event(self, name, **fields):
        LOG.info('Scenario event=%s fields=%s',name,fields)
        if self.event_sink is not None:
            self.event_sink(name,**fields)

    def capture_lights(self):
        if self.light_snapshot is not None:
            return copy.deepcopy(self.light_snapshot)
        now = self.connection.simulation.getTime()
        lights = self.connection.trafficlight
        self.light_snapshot = {name:dict(program=lights.getProgram(name),phase=lights.getPhase(name),
                                        remaining=max(.001,lights.getNextSwitch(name)-now),
                                        state=lights.getRedYellowGreenState(name)) for name in lights.getIDList()}
        # 正式固定场景不继承启动阶段已经消耗的相位时间，保证跨进程复现。
        if 'traffic_light_initial_phase' in self.config:
            for saved in self.light_snapshot.values():
                if saved['program']=='online':
                    raise RuntimeError('固定实验需要 SUMO 信号灯程序，不能使用外部 online 灯态。')
                saved['phase']=int(self.config['traffic_light_initial_phase'])
                saved['remaining']=float(self.config['traffic_light_initial_phase_seconds'])
        return copy.deepcopy(self.light_snapshot)

    def restore_lights(self):
        lights = self.connection.trafficlight
        for name,saved in self.light_snapshot.items():
            if saved['program'] == 'online':
                lights.setRedYellowGreenState(name,saved['state'])
            else:
                lights.setProgram(name,saved['program'])
                lights.setPhase(name,saved['phase'])
                lights.setPhaseDuration(name,saved['remaining'])
                saved['state']=lights.getRedYellowGreenState(name)

    def reset(self, emergency_active=False):
        if emergency_active:
            raise RuntimeError('请先由用户在 SUMO 中取消紧急状态，再恢复初始场景。')
        self.capture_lights()
        self.traffic.reset()
        self.restore_lights()
        self.prepared,self.started = True,False
        self.event('scenario_reset',seed=self.config['seed'],npc_count=self.traffic.count)
        return copy.deepcopy(self.config['npcs'])

    def start(self, sim_time):
        if not self.prepared or self.started:
            raise RuntimeError('场景尚未准备或已开始。')
        # 准备阶段可能已经推进多个仿真步；正式开始时重新对齐灯的初始相位。
        self.restore_lights()
        self.traffic.start(sim_time)
        self.started = True
        self.event('scenario_started',sim_time=sim_time)

    def step(self, sim_time):
        self.traffic.sim_time = float(sim_time)

    def snapshot(self):
        return dict(name=self.config['name'],prepared=self.prepared,started=self.started,
                    npc_requested=self.traffic.count,npc_added=self.traffic.next_index,
                    initial_npc_count=self.traffic.initial_count,initial_npcs_added=self.traffic.next_index>=self.traffic.initial_count,
                    start_time=self.traffic.start_time,seed=self.config['seed'],
                    traffic_lights=copy.deepcopy(self.light_snapshot))
