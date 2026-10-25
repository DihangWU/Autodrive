"""官方同步流程的 SUMO/CARLA 适配层：通信、批量命令与贴地几何。

算法在 run_synchronization.py，本文件不拥有仿真器或时钟。
"""
import math

GROUND_CLEARANCE = 0.02  # 包围盒底面与道路之间保留 2 cm。
MAX_ROAD_HEIGHT_DIFFERENCE = 2.0  # 高度差过大时不吸附，防止误选立交桥层级。
MAX_ROAD_HORIZONTAL_DISTANCE = 6.0


def road_aligned_transform(carla, transform, waypoint, model_bottom):
    """保持 SUMO 的 XY 和航向，将车型底面贴到 CARLA 局部道路平面。"""
    road = waypoint.transform
    up = road.get_up_vector()
    yaw = math.radians(transform.rotation.yaw)
    # 车辆可能正在换道，保留其航向，只让车身法线对齐道路法线。
    along = up.x * math.cos(yaw) + up.y * math.sin(yaw)
    across = -up.x * math.sin(yaw) + up.y * math.cos(yaw)
    pitch = math.degrees(math.atan2(-along, up.z))
    roll = math.degrees(math.asin(max(-1., min(1., across))))
    dx = transform.location.x - road.location.x
    dy = transform.location.y - road.location.y
    height = road.location.z + (GROUND_CLEARANCE - model_bottom - up.x * dx - up.y * dy) / up.z
    return carla.Transform(carla.Location(transform.location.x, transform.location.y, height),
                           carla.Rotation(pitch, transform.rotation.yaw, roll))


class NpcSumoAdapter:
    """把已完成的订阅帧适配为官方 SumoSimulation 的车辆接口。"""
    def __init__(self, adapter, traffic, tools):
        self.adapter, self.traffic, self.tools = adapter, traffic, tools
        self.subscribed = set()
        self.previous_present = set()
        self.teleporting = set()
        self.teleport_count = 0
        self.states = {}
        self.available_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()
        tc = tools.constants
        self.variables = [tc.VAR_POSITION3D, tc.VAR_ANGLE, tc.VAR_SLOPE,
                          tc.VAR_SIGNALS, tc.VAR_ROUTE_INDEX, tc.VAR_ROAD_ID, tc.VAR_EDGES]

    def get_net_offset(self):
        return self.adapter.network['offset']

    def prepare_frame(self, present, states, departed, arrived, teleport_start, teleport_end):
        """车辆生命周期来自 SUMO 事件；传送中的车辆不当作驶离后重新生成。"""
        owned = set(self.traffic.created)
        self.available_actors = present & owned
        self.states = {name: state for name, state in states.items() if name in self.available_actors}
        started = teleport_start & owned
        self.teleport_count += len(started)
        self.teleporting.update(started)
        self.teleporting.difference_update(teleport_end | arrived)
        self.spawned_actors = ((departed | (present - self.subscribed)) & owned)
        self.destroyed_actors = ((arrived & owned)
            | (self.previous_present - present - self.teleporting))
        self.previous_present = self.available_actors.copy()

    def subscribe(self, actor_id):
        if actor_id in self.subscribed:
            return
        connection = self.adapter.connection
        if connection.vehicle.getParameter(actor_id, 'autodrive.npc.batch') != self.traffic.batch_id:
            raise RuntimeError(f'NPC 归属不匹配：{actor_id}')
        connection.vehicle.subscribe(actor_id, self.variables)
        self.subscribed.add(actor_id)

    def unsubscribe(self, actor_id):
        if actor_id in self.subscribed:
            self.adapter.connection.vehicle.unsubscribe(actor_id)
            self.subscribed.discard(actor_id)

    def get_actor(self, actor_id):
        """对应官方 SumoSimulation.get_actor；静态车型来自创建时的记录。"""
        if actor_id not in self.available_actors:
            return None
        state = self.states.get(actor_id)
        if not state:
            state = self.adapter.connection.vehicle.getSubscriptionResults(actor_id)
            if not state:
                return None
            self.states[actor_id] = state
        t, tc = self.tools, self.tools.constants
        position = state[str(tc.VAR_POSITION3D)]
        if any(not math.isfinite(v) or v == tc.INVALID_DOUBLE_VALUE for v in position):
            return None
        record = self.traffic.created[actor_id]
        spec = record['spec']
        transform = t.carla.Transform(t.carla.Location(*position),
            t.carla.Rotation(state[str(tc.VAR_SLOPE)], state[str(tc.VAR_ANGLE)], 0.))
        extent = t.carla.Vector3D(*(float(spec[key]) / 2 for key in ('length', 'width', 'height')))
        return t.actor(spec['id'], t.actor_class(spec['vClass']), transform,
                       state[str(tc.VAR_SIGNALS)], extent, record['color'])


class NpcCarlaAdapter:
    """只执行官方同步流程的实体命令；贴地与批量提交都封装在这里。"""
    def __init__(self, adapter, tools):
        self.adapter, self.tools = adapter, tools
        self.blueprint_library = adapter.world.get_blueprint_library()
        self.road_map = adapter.world.get_map()
        self.model_bottoms = {}
        self.actors = {}
        self.last_lights = {}
        self.errors = {}
        self.begin_updates()

    def begin_updates(self):
        self.commands, self.command_actors = [], []
        self.pending_lights = {}

    def spawn_actor(self, blueprint, transform):
        t = self.tools
        name = str(blueprint.get_attribute('role_name'))
        spawn = t.carla.Transform(transform.location + t.carla.Location(z=t.spawn_offset_z), transform.rotation)
        command = t.carla.command
        response = self.adapter.client.apply_batch_sync([
            command.SpawnActor(blueprint, spawn).then(command.SetSimulatePhysics(command.FutureActor, False))
        ], False)[0]
        if response.error:
            self.errors[name] = f'创建失败：{response.error}'
            return -1
        self.actors[response.actor_id] = dict(name=name, type_id=blueprint.id)
        return response.actor_id

    def find_road_waypoint(self, transform):
        """三维查询并检查层级；失败时由调用方回退到官方姿态。"""
        waypoint = self.road_map.get_waypoint(transform.location, project_to_road=True,
                                              lane_type=self.tools.carla.LaneType.Driving)
        if waypoint is None:
            raise ValueError('未找到对应 CARLA 行车道')
        road = waypoint.transform
        horizontal = math.hypot(transform.location.x - road.location.x,
                                transform.location.y - road.location.y)
        if (abs(transform.location.z - road.location.z) > MAX_ROAD_HEIGHT_DIFFERENCE
                or horizontal > MAX_ROAD_HORIZONTAL_DISTANCE or road.get_up_vector().z < .5):
            raise ValueError('贴地匹配未通过层级检查，本帧使用官方 SUMO 姿态')
        return waypoint

    def model_bottom(self, actor_id):
        type_id = self.actors[actor_id]['type_id']
        if type_id not in self.model_bottoms:
            vehicle = self.adapter.world.get_actor(actor_id)
            if vehicle is None:
                raise RuntimeError(f'无法读取 NPC 模型几何：{actor_id}')
            box = vehicle.bounding_box
            self.model_bottoms[type_id] = box.location.z - box.extent.z
        return self.model_bottoms[type_id]

    def get_actor_light_state(self, actor_id):
        return self.last_lights.get(actor_id, self.tools.carla.VehicleLightState.NONE)

    def synchronize_vehicle(self, actor_id, transform, lights=None):
        """贴地失败时仍跟随官方 XY 姿态，不再冻结后追赶新位置。"""
        name = self.actors[actor_id]['name']
        try:
            waypoint = self.find_road_waypoint(transform)
            transform = road_aligned_transform(self.tools.carla, transform, waypoint,
                                               self.model_bottom(actor_id))
            self.errors.pop(name, None)
        except ValueError as error:
            self.errors[name] = str(error)
        command = self.tools.carla.command
        self.commands.append(command.ApplyTransform(actor_id, transform))
        self.command_actors.append(actor_id)
        if lights is not None and self.last_lights.get(actor_id) != lights:
            self.commands.append(command.SetVehicleLightState(actor_id, self.tools.carla.VehicleLightState(lights)))
            self.command_actors.append(actor_id)
            self.pending_lights[actor_id] = lights

    def flush_updates(self):
        if not self.commands:
            return
        results = self.adapter.client.apply_batch_sync(self.commands, False)
        for actor_id, response in zip(self.command_actors, results):
            if response.error:
                raise RuntimeError(f'NPC {actor_id} 更新失败：{response.error}')
        self.last_lights.update(self.pending_lights)

    def destroy_actor(self, actor_id):
        self.destroy_actors([actor_id])

    def destroy_actors(self, actor_ids):
        actor_ids = sorted(actor_ids)
        if not actor_ids:
            return
        if self.adapter.client.get_world().id == self.adapter.world.id:
            command = self.tools.carla.command
            responses = self.adapter.client.apply_batch_sync(
                [command.DestroyActor(actor_id) for actor_id in actor_ids], False)
            for actor_id, response in zip(actor_ids, responses):
                if response.error and self.adapter.world.get_actor(actor_id) is not None:
                    raise RuntimeError(f'NPC {actor_id} 删除失败：{response.error}')
        for actor_id in actor_ids:
            record = self.actors.pop(actor_id, None)
            if record:
                self.errors.pop(record['name'], None)
            self.last_lights.pop(actor_id, None)

    def close(self):
        self.destroy_actors(self.actors)
