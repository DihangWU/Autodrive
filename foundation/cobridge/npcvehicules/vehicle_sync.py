"""将一批 SUMO NPC 映射为 CARLA 无动力学代理，复用官方坐标和灯光转换。"""
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


class NpcCarlaProjection:
    def __init__(self, carla_adapter, sumo_adapter, tools, sync_lights):
        self.carla, self.sumo, self.tools = carla_adapter, sumo_adapter, tools
        self.sync_lights = sync_lights
        self.blueprints = carla_adapter.world.get_blueprint_library()
        self.road_map = carla_adapter.world.get_map()
        self.model_bottoms = {}
        self.actors = {}
        self.subscribed = set()
        self.last_lights = {}
        self.errors = {}
        tc = tools.constants
        self.variables = [tc.VAR_POSITION3D, tc.VAR_ANGLE, tc.VAR_SLOPE,
                          tc.VAR_SIGNALS, tc.VAR_ROUTE_INDEX, tc.VAR_ROAD_ID]

    def subscribe_new_vehicles(self, traffic, present):
        """只订阅本批已经上路的车辆；待插入车辆没有有效世界坐标。"""
        for name in traffic.created.keys() & (present - self.subscribed):
            if self.sumo.connection.vehicle.getParameter(name, 'autodrive.npc.batch') != traffic.batch_id:
                raise RuntimeError(f'NPC 车辆归属冲突：{name}')
            self.sumo.connection.vehicle.subscribe(name, self.variables)
            self.subscribed.add(name)

    def read_actor(self, record, state):
        """TraCI RPC 的 JSON 字典键为字符串，转换后交给官方 BridgeHelper。"""
        t, tc = self.tools, self.tools.constants
        spec = record['spec']
        position = state[str(tc.VAR_POSITION3D)]
        transform = t.carla.Transform(t.carla.Location(*position),
            t.carla.Rotation(state[str(tc.VAR_SLOPE)], state[str(tc.VAR_ANGLE)], 0.))
        extent = t.carla.Vector3D(*(float(spec[key]) / 2 for key in ('length', 'width', 'height')))
        return t.actor(spec['id'], t.actor_class(spec['vClass']), transform,
                       state[str(tc.VAR_SIGNALS)], extent, record['color'])

    def spawn_actor(self, name, actor, transform):
        t = self.tools
        blueprint = t.helper.get_carla_blueprint(actor, sync_color=True)
        if blueprint is None:
            raise RuntimeError(f'CARLA 缺少 NPC 模型：{actor.type_id}')
        blueprint.set_attribute('role_name', name)
        spawn = t.carla.Transform(transform.location + t.carla.Location(z=t.spawn_offset_z), transform.rotation)
        command = t.carla.command
        response = self.carla.client.apply_batch_sync([
            command.SpawnActor(blueprint, spawn).then(command.SetSimulatePhysics(command.FutureActor, False))
        ], False)[0]
        if response.error:
            raise RuntimeError(f'{name} 创建失败：{response.error}')
        self.actors[name] = response.actor_id

    def find_road_waypoint(self, transform):
        """使用包含高度的原位置查找车道，并拒绝远处或高度不相符的道路。"""
        waypoint = self.road_map.get_waypoint(transform.location, project_to_road=True,
                                              lane_type=self.tools.carla.LaneType.Driving)
        if waypoint is None:
            raise ValueError('未找到对应的 CARLA 行车道')
        road = waypoint.transform
        horizontal = math.hypot(transform.location.x - road.location.x,
                                transform.location.y - road.location.y)
        if (abs(transform.location.z - road.location.z) > MAX_ROAD_HEIGHT_DIFFERENCE
                or horizontal > MAX_ROAD_HORIZONTAL_DISTANCE or road.get_up_vector().z < .5):
            raise ValueError('未找到高度和位置相符的 CARLA 路面，暂缓更新，避免吸附到错误层级')
        return waypoint

    def model_bottom(self, name, type_id):
        """每种车型只读取一次实际模型包围盒，不把车身高度当作原点偏移。"""
        if type_id not in self.model_bottoms:
            vehicle = self.carla.world.get_actor(self.actors[name])
            if vehicle is None:
                raise RuntimeError(f'无法读取 NPC 模型几何：{name}')
            box = vehicle.bounding_box
            self.model_bottoms[type_id] = box.location.z - box.extent.z
        return self.model_bottoms[type_id]

    def update_vehicles(self, traffic, states, present):
        """一批提交姿态和灯光；apply_batch_sync(False) 不额外 tick。"""
        t = self.tools
        t.helper.blueprint_library = self.blueprints
        t.helper.offset = self.sumo.network['offset']
        self.destroy_vehicles(set(self.actors) - present)
        commands, names = [], []
        for name in traffic.created.keys() & present:
            state = states.get(name)
            if not state:
                continue
            actor = self.read_actor(traffic.created[name], state)
            transform = t.helper.get_carla_transform(actor.transform, actor.extent)
            try:
                waypoint = self.find_road_waypoint(transform)
            except ValueError as error:
                self.errors[name] = str(error)
                continue  # 新车暂不生成；已有车辆保留上一处有效贴地位置。
            if name not in self.actors:
                try:
                    self.spawn_actor(name, actor, transform)
                    self.errors.pop(name, None)
                except RuntimeError as error:
                    self.errors[name] = str(error)
                    continue  # 插入点暂时被占用，下一帧再试，不额外生成 SUMO 车辆。
            transform = road_aligned_transform(t.carla, transform, waypoint,
                                               self.model_bottom(name, actor.type_id))
            self.errors.pop(name, None)
            commands.append(t.carla.command.ApplyTransform(self.actors[name], transform))
            names.append(name)
            if self.sync_lights:
                lights = t.helper.get_carla_lights_state(t.carla.VehicleLightState.NONE, actor.signals)
                if self.last_lights.get(name) != lights:
                    commands.append(t.carla.command.SetVehicleLightState(
                        self.actors[name], t.carla.VehicleLightState(lights)))
                    names.append(name)
                    self.last_lights[name] = lights
        if commands:
            for name, response in zip(names, self.carla.client.apply_batch_sync(commands, False)):
                if response.error:
                    raise RuntimeError(f'NPC {name} 姿态/灯光更新失败：{response.error}')

    def destroy_vehicles(self, names):
        """只删除本投影记录的实体，世界切换后不向新世界写入旧 actor ID。"""
        names = sorted(names)
        if not names:
            return
        if self.carla.client.get_world().id == self.carla.world.id:
            command = self.tools.carla.command
            responses = self.carla.client.apply_batch_sync(
                [command.DestroyActor(self.actors[name]) for name in names], False)
            for name, response in zip(names, responses):
                if response.error and self.carla.world.get_actor(self.actors[name]) is not None:
                    raise RuntimeError(f'NPC {name} 删除失败：{response.error}')
        for name in names:
            self.actors.pop(name, None)
            self.last_lights.pop(name, None)
            self.errors.pop(name, None)

    def close(self):
        self.destroy_vehicles(self.actors)
