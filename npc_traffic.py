import carla
import random
import time

HOST = "127.0.0.1"
PORT = 2000

NUM_VEHICLES = 200

client = carla.Client(HOST, PORT)
client.set_timeout(30)

world = client.get_world()
traffic_manager = client.get_trafficmanager(8000)

traffic_manager.set_global_distance_to_leading_vehicle(2.5)

blueprints = world.get_blueprint_library()

vehicle_blueprints = blueprints.filter("vehicle.*")

# 只选择四轮车辆
vehicle_blueprints = [
    bp for bp in vehicle_blueprints
    if bp.has_attribute("number_of_wheels")
    and int(bp.get_attribute("number_of_wheels")) == 4
]

spawn_points = world.get_map().get_spawn_points()

random.shuffle(spawn_points)

vehicles = []

try:
    for spawn_point in spawn_points[:NUM_VEHICLES]:

        blueprint = random.choice(vehicle_blueprints)

        vehicle = world.try_spawn_actor(
            blueprint,
            spawn_point
        )

        if vehicle is not None:
            vehicle.set_autopilot(True, traffic_manager.get_port())
            vehicles.append(vehicle)

    print(f"成功生成 {len(vehicles)} 辆自动驾驶汽车")
    print("车辆正在 Town10HD 中自动行驶")
    print("按 Ctrl+C 结束后需要从启动 docker 开始重新执行流程，否则车辆自驾会有冲突")

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("正在清理车辆...")

finally:
    for vehicle in vehicles:
        if vehicle.is_alive:
            vehicle.destroy()

    print("车辆已清理")
