#!/usr/bin/env python
# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""从 CARLA 官方 run_synchronization.py 复制并适配的 NPC 同步流程。

保留 SUMO→CARLA 的生成、删除、姿态和车灯同步算法。
两端对象由 CoBridge 注入；本文件不创建仿真器、不修改世界设置、不推进时钟。
CARLA→SUMO 的 Lincoln 和红绿灯由各自模块管理，避免重复登记。
"""

INVALID_ACTOR_ID = -1


class SimulationSynchronization:
    def __init__(self, sumo_simulation, carla_simulation, bridge_helper,
                 sync_vehicle_color=True, sync_vehicle_lights=True):
        self.sumo = sumo_simulation
        self.carla = carla_simulation
        self.helper = bridge_helper
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights
        self.sumo2carla_ids = {}
        self.retry_spawns = set()

    def synchronize_vehicles(self):
        """每个 CoBridge 步骤调用一次；读取已完成的 SUMO 帧，不自行 tick。"""
        BridgeHelper = self.helper
        BridgeHelper.blueprint_library = self.carla.blueprint_library
        BridgeHelper.offset = self.sumo.get_net_offset()
        self.carla.begin_updates()

        # 官方 SUMO→CARLA 阶段：订阅并生成新车辆。
        # 适配：只处理本批 NPC，暂时生成失败的车辆在后续步骤重试。
        sumo_spawned_actors = self.sumo.spawned_actors | self.retry_spawns
        for sumo_actor_id in sorted(sumo_spawned_actors & self.sumo.available_actors):
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            if sumo_actor is None:
                self.retry_spawns.add(sumo_actor_id)
                continue
            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_blueprint.set_attribute('role_name', sumo_actor_id)
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform, sumo_actor.extent)
                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                    self.retry_spawns.discard(sumo_actor_id)
                else:
                    self.retry_spawns.add(sumo_actor_id)
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # 官方阶段：删除已驶离 SUMO 的 CARLA 代理。
        for sumo_actor_id in sorted(self.sumo.destroyed_actors):
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids[sumo_actor_id])
                del self.sumo2carla_ids[sumo_actor_id]
            self.retry_spawns.discard(sumo_actor_id)

        # 官方阶段：用 SUMO 姿态更新 CARLA，车灯仍用官方位转换。
        for sumo_actor_id, carla_actor_id in self.sumo2carla_ids.items():
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            if sumo_actor is None:
                continue  # SUMO 传送期间可能没有有效姿态，不生成第二个代理。
            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform, sumo_actor.extent)
            if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(
                    self.carla.get_actor_light_state(carla_actor_id), sumo_actor.signals)
            else:
                carla_lights = None
            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        # 适配：批量发送，False 表示不附带 CARLA tick。
        self.carla.flush_updates()

    def close(self):
        """只清理本批代理，不关闭仿真器或更改同步模式。"""
        self.carla.close()
        self.sumo2carla_ids.clear()
        self.retry_spawns.clear()
