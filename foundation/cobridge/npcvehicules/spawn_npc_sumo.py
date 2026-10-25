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
"""从 CARLA 官方 spawn_npc_sumo.py 复制并适配的随机车流算法。

保留“随机车型→允许通行的道路→单道路初始路线”和末端随机续路。
删除官方 main 中的地图导出、仿真器创建、无限循环、sleep 和时钟推进。
连接、随机数发生器及本批记录由调用方注入，不使用全局 traci 连接。
"""

SPAWN_EXTRA_SPACE = 5.0


def allowed_starting_edges(sumo_net, vclass, vehicle_length):
    """官方通行权限筛选，加上本工程已验证的过短出生道路保护。"""
    return [edge for edge in sumo_net.getEdges()
            if edge.allows(vclass) and edge.getLength() >= vehicle_length + SPAWN_EXTRA_SPACE
            and edge.getAllowedOutgoing(vclass)]


def choose_vehicle_and_route(sumo_net, vtypes, rng):
    """对应官方 Spawn vehicles 段，生成一条只有出生道路的路线。"""
    type_id = rng.choice(list(vtypes))
    spec = vtypes[type_id]
    allowed_edges = allowed_starting_edges(sumo_net, spec['vClass'], float(spec['length']))
    if not allowed_edges:
        raise ValueError(f"没有适合 {type_id} 出生的道路")
    edge = rng.choice(allowed_edges)
    return spec, [edge.getID()]


def spawn_vehicle(connection, vehicle_id, route, type_id):
    """对应官方 route.add/vehicle.add；保留本工程分散插入与批次命名。"""
    connection.route.add(vehicle_id, route)
    connection.vehicle.add(vehicle_id, vehicle_id, typeID=type_id,
        depart='now', departLane='random', departPos='random_free', departSpeed='0')


def update_vehicle_routes(connection, sumo_net, records, states, constants, rng):
    """对应官方 Updates vehicle routes 段，到最后一条道路才随机续接。

    实际路线和索引使用同一订阅帧，车型使用本批记录；不逐车执行三次远程查询。
    """
    for vehicle_id, record in records.items():
        results = states.get(vehicle_id)
        if not results:
            continue
        route = results[str(constants.VAR_EDGES)]
        index = results[str(constants.VAR_ROUTE_INDEX)]
        vclass = record['spec']['vClass']
        if index == len(route) - 1:
            current_edge = sumo_net.getEdge(route[index])
            available_edges = list(current_edge.getAllowedOutgoing(vclass).keys())
            if available_edges:
                next_edge = rng.choice(available_edges)
                new_route = [current_edge.getID(), next_edge.getID()]
                connection.vehicle.setRoute(vehicle_id, new_route)
