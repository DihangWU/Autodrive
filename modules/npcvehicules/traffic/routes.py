"""使用本次 SUMO 路网生成合法随机路线，并在接近终点时续接。"""

ROUTE_EDGES = 32
EXTEND_BEFORE_EDGES = 8


class RandomRoutes:
    def __init__(self, network, rng):
        self.network, self.rng = network, rng
        self.allowed = {}

    def starting_edges(self, vehicle_class, minimum_length=0.):
        """缓存允许该车型通行、且至少存在一个出口的普通道路。"""
        if vehicle_class not in self.allowed:
            self.allowed[vehicle_class] = [edge for edge in self.network.getEdges()
                if edge.allows(vehicle_class) and edge.getAllowedOutgoing(vehicle_class)]
        suitable = [edge for edge in self.allowed[vehicle_class] if edge.getLength() >= minimum_length]
        if not suitable:
            raise ValueError(f'当前地图没有可用于 {vehicle_class} 随机车流的道路。')
        return suitable

    def extend(self, prefix, vehicle_class):
        """只沿允许的道路连接续接；死路处结束，交给 SUMO 正常移除。"""
        edges = list(prefix)
        for _ in range(ROUTE_EDGES):
            current = self.network.getEdge(edges[-1])
            outgoing = list(current.getAllowedOutgoing(vehicle_class))
            if not outgoing:
                break
            edges.append(self.rng.choice(outgoing).getID())
        return edges

    def create(self, vehicle_class, minimum_length):
        first = self.rng.choice(self.starting_edges(vehicle_class, minimum_length))
        return self.extend([first.getID()], vehicle_class)

    def continue_vehicle(self, connection, name, record, route_index, road):
        """在普通道路上续接，保留尚未走完的原路线及路口出口。"""
        route = record['route']
        if (route_index < 0 or route_index < len(route) - EXTEND_BEFORE_EDGES
                or road.startswith(':') or route_index >= len(route)):
            return
        remaining = route[route_index:]
        extended = self.extend(remaining, record['spec']['vClass'])
        if len(extended) > len(remaining):
            connection.vehicle.setRoute(name, extended)
            record['route'] = extended
