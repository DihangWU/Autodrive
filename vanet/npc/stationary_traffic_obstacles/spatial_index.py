"""固定障碍物空间网格；动态障碍仍逐帧预测，不缩小原有碰撞边界。"""
import bisect
import math


class StaticObstacleIndex:
    def __init__(self, obstacles=(), cell_size=20.):
        self.cell_size = cell_size
        self.obstacles = tuple(obstacles)
        self.maximum_half_length = max((v['length'] for v in self.obstacles), default=0.)
        self.cells = {}
        self.bounds = []
        for index, obstacle in enumerate(self.obstacles):
            x, y, _ = obstacle['center']
            # A circle containing the oriented box is conservative for every yaw.
            radius = math.hypot(obstacle['length'], obstacle['width'])
            bounds = (x - radius, y - radius, x + radius, y + radius)
            self.bounds.append(bounds)
            for cell in self._cells(bounds):
                self.cells.setdefault(cell, []).append(index)

    def _cells(self, bounds):
        x0, y0, x1, y1 = (math.floor(v / self.cell_size) for v in bounds)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                yield x, y

    def near(self, center, padding):
        x, y = center[:2]
        query = (x - padding, y - padding, x + padding, y + padding)
        found = set()
        for cell in self._cells(query):
            for index in self.cells.get(cell, ()):
                if index in found:
                    continue
                found.add(index)
                x0, y0, x1, y1 = self.bounds[index]
                if x0 <= query[2] and x1 >= query[0] and y0 <= query[3] and y1 >= query[1]:
                    yield self.obstacles[index]


class ObstacleSet:
    """与原障碍物列表兼容，并提供按轨迹点查找静态碰撞候选的接口。"""
    def __init__(self, dynamic, static_index):
        self.dynamic = tuple(dynamic)
        self.static_index = static_index

    def __iter__(self):
        yield from self.dynamic
        yield from self.static_index.obstacles

    def __len__(self):
        return len(self.dynamic) + len(self.static_index.obstacles)

    def __getitem__(self, index):
        return (self.dynamic + self.static_index.obstacles)[index]

    def near(self, center, padding):
        yield from self.dynamic
        yield from self.static_index.near(center, padding)

    def collision_candidates(self, controller, speed, begin):
        for obstacle in self.dynamic:
            yield obstacle, range(begin, len(controller.samples))
        if not self.static_index.obstacles:
            return
        config = controller.collision_configuration
        # Static cars have zero closing-target speed. Include release hysteresis
        # and the same extra body reach used by the original guard.
        horizon = (config['minimum_gap_metres'] + speed * config['reaction_seconds']
                   + speed * speed / (2. * config['braking_deceleration_mps2'])
                   + config['release_margin_metres'] + controller.front
                   + 2. * self.static_index.maximum_half_length + 5.)
        end = bisect.bisect_right(controller.distances, controller.progress + horizon)
        padding = controller.front + controller.half_width + .8
        candidates = {}
        for index in range(begin, min(end, len(controller.samples))):
            for obstacle in self.static_index.near(controller.samples[index].center, padding):
                candidates.setdefault(obstacle['id'], []).append(index)
        # Retain the original obstacle order for deterministic nearest/tie logic.
        for obstacle in self.static_index.obstacles:
            indices = candidates.get(obstacle['id'])
            if indices:
                yield obstacle, indices
