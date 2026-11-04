"""每个交通边界更新 Lincoln 的 SUMO 通信圈，不改变车辆控制或仿真时钟。"""
import math


class LincolnCoverageCircle:
    IDENTIFIER = 'vanet_coverage_lincoln0'
    OWNER = 'vanet.lincoln_communication_coverage'
    LAYER = 90
    COLOR = (100, 170, 250, 255)

    def __init__(self, connection, radius):
        self.connection = connection
        self.radius = float(radius)
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('Invalid Lincoln communication radius.')
        self.vehicle_id = None
        self.center = None
        self.half_length = None
        self.offsets = [(self.radius * math.cos(i * 2 * math.pi / 128),
                         self.radius * math.sin(i * 2 * math.pi / 128)) for i in range(129)]

    def update(self, vehicle_id):
        if vehicle_id is None:
            self.close()
            return
        polygon = self.connection.polygon
        present = polygon.getIDList()
        if self.IDENTIFIER in present and polygon.getParameter(self.IDENTIFIER, 'owner') != self.OWNER:
            raise RuntimeError('Lincoln coverage polygon belongs to another component.')
        if vehicle_id not in self.connection.vehicle.getIDList():
            self.close()
            return
        vehicle = self.connection.vehicle
        x, y = vehicle.getPosition(vehicle_id)
        angle = math.radians(vehicle.getAngle(vehicle_id))
        if self.vehicle_id != vehicle_id or self.half_length is None or self.IDENTIFIER not in present:
            self.half_length = vehicle.getLength(vehicle_id) / 2.
        # SUMO reports the front bumper; circle is centred on the vehicle body.
        x -= self.half_length * math.sin(angle)
        y -= self.half_length * math.cos(angle)
        existing = self.vehicle_id == vehicle_id and self.IDENTIFIER in present
        if existing and self.center == (x, y):
            return
        shape = [(x + dx, y + dy) for dx, dy in self.offsets]
        if existing:
            polygon.setShape(self.IDENTIFIER, shape)
            self.center = (x, y)
            return
        if self.IDENTIFIER in present:
            polygon.remove(self.IDENTIFIER, layer=self.LAYER)
        polygon.add(self.IDENTIFIER, shape, self.COLOR, fill=False,
                    polygonType='vanet_lincoln_communication_range', layer=self.LAYER, lineWidth=1.5)
        try:
            polygon.setParameter(self.IDENTIFIER, 'owner', self.OWNER)
            polygon.setParameter(self.IDENTIFIER, 'radius_metres', str(self.radius))
        except Exception:
            polygon.remove(self.IDENTIFIER, layer=self.LAYER)
            self.vehicle_id = None
            raise
        self.vehicle_id = vehicle_id
        self.center = (x, y)

    def close(self):
        if self.vehicle_id is not None:
            polygon = self.connection.polygon
            if (self.IDENTIFIER in polygon.getIDList() and
                    polygon.getParameter(self.IDENTIFIER, 'owner') == self.OWNER):
                polygon.remove(self.IDENTIFIER, layer=self.LAYER)
        self.vehicle_id = None
        self.center = None
        self.half_length = None
