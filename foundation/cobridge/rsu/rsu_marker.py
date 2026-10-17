"""Short-lived CARLA RSU drawing, owned by CoBridge."""
import math


# 可调几何与显示参数
DEFAULT_LIFETIME = 0.1
DEFAULT_BEACON_RADIUS = 1.6
CROWN_RADIUS_FACTOR = 0.6
MIN_MAST_HEIGHT = 2.0
MAST_RADIUS_FACTOR = 1.6
ORANGE = (255, 100, 0)
CYAN = (0, 255, 255)
YELLOW = (255, 255, 0)
PERSISTENT_LINES = True
LINE_THICKNESS = 0.20
FOOTPRINT_HALF_WIDTH = 0.6
BRACE_LEVEL_HEIGHT = 10.0
BOX_Z_OFFSET = 0.1
BOX_HALF_WIDTH = 0.7
BOX_HALF_HEIGHT = 0.08
BOX_THICKNESS = 0.16
BRACE_THICKNESS = 0.12
CROWN_HEIGHT_FACTOR = 0.6
BEACON_THICKNESS = 0.35
GROUND_RING_RADIUS = 1.0
GROUND_RING_Z_OFFSET = 0.15
TOP_RING_RADIUS_FACTOR = 0.7
RING_SEGMENTS = 32
RING_THICKNESS = 0.25


def draw_marker(world, rsu, base_z, lifetime=DEFAULT_LIFETIME):
    import carla
    x, y, top = rsu['x'], rsu['y'], rsu['z']
    base_z = rsu.get('base_z', base_z)
    radius = rsu.get('beacon_radius', DEFAULT_BEACON_RADIUS)
    crown = radius * CROWN_RADIUS_FACTOR
    mast = max(MIN_MAST_HEIGHT, radius * MAST_RADIUS_FACTOR)
    orange = carla.Color(*ORANGE)
    cyan = carla.Color(*CYAN)
    yellow = carla.Color(*YELLOW)
    options = dict(life_time=lifetime, persistent_lines=PERSISTENT_LINES)

    def loc(dx=0, dy=0, z=top):
        return carla.Location(x=x + dx, y=y + dy, z=z)

    def line(a, b, color=orange, thickness=LINE_THICKNESS):
        world.debug.draw_line(a, b, thickness=thickness, color=color, **options)

    # Keep the footprint inside the grass island; height follows the RSU z.
    for dx, dy in ((-FOOTPRINT_HALF_WIDTH, -FOOTPRINT_HALF_WIDTH), (-FOOTPRINT_HALF_WIDTH, FOOTPRINT_HALF_WIDTH), (FOOTPRINT_HALF_WIDTH, FOOTPRINT_HALF_WIDTH), (FOOTPRINT_HALF_WIDTH, -FOOTPRINT_HALF_WIDTH)):
        line(loc(dx, dy, base_z), loc(dx, dy, top))
    levels = max(1, math.ceil((top - base_z) / BRACE_LEVEL_HEIGHT))
    for i in range(levels):
        low = base_z + (top - base_z) * i / levels
        high = base_z + (top - base_z) * (i + 1) / levels
        world.debug.draw_box(carla.BoundingBox(loc(z=low + BOX_Z_OFFSET), carla.Vector3D(BOX_HALF_WIDTH, BOX_HALF_WIDTH, BOX_HALF_HEIGHT)),
                             carla.Rotation(), thickness=BOX_THICKNESS, color=orange, **options)
        for side in (-FOOTPRINT_HALF_WIDTH, FOOTPRINT_HALF_WIDTH):
            line(loc(-FOOTPRINT_HALF_WIDTH, side, low), loc(FOOTPRINT_HALF_WIDTH, side, high), thickness=BRACE_THICKNESS)
            line(loc(FOOTPRINT_HALF_WIDTH, side, low), loc(-FOOTPRINT_HALF_WIDTH, side, high), thickness=BRACE_THICKNESS)
    world.debug.draw_box(carla.BoundingBox(loc(), carla.Vector3D(crown, crown, crown * CROWN_HEIGHT_FACTOR)),
                         carla.Rotation(), thickness=BEACON_THICKNESS, color=cyan, **options)
    line(loc(z=top), loc(z=top + mast), yellow, BEACON_THICKNESS)
    # Large aerial beacon, with a small ground ring that fits before the wall.
    for r, z, color in ((GROUND_RING_RADIUS, base_z + GROUND_RING_Z_OFFSET, orange), (radius, top + crown, cyan),
                        (radius * TOP_RING_RADIUS_FACTOR, top + mast, yellow)):
        for i in range(RING_SEGMENTS):
            a, b = i * math.tau / RING_SEGMENTS, (i + 1) * math.tau / RING_SEGMENTS
            line(loc(r * math.cos(a), r * math.sin(a), z),
                 loc(r * math.cos(b), r * math.sin(b), z), color, RING_THICKNESS)

