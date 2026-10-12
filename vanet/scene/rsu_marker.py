"""Short-lived CARLA RSU drawing, owned by the independent RSU command."""
import math
from vanet.scene.rsu_label import draw_label


def draw_marker(world, rsu, base_z, lifetime=.1, label="RSU"):
    import carla
    x, y, top = rsu['x'], rsu['y'], rsu['z']
    base_z = rsu.get('base_z', base_z)
    radius = rsu.get('beacon_radius', 1.6)
    crown = radius * .6
    mast = max(2, radius * 1.6)
    orange = carla.Color(255, 100, 0)
    cyan = carla.Color(0, 255, 255)
    yellow = carla.Color(255, 255, 0)
    options = dict(life_time=lifetime, persistent_lines=True)

    def loc(dx=0, dy=0, z=top):
        return carla.Location(x=x + dx, y=y + dy, z=z)

    def line(a, b, color=orange, thickness=.20):
        world.debug.draw_line(a, b, thickness=thickness, color=color, **options)

    # Keep the footprint inside the grass island; height follows the RSU z.
    for dx, dy in ((-.6, -.6), (-.6, .6), (.6, .6), (.6, -.6)):
        line(loc(dx, dy, base_z), loc(dx, dy, top))
    levels = max(1, math.ceil((top - base_z) / 10))
    for i in range(levels):
        low = base_z + (top - base_z) * i / levels
        high = base_z + (top - base_z) * (i + 1) / levels
        world.debug.draw_box(carla.BoundingBox(loc(z=low + .1), carla.Vector3D(.7, .7, .08)),
                             carla.Rotation(), thickness=.16, color=orange, **options)
        for side in (-.6, .6):
            line(loc(-.6, side, low), loc(.6, side, high), thickness=.12)
            line(loc(.6, side, low), loc(-.6, side, high), thickness=.12)
    world.debug.draw_box(carla.BoundingBox(loc(), carla.Vector3D(crown, crown, crown * .6)),
                         carla.Rotation(), thickness=.35, color=cyan, **options)
    line(loc(z=top), loc(z=top + mast), yellow, .35)
    # Large aerial beacon, with a small ground ring that fits before the wall.
    for r, z, color in ((1.0, base_z + .15, orange), (radius, top + crown, cyan),
                        (radius * .7, top + mast, yellow)):
        for i in range(32):
            a, b = i * math.tau / 32, (i + 1) * math.tau / 32
            line(loc(r * math.cos(a), r * math.sin(a), z),
                 loc(r * math.cos(b), r * math.sin(b), z), color, .25)
    draw_label(world, x, y, top + mast + 5, label, lifetime=lifetime)

