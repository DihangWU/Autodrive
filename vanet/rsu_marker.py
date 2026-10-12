"""Short-lived CARLA RSU drawing, owned by the OMNeT++ scene session."""
import math


def draw_marker(world, rsu, base_z, lifetime=.1, label="RSU"):
    import carla
    x, y, top = rsu['x'], rsu['y'], rsu['z']
    orange = carla.Color(255, 100, 0)
    cyan = carla.Color(0, 255, 255)
    yellow = carla.Color(255, 255, 0)
    options = dict(life_time=lifetime, persistent_lines=True)

    def loc(dx=0, dy=0, z=top):
        return carla.Location(x=x + dx, y=y + dy, z=z)

    def line(a, b, color=orange, thickness=.12):
        world.debug.draw_line(a, b, thickness=thickness, color=color, **options)

    # Four bright legs and cross braces form a conspicuous 5 m tower.
    for dx, dy in ((-.5, -.5), (-.5, .5), (.5, .5), (.5, -.5)):
        line(loc(dx, dy, base_z), loc(dx, dy, top))
    for z in (base_z + .15, (base_z + top) / 2, top - .35):
        world.debug.draw_box(carla.BoundingBox(loc(z=z), carla.Vector3D(.6, .6, .08)),
                             carla.Rotation(), thickness=.10, color=orange, **options)
    line(loc(-.5, -.5, base_z), loc(.5, -.5, top))
    line(loc(.5, -.5, base_z), loc(-.5, -.5, top))
    world.debug.draw_box(carla.BoundingBox(loc(), carla.Vector3D(1.1, 1.1, .55)),
                         carla.Rotation(), thickness=.18, color=cyan, **options)
    line(loc(z=top + .55), loc(z=top + 2), yellow, .18)
    # Two horizontal rings make the location easy to pick out from above.
    for radius, z, color in ((2.2, base_z + .2, orange), (1.6, top + 1.0, cyan)):
        for i in range(24):
            a, b = i * math.tau / 24, (i + 1) * math.tau / 24
            line(loc(radius * math.cos(a), radius * math.sin(a), z),
                 loc(radius * math.cos(b), radius * math.sin(b), z), color, .1)
    world.debug.draw_string(loc(z=top + 2.3), label, draw_shadow=True,
                            color=yellow, **options)

