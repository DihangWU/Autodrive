"""Runtime RSU collision geometry; no changes to the packaged CARLA map."""
import math


# CARLA 资源路径与塔身参数
MESH_PATH = '/Engine/BasicShapes/Cylinder.Cylinder'
BLUEPRINT_ID = 'static.prop.mesh'
MAX_DIAMETER = 1.2
MAX_HEIGHT = 200.0
MASS = '0'
SIMULATE_PHYSICS = False
ENABLE_GRAVITY = False
ENABLE_COLLISIONS = True
MIN_BOUNDS_RATIO = 0.45


class RsuBody:
    # CARLA's static.prop.mesh supports uniform scaling only. Stack short,
    # touching cylinders to keep the mast narrow at any configured height.

    def __init__(self, session=''):
        self.actors = []
        self.role_prefix = f'rsu_{session}_'

    def spawn(self, world, rsu, base_z):
        import carla
        height = rsu['z'] - base_z
        if not math.isfinite(height) or height <= 0 or height > MAX_HEIGHT:
            raise ValueError(f"{rsu['id']}: physical mast height must be within 0..{MAX_HEIGHT:g}m")
        count = math.ceil(height / MAX_DIAMETER)
        size = height / count  # Engine cylinder is 1m tall and 1m across.
        bp = world.get_blueprint_library().find(BLUEPRINT_ID)
        bp.set_attribute('mesh_path', MESH_PATH)
        bp.set_attribute('scale', str(size))
        bp.set_attribute('mass', MASS)
        for i in range(count):
            bp.set_attribute('role_name', f"{self.role_prefix}{rsu['id']}_mast_{i}")
            actor = world.spawn_actor(bp, carla.Transform(carla.Location(
                x=rsu['x'], y=rsu['y'], z=base_z + (i + .5) * size)))
            # Record immediately so the caller can roll back partial creation.
            self.actors.append(actor)
            actor.set_simulate_physics(SIMULATE_PHYSICS)
            actor.set_enable_gravity(ENABLE_GRAVITY)
            actor.set_collisions(ENABLE_COLLISIONS)
            box = actor.bounding_box
            if min(box.extent.x, box.extent.y, box.extent.z) < size * MIN_BOUNDS_RATIO:
                raise RuntimeError('RSU cylinder mesh is missing or has invalid bounds')

    def destroy(self):
        errors = []
        for actor in list(self.actors):
            try:
                # A newly spawned actor may not be in the cached snapshot yet
                # (notably while Qtenv is paused). Always send the destroy RPC.
                if not actor.destroy() and actor.is_alive:
                    raise RuntimeError(f'Could not destroy RSU actor {actor.id}')
                self.actors.remove(actor)
            except Exception as error:
                errors.append(error)
        if errors:
            raise RuntimeError('RSU cleanup failed: ' + str(errors[0]))
