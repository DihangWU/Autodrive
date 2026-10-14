"""Runtime RSU collision geometry; no changes to the packaged CARLA map."""
import math


class RsuBody:
    # CARLA's static.prop.mesh supports uniform scaling only. Stack short,
    # touching cylinders to keep the mast narrow at any configured height.
    MESH = '/Engine/BasicShapes/Cylinder.Cylinder'
    MAX_DIAMETER = 1.2

    def __init__(self):
        self.actors = []

    def spawn(self, world, rsu, base_z):
        import carla
        height = rsu['z'] - base_z
        if not math.isfinite(height) or height <= 0 or height > 200:
            raise ValueError(rsu['id'] + ': physical mast height must be within 0..200m')
        count = math.ceil(height / self.MAX_DIAMETER)
        size = height / count  # Engine cylinder is 1m tall and 1m across.
        bp = world.get_blueprint_library().find('static.prop.mesh')
        bp.set_attribute('mesh_path', self.MESH)
        bp.set_attribute('scale', str(size))
        bp.set_attribute('mass', '0')
        for i in range(count):
            bp.set_attribute('role_name', f"vanet_{rsu['id']}_mast_{i}")
            actor = world.spawn_actor(bp, carla.Transform(carla.Location(
                x=rsu['x'], y=rsu['y'], z=base_z + (i + .5) * size)))
            # Record immediately so the caller can roll back partial creation.
            self.actors.append(actor)
            actor.set_simulate_physics(False)
            actor.set_enable_gravity(False)
            actor.set_collisions(True)
            box = actor.bounding_box
            if min(box.extent.x, box.extent.y, box.extent.z) < size * .45:
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
