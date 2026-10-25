"""CARLA 固定跟随、自由视角与自由跟随；只设置 spectator，不推进仿真。"""
import math
import time
import carla


class SpectatorMixin:
    def set_camera_mode(self, follow, acquire=True, replace=True, free_follow=False):
        with self.camera_lock:
            if acquire:
                acquire = self.camera_owner.claim(replace=replace)
            self.camera_control_enabled = acquire
            self.follow = follow
            self.free_follow = free_follow and not follow
            self.camera_vehicle_position = None
            self.camera_pose = None
            self.camera_time = None
            if self.spectator is None:
                self.spectator = self.world.get_spectator()
            if not follow:
                t = self.spectator.get_transform()
                self.free_pose = (t.location.x, t.location.y, t.location.z, t.rotation.pitch, t.rotation.yaw)
                if self.free_follow:
                    self.translate_free_camera(self.world.get_snapshot())
            else:
                self.update_camera()

    def update_camera(self, snapshot=None):
        with self.camera_lock:
            with self.camera_owner.access() as owned:
                if not owned:
                    self.camera_control_enabled = False
                    self.follow = False
                    self.free_follow = False
                    self.camera_vehicle_position = None
                    return
                if self.camera_control_enabled:
                    self._update_camera(snapshot)

    def translate_free_camera(self, snapshot):
        """只叠加车辆的世界坐标位移，保留用户设置的位置偏移和朝向。"""
        actor = snapshot.find(self.car.id)
        if actor is None:
            self.camera_vehicle_position = None
            return
        location = actor.get_transform().location
        position = (location.x, location.y, location.z)
        previous = self.camera_vehicle_position
        if previous is not None and self.free_pose is not None:
            x, y, z, pitch, yaw = self.free_pose
            self.free_pose = (x + position[0] - previous[0],
                              y + position[1] - previous[1],
                              z + position[2] - previous[2], pitch, yaw)
        self.camera_vehicle_position = position

    def _update_camera(self, snapshot=None):
        # Match the simulation cadence, rather than the unrelated GUI cadence.
        if not self.follow:
            self.camera_time = None
            if self.free_follow:
                self.translate_free_camera(snapshot or self.world.get_snapshot())
            if (self.free_pose is not None and self.camera_control_enabled
                    and self.free_pose != self.camera_pose):
                x, y, z, pitch, yaw = self.free_pose
                self.spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                            carla.Rotation(pitch=pitch, yaw=yaw)))
                self.camera_pose = self.free_pose
            return
        actor = (snapshot or self.world.get_snapshot()).find(self.car.id)
        if actor is None:
            return
        now = time.monotonic()
        transform = actor.get_transform()
        loc, rot = transform.location, transform.rotation
        yaw = math.radians(rot.yaw)
        target = (loc.x - 8 * math.cos(yaw), loc.y - 8 * math.sin(yaw), loc.z + 4, rot.yaw)
        if self.camera_pose is None:
            self.camera_pose = target
        else:
            dt = max(0., min(now - self.camera_time, .2))
            alpha = 1. - math.exp(-dt / .10)
            old = self.camera_pose
            yaw_delta = (target[3] - old[3] + 180.) % 360. - 180.
            heading = old[3] + alpha * yaw_delta
            # Keep a fixed vehicle-relative offset; position smoothing causes
            # the car to drift/jump relative to the camera during acceleration.
            heading_rad = math.radians(heading)
            self.camera_pose = (loc.x - 8 * math.cos(heading_rad),
                                loc.y - 8 * math.sin(heading_rad), loc.z + 4, heading)
        self.camera_time = now
        if self.spectator is None:
            self.spectator = self.world.get_spectator()
        x, y, z, heading = self.camera_pose
        self.spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                   carla.Rotation(pitch=-15, yaw=heading)))

