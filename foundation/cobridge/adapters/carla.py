"""Attach to a running CARLA world and restore autonomous settings on detach."""
import hashlib

TIME_TOLERANCE = 1e-5


class CarlaAdapter:
    def __init__(self, host, port, timeout, step_length, fingerprint=None):
        import carla
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = self.client.get_world()
        current_map = self.world.get_map()
        self.fingerprint = hashlib.sha256(current_map.to_opendrive().strip().encode()).hexdigest()
        if fingerprint is not None and self.fingerprint != fingerprint:
            raise RuntimeError('CARLA 与已接入 SUMO 的地图不一致。')
        self.original = self.world.get_settings()
        if self.original.synchronous_mode:
            raise RuntimeError('CARLA 已在同步模式，请先释放其他时钟或恢复遗留设置。')
        self.step_length = step_length
        self.timeout = timeout
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = step_length
        if settings.substepping and step_length > settings.max_substep_delta_time * settings.max_substeps:
            raise ValueError('步长超过 CARLA 物理子步容量。')
        self.configured = True
        try:
            self.world.apply_settings(settings)
            snapshot = self.world.get_snapshot()
            self.frame = snapshot.frame
            self.elapsed = snapshot.timestamp.elapsed_seconds
        except BaseException:
            self.close()
            raise

    def step(self):
        if self.client.get_world().id != self.world.id:
            raise RuntimeError('CARLA 世界已切换。')
        frame = self.world.tick(self.timeout)
        snapshot = self.world.get_snapshot()
        if frame != self.frame + 1 or snapshot.frame != frame:
            raise RuntimeError('检测到其他客户端推进 CARLA。')
        if abs(snapshot.timestamp.elapsed_seconds - self.elapsed - self.step_length) > TIME_TOLERANCE:
            raise RuntimeError('CARLA 时间增量不一致。')
        self.frame, self.elapsed = frame, snapshot.timestamp.elapsed_seconds

    def close(self):
        if self.configured:
            if self.client.get_world().id == self.world.id:
                self.world.apply_settings(self.original)
            self.configured = False
