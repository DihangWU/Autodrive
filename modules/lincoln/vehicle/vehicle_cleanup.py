"""只清理本次 Lincoln 的控制器、窗口、传感器和 CARLA actor。"""
import gc
import json
import logging
import signal
import subprocess
import carla

RVIZ_EXIT_TIMEOUT = 3


class VehicleCleanupMixin:
    def stop_vehicle_controller(self):
        service = getattr(self, 'emergency_service', None)
        if service is not None:
            service.close()
        if self.navigation is not None:
            self.navigation.close()
        self.xbox.close()
        if self.driver is not None:
            self.driver.close()
        self.autopilot = False

    def close_vehicle_rviz(self):
        if self.rviz is None or self.rviz.poll() is not None:
            return
        self.rviz.send_signal(signal.SIGINT)
        try:
            self.rviz.wait(timeout=RVIZ_EXIT_TIMEOUT)
        except subprocess.TimeoutExpired:
            self.rviz.kill()
            self.rviz.wait()

    def close_navigation_rviz(self):
        if self.navigation_view is not None:
            self.navigation_view.close()

    def stop_sensor_streams(self):
        for entry in self.sensors:
            try:
                entry['actor'].stop()
            except RuntimeError:
                logging.exception('停止传感器流失败：%s', entry['spec']['name'])
        if self.sensors:
            logging.info('SENSOR_COUNTS %s', json.dumps({
                entry['spec']['name']: [entry['received'], entry['published']]
                for entry in self.sensors}))

    def destroy_owned_actors(self):
        if not self.actors:
            return
        if self.client.get_world().id != self.world.id:
            return  # 世界已重载，不能把旧 actor ID 用于新世界。
        responses = self.client.apply_batch_sync(
            [carla.command.DestroyActor(actor.id) for actor in reversed(self.actors)], False)
        errors = [reply.error for reply in responses if reply.error]
        if errors:
            raise RuntimeError('CARLA 清理结果：' + '; '.join(errors))

    def release_native_references(self):
        self.actors.clear()
        # 回调闭包引用 entry；entry 再持有 actor 会形成跨语言引用环。
        for entry in self.sensors:
            entry['actor'] = None
        self.sensors.clear()
        self.sensor_display_frames.clear()
        if hasattr(self, 'wheel_pose_frames'):
            self.wheel_pose_frames.clear()
        self.latest_images.clear()
        self.driver = None
        self.car = self.spectator = self.world = None
        gc.collect()
        self.client = None
        gc.collect()

    def cleanup(self):
        """某一步清理失败也继续其余步骤，最后统一报告错误。"""
        errors = []
        for cleanup in (self.camera_owner.release, self.stop_vehicle_controller,
                        self.close_navigation_rviz, self.close_vehicle_rviz,
                        self.stop_sensor_streams, self.destroy_owned_actors):
            try:
                cleanup()
            except Exception as error:
                errors.append(str(error))
                logging.exception('Lincoln 退出清理：%s', cleanup.__name__)
        self.release_native_references()
        if errors:
            raise RuntimeError('Lincoln 未能完整清理：' + '; '.join(errors))
