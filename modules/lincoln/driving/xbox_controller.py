"""读取 Linux Xbox 手柄，为一辆 Lincoln 提供线性驾驶输入。

使用系统 joystick 接口，无额外依赖、线程或仿真时钟。每个手柄通过
进程锁只分配给一辆车；支持热插拔及 --xbox-device 固定配对。
读取轴值和按键，复用现有驾驶、视角入口；车辆控制统一由 drive() 输出。
"""
import array
import fcntl
import logging
import math
import os
from pathlib import Path
import struct
import time

from modules.lincoln.runtime.paths import RUNTIME_DIR

INPUT_DIR = Path('/dev/input')
DEVICE_LOCK_DIR = RUNTIME_DIR / 'xbox'
EVENT = struct.Struct('IhBB')
READ_SIZE = EVENT.size * 64
RECONNECT_INTERVAL = 1.0
STEERING_DEADZONE = 0.08
AXIS_EVENT = 0x02
BUTTON_EVENT = 0x01
INITIAL_EVENT = 0x80
GET_NAME = 0x80806A13
GET_AXES_COUNT = 0x80016A11
GET_AXIS_MAP = 0x80406A32
GET_BUTTON_MAP = 0x84006A34
ABS_X, ABS_Z, ABS_RZ, ABS_GAS, ABS_BRAKE = 0, 2, 5, 9, 10
ABS_Y, ABS_RX, ABS_RY = 1, 3, 4
BTN_A, BTN_LB, BTN_RIGHT_STICK = 304, 310, 318
CAMERA_SPEED = 10.0
CAMERA_YAW_SPEED = 180.0
CAMERA_PITCH_SPEED = 45.0
MAX_CAMERA_ELAPSED = 0.1


def linear_steering(value):
    """中心死区之外线性映射到 -1～1，防止摇杆回中漂移。"""
    axis = max(-1., min(1., value / 32767.))
    amount = max(0., (abs(axis) - STEERING_DEADZONE) / (1. - STEERING_DEADZONE))
    return -amount if axis < 0 else amount


def linear_trigger(value):
    """Linux joystick 扳机从 -32767～32767 线性映射到 0～1。"""
    return max(0., min(1., (value + 32767.) / 65534.))


def read_device_axes(fd):
    """按 Linux 轴语义识别 USB/蓝牙映射，不写死第几个轴。"""
    count = bytearray(1)
    fcntl.ioctl(fd, GET_AXES_COUNT, count)
    axes = array.array('B', [0] * 64)
    fcntl.ioctl(fd, GET_AXIS_MAP, axes)
    codes = list(axes[:count[0]])
    # 蓝牙 Xbox 使用 GAS/BRAKE，USB 常用 Z/RZ；蓝牙的 Z 是右摇杆。
    left, right = (ABS_BRAKE, ABS_GAS) if ABS_GAS in codes and ABS_BRAKE in codes else (ABS_Z, ABS_RZ)
    right_x, right_y = (ABS_Z, ABS_RZ) if left == ABS_BRAKE else (ABS_RX, ABS_RY)
    if not all(code in codes for code in (ABS_X, left, right, right_x, right_y, ABS_Y)):
        raise ValueError('设备没有 Xbox 所需的独立扳机与左摇杆轴。')
    return tuple(codes.index(code) for code in (ABS_X, left, right, right_x, right_y, ABS_Y))


def read_device_buttons(fd):
    """按 Linux 按键语义识别 A、左肩键和右摇杆按下，不写死按钮序号。"""
    codes = array.array('H', [0] * 512)
    fcntl.ioctl(fd, GET_BUTTON_MAP, codes)
    return {codes.index(code): action for code, action in (
        (BTN_A, 'autopilot'), (BTN_LB, 'reverse'), (BTN_RIGHT_STICK, 'free_follow')) if code in codes}


class XboxController:
    def __init__(self, vehicle_name, device=None):
        self.vehicle_name = vehicle_name
        self.requested_device = Path(device) if device else None
        self.fd = self.device_lock = None
        self.axes = {}
        self.axis_indices = None
        self.button_actions = {}
        self.buttons = {}
        self.pending_actions = []
        self.camera_speed = CAMERA_SPEED
        self.camera_input = (0., 0., 0., 0., 0.)
        self.camera_time = time.monotonic()
        self.next_search = 0.
        self.armed = False
        self.status = 'Xbox：等待空闲手柄'
        self.device_name = ''

    def claim_device(self, path):
        """先占用设备对应的锁，再开始读取，防止多辆车共享同一手柄。"""
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        lock = None
        try:
            name = bytearray(128)
            fcntl.ioctl(fd, GET_NAME, name)
            name = name.split(b'\0', 1)[0].decode(errors='replace')
            if not self.requested_device and not any(word in name.lower() for word in ('xbox', 'x-box', 'xinput')):
                return False
            indices = read_device_axes(fd)
            button_actions = read_device_buttons(fd)
            device_id = os.fstat(fd).st_rdev
            DEVICE_LOCK_DIR.mkdir(parents=True, exist_ok=True)
            lock = (DEVICE_LOCK_DIR / f'{os.major(device_id)}-{os.minor(device_id)}.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return False
            self.fd, self.device_lock, self.axis_indices = fd, lock, indices
            self.button_actions = button_actions
            self.device_name = f'{path.name} ({name})'
            self.axes.clear()
            self.armed = False
            logging.info('%s 占用手柄 %s', self.vehicle_name, path)
            return True
        finally:
            if self.fd != fd:
                os.close(fd)
                if lock is not None:
                    lock.close()

    def find_device(self):
        """未分配时每秒重试；指定设备被占用时不改配另一只手柄。"""
        if time.monotonic() < self.next_search:
            return
        self.next_search = time.monotonic() + RECONNECT_INTERVAL
        paths = [self.requested_device] if self.requested_device else sorted(INPUT_DIR.glob('js*'))
        for path in paths:
            try:
                if self.claim_device(path):
                    return
            except (OSError, ValueError) as error:
                logging.debug('手柄暂不可用 %s：%s', path, error)

    def read_axis_events(self):
        """排空当前输入队列，只保留最新轴值，不等待新事件。"""
        while True:
            try:
                data = os.read(self.fd, READ_SIZE)
            except BlockingIOError:
                return
            if not data:
                raise OSError('手柄已断开')
            for offset in range(0, len(data), EVENT.size):
                _, value, kind, axis = EVENT.unpack_from(data, offset)
                if kind & ~INITIAL_EVENT == AXIS_EVENT:
                    self.axes[axis] = value
                elif kind & ~INITIAL_EVENT == BUTTON_EVENT:
                    was_pressed = self.buttons.get(axis, False)
                    self.buttons[axis] = bool(value)
                    if value and not was_pressed and not kind & INITIAL_EVENT and axis in self.button_actions:
                        self.pending_actions.append(self.button_actions[axis])

    def poll_device(self):
        """非阻塞更新设备和输入；断连时清除轴值与待处理按键。"""
        if self.fd is None:
            self.find_device()
        if self.fd is None:
            self.status = 'Xbox：等待空闲手柄（可热插拔）'
            return False
        try:
            self.read_axis_events()
        except OSError:
            self.close()
            self.status = 'Xbox：已断开，已清除驾驶输入'
            return False
        return True

    def driving_values(self, enabled):
        """驾驶输入恢复前检查回中；相机与按键不经过驾驶资格判断。"""
        steer_axis, left_axis, right_axis = self.axis_indices[:3]
        ready = all(axis in self.axes for axis in self.axis_indices[:3])
        values = (linear_trigger(self.axes.get(right_axis, -32767)),
                  linear_trigger(self.axes.get(left_axis, -32767)),
                  linear_steering(self.axes.get(steer_axis, 0)))
        neutral = max(abs(value) for value in values) < .01
        if not enabled or not ready:
            self.armed = False
            self.status = f'Xbox {self.device_name}：车辆手动输入暂停'
            return None
        if not self.armed:
            self.armed = neutral
        self.status = f'Xbox {self.device_name}：' + ('已就绪' if self.armed else '请松开扳机并回中摇杆')
        return values if self.armed else None

    def sample(self, enabled):
        """仅读取驾驶轴，供独立输入检查使用；应用入口使用 update。"""
        return self.driving_values(enabled) if self.poll_device() else None

    def handle_buttons(self, node):
        """仅处理按下边沿；按住不连发，初始化时的按下状态不触发动作。"""
        actions, self.pending_actions = self.pending_actions, []
        if not node.camera_control_enabled:
            return
        for action in actions:
            if action == 'autopilot':
                node.set_autopilot(not node.autopilot)
            elif action == 'reverse' and node.follow and not node.autopilot:
                node.toggle_reverse()
            elif action == 'free_follow':
                if node.free_follow:
                    node.set_camera_mode(True)
                else:
                    node.set_autopilot(True)
                    node.set_camera_mode(False, free_follow=True)

    def update_free_camera(self, node):
        """自由跟随时只调整相机，不把扳机或摇杆发送为车辆驾驶命令。"""
        now = time.monotonic()
        elapsed = max(0., min(now - self.camera_time, MAX_CAMERA_ELAPSED))
        self.camera_time = now
        self.camera_input = (0., 0., 0., 0., 0.)
        if not node.free_follow or not node.camera_control_enabled or len(self.axis_indices) < 6:
            return
        left_x, left, right, right_x, right_y, left_y = self.axis_indices
        forward = -linear_steering(self.axes.get(left_y, 0))
        strafe = linear_steering(self.axes.get(left_x, 0))
        rise = linear_trigger(self.axes.get(right, -32767)) - linear_trigger(self.axes.get(left, -32767))
        turn = linear_steering(self.axes.get(right_x, 0))
        tilt = -linear_steering(self.axes.get(right_y, 0))
        # 面板正在操作视角时优先，松开后恢复当前手柄输入。
        if node.camera_panel_active:
            return
        self.camera_input = (forward, strafe, rise, turn, tilt)
        if not any(self.camera_input):
            return
        with node.camera_lock:
            if node.free_pose is None:
                return
            x, y, z, pitch, yaw = node.free_pose
            yaw = (yaw + turn * CAMERA_YAW_SPEED * elapsed) % 360.
            pitch = max(-89., min(89., pitch + tilt * CAMERA_PITCH_SPEED * elapsed))
            distance = self.camera_speed * elapsed
            heading = math.radians(yaw)
            node.free_pose = (x + distance * (forward * math.cos(heading) - strafe * math.sin(heading)),
                              y + distance * (forward * math.sin(heading) + strafe * math.cos(heading)),
                              z + distance * rise, pitch, yaw)

    def update(self, node):
        """统一处理一次输入：按键切换模式，再把轴值分配给车辆或相机。"""
        if not self.poll_device():
            self.camera_input = (0., 0., 0., 0., 0.)
            self.camera_time = time.monotonic()
            return None
        self.handle_buttons(node)
        values = self.driving_values(node.follow and node.camera_control_enabled and not node.autopilot)
        self.update_free_camera(node)
        if node.camera_control_enabled and node.free_follow:
            self.status = f'Xbox {self.device_name}：自由跟随 / 自动驾驶{"开" if node.autopilot else "关"}'
        elif node.camera_control_enabled and node.autopilot:
            self.status = f'Xbox {self.device_name}：自动驾驶（A 关闭；面板可接管）'
        return values

    def close(self):
        """正常退出、断连时释放；进程退出也由系统释放设备锁。"""
        if self.fd is not None:
            os.close(self.fd)
        if self.device_lock is not None:
            self.device_lock.close()
        self.fd = self.device_lock = None
        self.axes.clear()
        self.buttons.clear()
        self.pending_actions.clear()
        self.camera_input = (0., 0., 0., 0., 0.)
        self.armed = False
