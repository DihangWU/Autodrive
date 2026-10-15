"""独立的 CARLA 自由视角悬浮面板，只控制 spectator，不操作车辆或仿真时钟。"""

import argparse
import math
import multiprocessing
import queue
import re
import subprocess
import time
import tkinter as tk
from tkinter import ttk


def move_camera(transform, actions, elapsed, speed, turn_speed, drag, ground_locked=False):
    """根据按住的按钮和拖拽增量，计算新的相机位置、朝向。"""
    import carla
    yaw = transform.rotation.yaw + drag[0]
    pitch = transform.rotation.pitch + drag[1]
    yaw += (('look_right' in actions) - ('look_left' in actions)) * turn_speed * elapsed
    pitch += (('look_up' in actions) - ('look_down' in actions)) * turn_speed * elapsed
    rotation = carla.Rotation(pitch=max(-89, min(89, pitch)),
                              yaw=(yaw + 180) % 360 - 180, roll=transform.rotation.roll)
    if ground_locked:
        rotation.pitch, rotation.roll = -45, 0
    # 锁定俯视时，用水平朝向计算平移，避免前进时随视线向地面下降。
    movement_rotation = carla.Rotation(yaw=rotation.yaw) if ground_locked else rotation
    forward, right = movement_rotation.get_forward_vector(), movement_rotation.get_right_vector()
    longitudinal = ('forward' in actions) - ('back' in actions)
    lateral = ('right' in actions) - ('left' in actions)
    vertical = ('up' in actions) - ('down' in actions)
    dx = longitudinal * forward.x + lateral * right.x
    dy = longitudinal * forward.y + lateral * right.y
    dz = longitudinal * forward.z + lateral * right.z + vertical
    length = max(1, math.sqrt(dx * dx + dy * dy + dz * dz))
    distance = speed * elapsed / length
    position = transform.location
    return carla.Transform(carla.Location(x=position.x + dx * distance,
                                         y=position.y + dy * distance,
                                         z=position.z + dz * distance), rotation)


def preset_view(transform, mode):
    """保留位置和水平方向，恢复水平或切到 45 度俯视。"""
    import carla
    pitch = {'level': 0, 'ground': -45}[mode]
    return carla.Transform(transform.location,
                           carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw, roll=0))


def carla_window_bounds():
    """可选的 X11 定位。多窗口或查询失败时交给用户手动拖动。"""
    try:
        result = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True,
                                text=True, timeout=1, check=True)
        windows = re.findall(
            r'(0x[0-9a-fA-F]+)\s+"([^"\n]+)":\s+\("([^"]*)"\s+"([^"]*)"\)', result.stdout)
        # 排除窗口管理器的同名装饰框，只选 CARLA 自身的客户端窗口。
        candidates = [wid for wid, name, instance, window_class in windows
                      if name.startswith(('CarlaUE4', 'CarlaUnreal'))
                      and 'carla' in (instance + window_class).lower()]
        if len(candidates) != 1:
            return None
        result = subprocess.run(['xwininfo', '-id', candidates[0]], capture_output=True,
                                text=True, timeout=1, check=True)
        values = [re.search(pattern, result.stdout) for pattern in (
            r'Absolute upper-left X:\s*(-?\d+)', r'Absolute upper-left Y:\s*(-?\d+)',
            r'Width:\s*(\d+)', r'Height:\s*(\d+)')]
        if all(values):
            return tuple(int(value.group(1)) for value in values)
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def camera_loop(args, commands, messages, stopped):
    """CARLA RPC 放到独立进程，超时不会阻塞 Tk 的界面进程。"""
    try:
        import carla
    except ImportError:
        messages.put(('reset', '缺少 CARLA Python API，请使用 carla-venv 启动。'))
        return
    client = carla.Client(args.host, args.port)
    client.set_timeout(3)
    speed = args.speed
    while not stopped.is_set():
        try:
            world = client.get_world()
            spectator = world.get_spectator()
            messages.put(('reset', '已连接 · 仅控制自由观察相机'))
            previous = time.monotonic()
            next_world_check = previous + 1
            target, actions = None, set()
            ground_locked = False
            while not stopped.is_set():
                now = time.monotonic()
                elapsed = min(now - previous, .1)
                previous = now
                drag = [0., 0.]
                view = None
                while True:
                    try:
                        command = commands.get_nowait()
                    except queue.Empty:
                        break
                    if command[0] == 'stop':
                        actions.clear()
                        drag = [0., 0.]
                    elif command[0] == 'actions':
                        actions = set(command[1])
                    elif command[0] == 'speed':
                        speed = command[1]
                    elif command[0] == 'drag':
                        drag[0] += command[1]
                        drag[1] += command[2]
                    elif command[0] == 'view':
                        view = command[1]
                        ground_locked = False
                        actions.clear()
                        drag = [0., 0.]
                    elif command[0] == 'ground_lock':
                        ground_locked = command[1]
                        view = 'ground' if ground_locked else None
                        actions.clear()
                        drag = [0., 0.]
                if now >= next_world_check:
                    latest_world = client.get_world()
                    if latest_world.id != world.id:
                        world, spectator = latest_world, latest_world.get_spectator()
                        target, actions, drag = None, set(), [0., 0.]
                        view = None
                        ground_locked = False
                        messages.put(('reset', '地图已切换 · 重新取得观察相机'))
                    next_world_check = now + 1
                if view is not None or actions or any(drag) or ground_locked:
                    # 连续移动使用累计位置，避免旧快照让观察相机倒退。
                    if target is None:
                        target = spectator.get_transform()
                    if view is not None:
                        target = preset_view(target, view)
                    else:
                        target = move_camera(target, actions, elapsed, speed, args.turn_speed,
                                             drag, ground_locked)
                    if not stopped.is_set():
                        spectator.set_transform(target)
                else:
                    target = None
                stopped.wait(max(0, 1 / 30 - (time.monotonic() - now)))
        except (RuntimeError, OSError) as error:
            messages.put(('reset', f'连接中断，正在重试：{error}'))
            # 断开期间的移动请求不在重连后继续执行。
            stopped.wait(2)
            while True:
                try:
                    commands.get_nowait()
                except queue.Empty:
                    break


class CameraPanel:
    def __init__(self, root, args):
        self.root, self.args = root, args
        context = multiprocessing.get_context('spawn')
        self.commands = context.Queue()
        self.messages = context.Queue()
        self.stopped = context.Event()
        self.actions = set()
        self.speed = args.speed
        self.closed = False
        self.ground_locked = False
        self.drag_position = None
        root.title('CARLA 自由视角')
        root.resizable(False, False)
        root.attributes('-topmost', True)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.build_controls()
        # 窗口外松开或窗口失焦时停止，避免远程桌面丢失松开事件后持续移动。
        root.bind_all('<ButtonRelease-1>', self.release_mouse)
        root.bind('<FocusOut>', lambda event: self.stop_motion())
        root.bind('<Unmap>', lambda event: self.stop_motion())
        root.bind_all('<MouseWheel>', self.mouse_wheel)
        root.bind_all('<Button-4>', lambda event: self.change_speed(1.25))
        root.bind_all('<Button-5>', lambda event: self.change_speed(1 / 1.25))
        self.worker = context.Process(target=camera_loop, name='carla-spectator', daemon=True,
                                      args=(args, self.commands, self.messages, self.stopped))
        self.worker.start()
        root.after(100, self.refresh_status)
        root.after(150, self.place_panel)

    def build_controls(self):
        panel = ttk.Frame(self.root, padding=10)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text=f'CARLA  {self.args.host}:{self.args.port}').pack(anchor='w')
        self.status = tk.StringVar(value='正在连接 CARLA…')
        ttk.Label(panel, textvariable=self.status, wraplength=365).pack(anchor='w', pady=(2, 8))
        groups = ttk.Frame(panel)
        groups.pack(fill='x')
        movement = ttk.LabelFrame(groups, text='移动与转头按住操作，视角预设单击', padding=5)
        movement.pack(fill='x', expand=True)
        for column in range(3):
            movement.columnconfigure(column, weight=1)
        buttons = [('左转', 'look_left', 0, 0), ('W 前进', 'forward', 0, 1),
                   ('右转', 'look_right', 0, 2), ('A 左移', 'left', 1, 0),
                   ('S 后退', 'back', 1, 1), ('D 右移', 'right', 1, 2),
                   ('Q 上升', 'up', 2, 0), ('抬头', 'look_up', 2, 1),
                   ('E 下降', 'down', 3, 0), ('低头', 'look_down', 3, 1)]
        for text, action, row, column in buttons:
            self.hold_button(movement, text, action, row, column)
        ttk.Button(movement, text='恢复水平视角', command=lambda: self.set_view('level')).grid(
            row=2, column=2, sticky='ew', padx=1, pady=2)
        self.ground_button_text = tk.StringVar(value='45° 看地面')
        self.ground_button = ttk.Button(movement, textvariable=self.ground_button_text,
                                       command=self.toggle_ground_lock)
        self.ground_button.grid(
            row=3, column=2, sticky='ew', padx=1, pady=2)
        drag_pad = tk.Label(panel, text='按住此处拖动：转动视角', bg='#dbe8f5', pady=12, cursor='fleur')
        drag_pad.pack(fill='x', pady=8)
        drag_pad.bind('<ButtonPress-1>', self.begin_drag)
        drag_pad.bind('<B1-Motion>', self.drag_view)
        self.speed_label = tk.StringVar()
        ttk.Label(panel, textvariable=self.speed_label).pack(anchor='w')
        self.speed_slider = tk.Scale(panel, from_=.5, to=100, resolution=.5, orient='horizontal',
                                     showvalue=False, command=self.set_speed, highlightthickness=0)
        self.speed_slider.set(self.speed)
        self.speed_slider.pack(fill='x')
        controls = ttk.Frame(panel)
        controls.pack(fill='x', pady=(4, 0))
        ttk.Button(controls, text='速度 −', command=lambda: self.change_speed(1 / 1.25)).pack(side='left')
        ttk.Button(controls, text='速度 ＋', command=lambda: self.change_speed(1.25)).pack(side='left', padx=4)
        ttk.Button(controls, text='停止', command=self.stop_motion).pack(side='left')
        ttk.Button(controls, text='贴靠 CARLA', command=self.place_panel).pack(side='right')
        ttk.Label(panel, text='滚轮调速 · 松开停止 · 拖动标题栏移动面板').pack(pady=(8, 0))
        self.set_speed(self.speed)

    def hold_button(self, parent, text, action, row, column):
        button = ttk.Button(parent, text=text, width=12)
        button.grid(row=row, column=column, sticky='ew', padx=1, pady=2)
        def press(event):
            self.stop_motion()
            self.actions.add(action)
            self.commands.put(('actions', list(self.actions)))
            button.grab_set()
            return 'break'
        button.bind('<ButtonPress-1>', press)

    def release_mouse(self, event=None):
        self.stop_motion()
        grabbed = self.root.grab_current()
        if grabbed is not None:
            grabbed.grab_release()

    def stop_motion(self):
        self.actions.clear()
        self.commands.put(('stop',))
        self.drag_position = None

    def set_view(self, mode):
        self.stop_motion()
        self.update_ground_lock(False)
        self.commands.put(('view', mode))

    def update_ground_lock(self, locked):
        self.ground_locked = locked
        self.ground_button_text.set('45° 已锁定 · 点击释放' if locked else '45° 看地面')
        self.ground_button.state(['pressed'] if locked else ['!pressed'])

    def toggle_ground_lock(self):
        self.stop_motion()
        self.update_ground_lock(not self.ground_locked)
        self.commands.put(('ground_lock', self.ground_locked))

    def begin_drag(self, event):
        self.stop_motion()
        self.drag_position = (event.x_root, event.y_root)
        event.widget.grab_set()

    def drag_view(self, event):
        if self.drag_position is None:
            return
        old_x, old_y = self.drag_position
        self.commands.put(('drag', (event.x_root - old_x) * .2,
                            -(event.y_root - old_y) * .2))
        self.drag_position = (event.x_root, event.y_root)

    def set_speed(self, value):
        self.speed = float(value)
        self.commands.put(('speed', self.speed))
        self.speed_label.set(f'移动速度：{self.speed:.1f} 米/秒')

    def change_speed(self, factor):
        self.speed_slider.set(max(.5, min(100, self.speed * factor)))

    def mouse_wheel(self, event):
        self.change_speed(1.25 if event.delta > 0 else 1 / 1.25)

    def place_panel(self):
        self.root.update_idletasks()
        width, height = self.root.winfo_width(), self.root.winfo_height()
        bounds = carla_window_bounds()
        if bounds:
            x, y, window_width, window_height = bounds
            x, y = x + window_width - width - 16, y + window_height - height - 40
        else:
            x = self.root.winfo_screenwidth() - width - 24
            y = self.root.winfo_screenheight() - height - 64
        self.root.geometry(f'+{max(0, x)}+{max(0, y)}')

    def refresh_status(self):
        while True:
            try:
                kind, text = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == 'reset':
                self.stop_motion()
                self.update_ground_lock(False)
                self.commands.put(('ground_lock', False))
                self.commands.put(('speed', self.speed))
            self.status.set(text)
        if not self.stopped.is_set():
            self.root.after(100, self.refresh_status)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop_motion()
        self.stopped.set()
        self.root.destroy()
        self.worker.join(timeout=.15)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=1)
        self.commands.close()
        self.messages.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='CARLA 地址')
    parser.add_argument('--port', type=int, default=2000, help='CARLA RPC 端口')
    parser.add_argument('--speed', type=float, default=10, help='初始移动速度，米/秒，默认 10')
    parser.add_argument('--turn-speed', type=float, default=45, help='按钮转头速度，度/秒，默认 45')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not .5 <= args.speed <= 100 or not 0 < args.turn_speed <= 180:
        parser.error('端口为 1～65535，速度为 0.5～100，转头速度大于 0 且不超过 180。')
    root = tk.Tk()
    panel = CameraPanel(root, args)
    try:
        root.mainloop()
    finally:
        panel.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ImportError, tk.TclError) as error:
        raise SystemExit(f'错误：{error}')
