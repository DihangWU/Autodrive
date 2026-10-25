"""Lincoln 内嵌自由视角：复用独立面板控件和运算，不启动额外窗口或进程。"""
import queue
import time
import tkinter as tk
from types import SimpleNamespace
import carla
from foundation.carla.mouse_move.mouse_move import (
    CameraPanel, DEFAULT_SPEED, DEFAULT_TURN_SPEED, MAX_ELAPSED,
    SPEED_FACTOR, MIN_SPEED, MAX_SPEED, move_camera, preset_view,
)

HORIZONTAL_TURN_SPEED = 180.0  # 左右转头速度，度/秒。
KEY_ACTIONS = {'w': 'forward', 's': 'back', 'a': 'left', 'd': 'right',
               'q': 'down', 'e': 'up'}


class FreeCameraControls(CameraPanel):
    def __init__(self, window, node):
        # 只复用控件，不调用会创建独立相机进程的 CameraPanel.__init__。
        self.root, self.node = window, node
        self.args = SimpleNamespace(host=node.args.host, port=node.args.port)
        self.commands = queue.SimpleQueue()
        self.actions, self.keys, self.releases = set(), set(), {}
        self.auto_forward = False
        self.speed = DEFAULT_SPEED
        self.closed = False
        self.drag_position = None
        self.last_time = time.monotonic()
        self.enabled = None
        self.build_controls()
        # 车辆驾驶区在左，自由视角区在右；仍然只有一个顶层窗口。
        self.controls = window.winfo_children()[-1]
        self.controls.pack_configure(side='right', anchor='n', fill='y')
        self.controls.winfo_children()[-1].pack_configure(side='bottom')
        self.speed_buttons = self.controls.winfo_children()[-2]
        for widget in self.speed_buttons.winfo_children():
            if widget.cget('text') == '贴靠 CARLA':
                widget.destroy()  # 由综合面板在驾驶区提供，跟随时仍可使用。
        for widget in self.speed_buttons.winfo_children():
            widget.pack_forget()
        for column, widget in enumerate(self.speed_buttons.winfo_children()):
            self.speed_buttons.columnconfigure(column, weight=1, uniform='speed_buttons')
            widget.grid(row=0, column=column, sticky='ew')
        header = self.controls.winfo_children()[0]
        for widget in header.winfo_children():
            if isinstance(widget, tk.Label) and widget.cget('text') == '拖动\n视角':
                widget.destroy()
            elif 'wraplength' in widget.keys():
                widget.configure(wraplength=0)
        self.mode_changed()

    def clear_keyboard(self):
        self.keys.clear()
        for callback in self.releases.values():
            self.root.after_cancel(callback)
        self.releases.clear()

    def mode_changed(self):
        self.clear_keyboard()
        self.stop_motion()
        self.release_mouse()
        self.last_time = time.monotonic()
        self.set_controls_enabled(not self.node.follow and self.node.camera_control_enabled)

    def set_controls_enabled(self, enabled):
        if enabled == self.enabled:
            return
        self.enabled = enabled
        if enabled:
            self.controls.pack(side='right', anchor='n', fill='y')
        else:
            self.controls.pack_forget()
        self.root.geometry('')  # 按可见控件重新计算窗口尺寸，不留下右侧空白。
        def configure_children(widget):
            for child in widget.winfo_children():
                if 'state' in child.keys():
                    child.configure(state='normal' if enabled else 'disabled')
                configure_children(child)
        configure_children(self.controls)
        self.status.set('自由视角：WASD 平移 · Q/E 升降 · Shift 加速' if enabled else
                        '跟随模式：WASD 驾驶车辆；释放视角后启用下方控件')

    def send_actions(self):
        keyboard = {KEY_ACTIONS[key] for key in self.keys if key in KEY_ACTIONS}
        actions = self.actions | keyboard | ({'forward'} if self.auto_forward else set())
        self.commands.put(('actions', actions))

    def press(self, event):
        key = event.keysym.lower()
        if not self.enabled or key not in (*KEY_ACTIONS, 'shift_l', 'shift_r'):
            return
        pending = self.releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if key not in self.keys:
            if key == 's':
                self.stop_motion()
            self.keys.add(key)
            self.send_actions()
        return 'break'

    def release(self, event):
        key = event.keysym.lower()
        if key not in self.keys:
            return
        def finish():
            self.releases.pop(key, None)
            self.keys.discard(key)
            self.send_actions()
        if key not in self.releases:
            self.releases[key] = self.root.after(20, finish)
        return 'break'

    def stop_motion(self):
        self.clear_keyboard()
        super().stop_motion()

    def lose_focus(self):
        if self.closed:
            return
        focused = self.root.focus_displayof()
        if focused is None or focused.winfo_toplevel() != self.root:
            # 与独立面板一致，失焦只取消临时操作，保留明确开启的自动前进。
            self.clear_keyboard()
            self.release_mouse()

    def wheel(self, event):
        if not self.enabled:
            return
        steps = (1 if event.num == 4 else -1) if getattr(event, 'num', None) in (4, 5) else event.delta / 120
        if steps:
            self.change_speed(SPEED_FACTOR ** steps)
        return 'break'

    def change_speed(self, factor):
        """立即更新速度，滑块回调未执行时也能供连续移动使用。"""
        self.speed_slider.set(max(MIN_SPEED, min(MAX_SPEED, self.speed * factor)))
        self.set_speed(self.speed_slider.get())

    def update(self):
        with self.node.camera_owner.access() as owned:
            enabled = owned and self.node.camera_control_enabled and not self.node.follow
        if not enabled and self.enabled:
            self.mode_changed()
        self.set_controls_enabled(enabled)
        if enabled:
            self.status.set(('自由跟随：位置随车移动，朝向自主控制' if self.node.free_follow else
                             '自由视角：WASD 平移 · Q/E 升降 · Shift 加速'))
        elapsed = min(time.monotonic() - self.last_time, MAX_ELAPSED)
        self.last_time = time.monotonic()
        view = None
        # 每轮使用当前按键状态；队列只暂存本轮输入，不执行任何 CARLA RPC。
        while not self.commands.empty():
            command = self.commands.get_nowait()
            if command[0] == 'view':
                view = command[1]
        if not enabled:
            return
        actions = self.actions | {KEY_ACTIONS[k] for k in self.keys if k in KEY_ACTIONS}
        if self.auto_forward:
            actions.add('forward')
        if not actions and view is None:
            return
        with self.node.camera_lock:
            if self.node.free_pose is None:
                return
            x, y, z, pitch, yaw = self.node.free_pose
            pose = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw))
            if view is not None:
                pose = preset_view(pose, view)
            speed = self.speed * (4 if self.keys & {'shift_l', 'shift_r'} else 1)
            turn_speed = HORIZONTAL_TURN_SPEED if actions & {'look_left', 'look_right'} else DEFAULT_TURN_SPEED
            pose = move_camera(pose, actions, elapsed, speed, turn_speed, (0., 0.))
            self.node.free_pose = (pose.location.x, pose.location.y, pose.location.z,
                                   pose.rotation.pitch, pose.rotation.yaw)

    def close(self):
        self.closed = True
        try:
            self.clear_keyboard()
            self.release_mouse()
        except tk.TclError:
            pass  # 顶层窗口可能已由应用退出流程销毁。
