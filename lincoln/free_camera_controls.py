"""Embedded free-spectator controls; input stays scoped to its Tk canvas."""
import math
import time
import tkinter as tk


class FreeCameraControls:
    def __init__(self, window, node):
        self.node, self.window = node, window
        self.keys, self.releases = set(), {}
        self.speed = 20.
        self.captured = False
        self.last_mouse = None
        self.reset_pose = None
        self.last_time = time.monotonic()
        self.render_input = None
        try:
            from carla_window_input import CarlaWindowInput
            self.render_input = CarlaWindowInput(node)
        except (OSError, RuntimeError) as error:
            print(f'CARLA-window input unavailable; panel controls remain available: {error}', flush=True)
        self.canvas = tk.Canvas(window, width=400, height=100, background='#181d27', takefocus=True)
        self.canvas.pack(padx=8, pady=6)
        self.text = self.canvas.create_text(200, 50, fill='white', text='')
        self.canvas.bind('<ButtonPress-1>', self.focus)
        self.canvas.bind('<KeyPress>', self.press)
        self.canvas.bind('<KeyRelease>', self.release)
        self.canvas.bind('<Motion>', self.motion)
        self.canvas.bind('<MouseWheel>', lambda e: self.wheel(e.delta / 120))
        self.canvas.bind('<Button-4>', lambda e: self.wheel(1))
        self.canvas.bind('<Button-5>', lambda e: self.wheel(-1))
        self.canvas.bind('<FocusOut>', lambda e: self.clear())
        self.canvas.bind('<Leave>', lambda e: self.clear() if not self.captured else None)
        self.refresh()

    def refresh(self):
        self.canvas.itemconfigure(self.text, text=('跟随视角已开启' if self.node.follow else
            '自由视角：点击 CARLA 窗口即可鼠标转向\nWASD 移动  Q/E 升降  Shift 加速\nF/Esc 释放鼠标；也可在此区域拖动、滚轮调速\n'
            f'移动速度：{self.speed:.1f} m/s'))

    def clear(self):
        self.keys.clear()
        for callback in self.releases.values():
            self.window.after_cancel(callback)
        self.releases.clear()
        self.capture(False)
        self.last_mouse = None

    def mode_changed(self):
        self.clear()
        if self.render_input:
            self.render_input.release()
        self.reset_pose = self.node.free_pose
        self.refresh()

    def focus(self, event):
        if not self.node.follow:
            self.node.camera_control_enabled = True
            self.canvas.focus_set()
            self.last_mouse = (event.x, event.y)
        return 'break'

    def capture(self, enabled):
        self.captured = enabled and not self.node.follow
        self.canvas.configure(cursor='none' if self.captured else '')
        if self.captured:
            self.canvas.grab_set()
            self.center()
        elif self.canvas.grab_current() == self.canvas:
            self.canvas.grab_release()

    def center(self):
        x, y = self.canvas.winfo_width() // 2, self.canvas.winfo_height() // 2
        self.last_mouse = (x, y)
        self.canvas.event_generate('<Motion>', x=x, y=y, warp=True)

    def press(self, event):
        if self.node.follow:
            return
        key = event.keysym.lower()
        callback = self.releases.pop(key, None)
        if callback:
            self.window.after_cancel(callback)
        if key in self.keys:
            return 'break'
        self.keys.add(key)
        if key == 'f':
            self.capture(not self.captured)
        elif key == 'escape':
            self.capture(False)
        elif key == 'r' and self.reset_pose:
            with self.node.camera_lock:
                self.node.free_pose = self.reset_pose
        return 'break'

    def release(self, event):
        if self.node.follow:
            return
        key = event.keysym.lower()
        def finish():
            self.releases.pop(key, None)
            self.keys.discard(key)
        if key not in self.releases:
            self.releases[key] = self.window.after(20, finish)
        return 'break'

    def motion(self, event):
        if self.node.follow or not (self.captured or event.state & 0x100):
            self.last_mouse = (event.x, event.y)
            return
        if self.last_mouse is not None:
            dx, dy = event.x - self.last_mouse[0], event.y - self.last_mouse[1]
            if dx or dy:
                with self.node.camera_lock:
                    x, y, z, pitch, yaw = self.node.free_pose
                    self.node.free_pose = (x, y, z, max(-89., min(89., pitch - dy * .18)), (yaw + dx * .18) % 360.)
                if self.captured:
                    self.center()
                    return
        self.last_mouse = (event.x, event.y)

    def wheel(self, steps):
        if not self.node.follow:
            self.speed = max(1., min(300., self.speed * 1.2 ** steps))
            self.refresh()
        return 'break'

    def update(self):
        if self.render_input:
            self.render_input.update(self.speed)
        now = time.monotonic()
        dt = min(now - self.last_time, .1)
        self.last_time = now
        if self.node.follow or not self.keys:
            return
        with self.node.camera_lock:
            x, y, z, pitch, yaw = self.node.free_pose
            angle = math.radians(yaw)
            forward = int('w' in self.keys) - int('s' in self.keys)
            right = int('d' in self.keys) - int('a' in self.keys)
            up = int('e' in self.keys) - int('q' in self.keys)
            dx = forward * math.cos(angle) - right * math.sin(angle)
            dy = forward * math.sin(angle) + right * math.cos(angle)
            magnitude = math.sqrt(dx * dx + dy * dy + up * up)
            if magnitude:
                scale = self.speed * dt / magnitude
                if self.keys & {'shift_l', 'shift_r'}:
                    scale *= 4
                self.node.free_pose = (x + dx * scale, y + dy * scale, z + up * scale, pitch, yaw)

    def close(self):
        try:
            self.clear()
        except tk.TclError:
            pass  # Tk may already have destroyed the panel on normal exit.
        if self.render_input:
            self.render_input.close()
            self.render_input = None
