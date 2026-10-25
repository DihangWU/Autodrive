"""Lincoln 综合面板：跟随时键盘驾驶，释放后键盘控制自由视角。"""
import math
import tkinter as tk
from tkinter import ttk


def navigation_control_display(status, automatic):
    """自动驾驶开关表示整项任务；此行单独说明实际控制者与等待原因。"""
    state = status.get('state')
    if state == 'unreachable':
        return '导航失败：目的地不可达', '#b00020'
    if state == 'error':
        return '导航异常：已制动', '#b00020'
    if state == 'arrived':
        return '导航完成：停车保持', '#166534'
    if state == 'planning':
        if status.get('display_preparing'):
            return '导航控制：准备地图与 RViz，停车等待', '#946200'
        return '导航控制：规划中，停车等待', '#946200'
    mode = status.get('controller')
    if state == 'navigating':
        if mode == 'local':
            waiting = {'traffic_light': ' · 红黄灯等待', 'obstacle': ' · 前方障碍物等待'}
            return '导航控制：路口局部控制' + waiting.get(status.get('control_detail'), ''), '#b45309'
        if mode == 'preparing':
            return '导航控制：准备路口接管（TM）', '#946200'
        if mode == 'stopping':
            return '导航控制：正在制动停车', '#b00020'
        return '导航控制：TM', '#1557b0'
    return ('驾驶控制：TM（普通自动驾驶）', '#1557b0') if automatic else ('驾驶控制：手动', '#444444')


def run_mouse_driving_panel(node, done):
    window = tk.Tk()
    window.title('Lincoln 驾驶与视角 — ' + node.uid)
    driving = tk.Frame(window)
    driving.pack(side='left', anchor='n', fill='y', padx=6)
    tk.Label(driving, text=f'{node.uid}\n鼠标按住驾驶；W 油门 / S 刹车 / A D 转向 / Q 切换前进倒车'
                          '\n油门右键：定速／取消；非定速时松开或失焦收油滑行').pack(pady=8)
    navigation_status = tk.Label(driving, text='驾驶控制：手动', wraplength=450)
    navigation_status.pack(pady=(0, 4))
    pressed_keys = set()
    pending_releases = {}
    def stop_driving(event=None):
        pressed_keys.clear()
        for callback in pending_releases.values():
            window.after_cancel(callback)
        pending_releases.clear()
        node.active = None
        node.drive()
    grid = tk.Frame(driving)
    grid.pack()
    def activate(action):
        node.set_autopilot(False)
        stop_driving()
        node.active = action
        node.drive()
    def toggle_cruise(event=None):
        node.set_autopilot(False)
        if node.cruise_speed is not None:
            node.cruise_speed = None
        else:
            velocity = node.car.get_velocity()
            node.cruise_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            node.cruise_integral = node.car.get_control().throttle
            node.cruise_time = None
        stop_driving()
        return 'break'
    driving_buttons = {}
    for i, (label, action) in enumerate([('油门', 'go'), ('左转', 'left'),
                                       ('刹车', 'brake'), ('右转', 'right')]):
        button = tk.Button(grid, text=label, width=12, height=3)
        driving_buttons[action] = button
        if i == 0:
            button.grid(row=0, column=1, padx=4, pady=4)
        else:
            button.grid(row=1, column=i - 1, padx=4, pady=4)
        button.bind('<ButtonPress-1>',
                    lambda event, a=action: activate(a))
        if action == 'go':
            button.bind('<ButtonPress-3>', toggle_cruise)
    speed_input = tk.Frame(grid)
    speed_input.grid(row=0, column=2, padx=4, pady=4)
    tk.Label(speed_input, text='目标速度 km/h').pack()
    target_entry = tk.Entry(speed_input, width=12)
    target_entry.pack()
    target_hint = tk.Label(speed_input, text='输入后按 Enter')
    target_hint.pack()
    def set_target_speed(event=None):
        try:
            kmh = float(target_entry.get().strip())
            if not math.isfinite(kmh) or kmh < 0:
                raise ValueError
        except ValueError:
            target_hint.config(text='请输入非负数字', fg='red')
            return 'break'
        node.set_autopilot(False)
        node.cruise_speed = kmh / 3.6
        node.cruise_integral = node.car.get_control().throttle
        node.cruise_time = None
        stop_driving()
        target_hint.config(text=f'目标 {kmh:g} km/h', fg='black')
        window.focus_set()
        return 'break'
    target_entry.bind('<Return>', set_target_speed)
    target_entry.bind('<KP_Enter>', set_target_speed)
    def toggle_autopilot():
        stop_driving()
        node.set_autopilot(not node.autopilot)
    autopilot_button = tk.Button(grid, text='开启自动驾驶', width=12, height=3,
                                 command=toggle_autopilot)
    autopilot_button.grid(row=0, column=0, padx=4, pady=4)
    def release(event=None):
        stop_driving()
        free_camera.release_mouse(event)
    window.bind_all('<ButtonRelease-1>', release)
    def focus_out(event=None):
        # Moving focus between buttons must not cancel a command.
        window.after_idle(lambda: stop_driving() if window.focus_get() is None else None)
        window.after_idle(lambda: free_camera.lose_focus())
    window.bind('<FocusOut>', focus_out)
    window.bind('<Leave>', lambda event: release() if event.widget == window else None)
    def reverse():
        node.toggle_reverse()
        gear.config(text='当前：倒车' if node.reverse else '当前：前进')
    gear = tk.Button(driving, text='当前：前进', command=reverse)
    gear.pack(pady=5)
    xbox_status = tk.Label(driving, text='Xbox：等待空闲手柄', wraplength=450)
    xbox_status.pack()
    def keyboard_drive():
        if pressed_keys & {'w', 'a', 's', 'd'}:
            node.set_autopilot(False)
        parts = []
        if 's' in pressed_keys:
            parts.append('brake')
        elif 'w' in pressed_keys:
            parts.append('go')
        if ('a' in pressed_keys) != ('d' in pressed_keys):
            parts.append('left' if 'a' in pressed_keys else 'right')
        node.active = '-'.join(parts) or None
        node.drive()
    def key_press(event):
        if isinstance(event.widget, (tk.Entry, tk.Spinbox)):
            return
        if not node.follow:
            return free_camera.press(event)
        key = event.keysym.lower()
        if key not in ('w', 'a', 's', 'd', 'q'):
            return
        callback = pending_releases.pop(key, None)
        if callback is not None:
            window.after_cancel(callback)
        if key not in pressed_keys:
            pressed_keys.add(key)
            if key == 'q':
                reverse()
            keyboard_drive()
        return 'break'
    def key_release(event):
        if not node.follow:
            return free_camera.release(event)
        key = event.keysym.lower()
        if key not in pressed_keys:
            return
        def finish():
            pending_releases.pop(key, None)
            pressed_keys.discard(key)
            keyboard_drive()
        # X11 auto-repeat can emit release/press pairs for a held key.
        if key not in pending_releases:
            pending_releases[key] = window.after(20, finish)
        return 'break'
    window.bind('<KeyPress>', key_press)
    window.bind('<KeyRelease>', key_release)
    from modules.lincoln.ui.free_camera_controls import FreeCameraControls
    def toggle_camera_mode():
        stop_driving()
        node.set_camera_mode(not node.follow)
        free_camera.mode_changed()
        window.focus_set()
        camera_button.config(text='释放视角' if node.follow else '跟随本车')
    camera_row = tk.Frame(driving)
    camera_row.rowconfigure(0, weight=1)
    camera_button = tk.Button(camera_row, text='释放视角' if node.follow else '跟随本车',
                              command=toggle_camera_mode)
    def toggle_free_follow():
        stop_driving()
        enable = not node.free_follow
        if enable:
            node.set_autopilot(True)
        node.set_camera_mode(False, free_follow=enable)
        free_camera.mode_changed()
        window.focus_set()
    free_follow_button = tk.Button(camera_row, text='自由跟随', command=toggle_free_follow)
    node.set_camera_mode(node.follow, acquire=node.follow, replace=False)
    free_camera = FreeCameraControls(window, node)
    window.bind('<MouseWheel>', free_camera.wheel)
    window.bind('<Button-4>', free_camera.wheel)
    window.bind('<Button-5>', free_camera.wheel)
    speed = tk.Label(driving, borderwidth=0, pady=0)
    speed.pack(side='bottom', pady=(16, 10))
    docking = ttk.Button(camera_row, text='贴靠 CARLA', command=free_camera.place_panel)
    for column, button in enumerate((free_follow_button, camera_button, docking)):
        camera_row.columnconfigure(column, weight=1, uniform='camera_buttons')
        button.grid(row=0, column=column, sticky='nsew')
    # 为收起右侧面板后的底部按钮预留高度，避免覆盖驾驶控件。
    camera_row_space = tk.Frame(driving)
    camera_row_space.pack(side='bottom', pady=8)
    def align_camera_buttons(event=None):
        """左侧三键等宽等高，展开时与右侧三键保持同一水平行。"""
        references = free_camera.speed_buttons.winfo_children()
        reference = references[0]
        width = sum(max(button.winfo_reqwidth(), button.winfo_width()) for button in references)
        height = reference.winfo_height() if reference.winfo_height() > 1 else reference.winfo_reqheight()
        camera_row_space.configure(height=height)
        if free_camera.controls.winfo_ismapped():
            y = free_camera.speed_buttons.winfo_rooty() - driving.winfo_rooty()
        else:
            y = camera_row_space.winfo_y()
        camera_row.place(relx=.5, y=y, anchor='n', width=width, height=height)
        camera_row.lift()
    free_camera.speed_buttons.bind('<Configure>', align_camera_buttons)
    free_camera.speed_buttons.bind('<Map>', align_camera_buttons)
    free_camera.speed_buttons.bind('<Unmap>', align_camera_buttons)
    for button in free_camera.speed_buttons.winfo_children():
        button.bind('<Configure>', align_camera_buttons)
    camera_row_space.bind('<Configure>', align_camera_buttons)
    driving.bind('<Configure>', align_camera_buttons)
    window.after_idle(align_camera_buttons)
    def close():
        free_camera.close()
        node.stop_requested = True
    window.protocol('WM_DELETE_WINDOW', close)
    camera_mode = (node.follow, node.free_follow)
    # 退出信号由应用入口统一处理。
    def poll():
        nonlocal camera_mode
        if done():
            window.destroy()
            return
        node.drive()
        current_camera_mode = (node.follow, node.free_follow)
        if current_camera_mode != camera_mode:
            stop_driving()
            free_camera.mode_changed()
            camera_mode = current_camera_mode
        free_camera.update()
        gear.config(text='当前：倒车' if node.reverse else '当前：前进')
        camera_button.config(text="释放视角" if node.follow else "跟随本车")
        free_follow_button.config(relief='sunken' if node.free_follow else 'raised')
        autopilot_button.config(text='关闭自动驾驶' if node.autopilot else '开启自动驾驶')
        navigation_state = node.navigation.snapshot()
        status_text, status_color = navigation_control_display(navigation_state, node.autopilot)
        navigation_status.config(text=status_text, fg=status_color)
        velocity = node.car.get_velocity()
        control = node.car.get_control()
        xbox_active = node.control_source == 'xbox'
        for action, button in driving_buttons.items():
            gamepad_pressed = xbox_active and {
                'go': control.throttle > 0., 'brake': control.brake > 0.,
                'left': control.steer < 0., 'right': control.steer > 0.,
            }[action]
            pressed = gamepad_pressed or action in (node.active or '')
            button.config(relief='sunken' if pressed else 'raised',
                          bg='#b9dfef' if gamepad_pressed else window.cget('bg'))
        xbox_status.config(text=node.xbox.status)
        command_source = 'Xbox' if xbox_active else node.active or '收油／滑行'
        if node.autopilot:
            command_source = '路口局部控制' if navigation_state.get('controller') == 'local' else 'TM 自动驾驶'
        speed.config(text=f'{math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)*3.6:.1f} km/h'
                     + (f'  定速目标：{node.cruise_speed*3.6:.1f} km/h' if node.cruise_speed is not None else '')
                     + f'\n指令：{command_source}  油门：{control.throttle:.2f}  刹车：{control.brake:.2f}')
        window.after(30, poll)
    window.after(30, poll)
    try:
        window.mainloop()
    finally:
        free_camera.close()
