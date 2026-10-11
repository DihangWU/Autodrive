"""Read X11 input only while the CARLA render window is focused.

No tick ownership and no input interception: pointer motion is sampled/recentered.
"""
import ctypes as C
import math
import time


class CarlaWindowInput:
    def __init__(self, node):
        self.node = node
        self.x = C.CDLL('libX11.so.6')
        self.fx = C.CDLL('libXfixes.so.3')
        ptr, window = C.c_void_p, C.c_ulong
        signatures = {
            'XOpenDisplay': ([C.c_char_p], ptr), 'XDefaultRootWindow': ([ptr], window),
            'XInternAtom': ([ptr, C.c_char_p, C.c_int], window),
            'XGetWindowProperty': ([ptr, window, window, C.c_long, C.c_long, C.c_int, window,
                                   C.POINTER(window), C.POINTER(C.c_int), C.POINTER(window),
                                   C.POINTER(window), C.POINTER(C.POINTER(C.c_ubyte))], C.c_int),
            'XFetchName': ([ptr, window, C.POINTER(C.c_void_p)], C.c_int),
            'XFree': ([ptr], C.c_int), 'XCloseDisplay': ([ptr], C.c_int),
            'XQueryPointer': ([ptr, window, C.POINTER(window), C.POINTER(window),
                               C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_int),
                               C.POINTER(C.c_int), C.POINTER(C.c_uint)], C.c_int),
            'XGetGeometry': ([ptr, window, C.POINTER(window), C.POINTER(C.c_int), C.POINTER(C.c_int),
                              C.POINTER(C.c_uint), C.POINTER(C.c_uint), C.POINTER(C.c_uint),
                              C.POINTER(C.c_uint)], C.c_int),
            'XWarpPointer': ([ptr, window, window, C.c_int, C.c_int, C.c_uint, C.c_uint, C.c_int, C.c_int], C.c_int),
            'XFlush': ([ptr], C.c_int), 'XQueryKeymap': ([ptr, C.c_void_p], C.c_int),
            'XStringToKeysym': ([C.c_char_p], window), 'XKeysymToKeycode': ([ptr, window], C.c_ubyte),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.x, name);function.argtypes = args;function.restype = result
        for name in ('XFixesHideCursor', 'XFixesShowCursor'):
            getattr(self.fx, name).argtypes = [ptr, window]
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('Cannot connect to X11 display')
        self.root = self.x.XDefaultRootWindow(self.display)
        self.active_atom = self.x.XInternAtom(self.display, b'_NET_ACTIVE_WINDOW', 0)
        self.codes = {key: self.x.XKeysymToKeycode(self.display, self.x.XStringToKeysym(key.encode()))
                      for key in ('w', 'a', 's', 'd', 'q', 'e', 'f', 'r', 'Escape', 'Shift_L', 'Shift_R')}
        self.window = 0
        self.hidden = False
        self.paused = False
        self.previous_keys = set()
        self.last_time = time.monotonic()
        self.reset_pose = None

    def active_window(self):
        actual, count, remaining = C.c_ulong(), C.c_ulong(), C.c_ulong()
        fmt = C.c_int();data = C.POINTER(C.c_ubyte)()
        self.x.XGetWindowProperty(self.display, self.root, self.active_atom, 0, 1, 0, 0,
                                 C.byref(actual), C.byref(fmt), C.byref(count), C.byref(remaining), C.byref(data))
        active = C.cast(data, C.POINTER(C.c_ulong))[0] if data and count.value and fmt.value == 32 else 0
        if data:self.x.XFree(data)
        if not active:return 0
        name = C.c_void_p()
        self.x.XFetchName(self.display, active, C.byref(name))
        title = C.string_at(name).decode(errors='replace') if name.value else ''
        if name.value:self.x.XFree(name)
        # Exclude the standalone controller window and all RViz/panel windows.
        return active if ('CarlaUE4' in title or title.strip().lower() == 'carla') else 0

    def release(self):
        if self.hidden:
            self.fx.XFixesShowCursor(self.display, self.window)
            self.x.XFlush(self.display)
        self.hidden = False
        self.previous_keys.clear()

    def update(self, speed=20.):
        now = time.monotonic();dt = min(now - self.last_time, .1);self.last_time = now
        active = self.active_window() if not self.node.follow and self.node.camera_control_enabled else 0
        if not active:
            self.release();self.window = 0;self.paused = False
            return
        first = active != self.window
        if first:
            self.release();self.window = active;self.paused = False;self.reset_pose = self.node.free_pose
        bits = C.create_string_buffer(32);self.x.XQueryKeymap(self.display, bits)
        keys = {key for key, code in self.codes.items() if code and bits.raw[code // 8] & (1 << (code % 8))}
        fresh = keys - self.previous_keys;self.previous_keys = keys
        if 'f' in fresh:self.paused = not self.paused
        if 'Escape' in fresh:self.paused = True
        if self.paused:
            if self.hidden:
                self.fx.XFixesShowCursor(self.display, self.window);self.x.XFlush(self.display);self.hidden = False
            return
        root, child = C.c_ulong(), C.c_ulong()
        rx, ry, px, py = C.c_int(), C.c_int(), C.c_int(), C.c_int();mask=C.c_uint()
        width, height, border, depth=C.c_uint(), C.c_uint(), C.c_uint(), C.c_uint();gx,gy=C.c_int(),C.c_int()
        if not self.x.XGetGeometry(self.display, active, C.byref(root), C.byref(gx), C.byref(gy),
                                  C.byref(width), C.byref(height), C.byref(border), C.byref(depth)):return
        if not self.x.XQueryPointer(self.display, active, C.byref(root), C.byref(child), C.byref(rx), C.byref(ry),
                                   C.byref(px), C.byref(py), C.byref(mask)):return
        cx,cy=width.value//2,height.value//2
        with self.node.camera_lock:
            if self.node.free_pose is None:return
            x,y,z,pitch,yaw=self.reset_pose if 'r' in fresh else self.node.free_pose
            if not first and self.hidden:
                yaw=(yaw+(px.value-cx)*.18)%360.
                pitch=max(-89.,min(89.,pitch-(py.value-cy)*.18))
            angle=math.radians(yaw)
            forward=int('w' in keys)-int('s' in keys);right=int('d' in keys)-int('a' in keys);up=int('e' in keys)-int('q' in keys)
            dx=forward*math.cos(angle)-right*math.sin(angle);dy=forward*math.sin(angle)+right*math.cos(angle)
            magnitude=math.sqrt(dx*dx+dy*dy+up*up)
            if magnitude:
                scale=speed*dt/magnitude*(4 if keys & {'Shift_L','Shift_R'} else 1)
                x+=dx*scale;y+=dy*scale;z+=up*scale
            self.node.free_pose=(x,y,z,pitch,yaw)
        if not self.hidden:
            self.fx.XFixesHideCursor(self.display,active);self.hidden=True
        self.x.XWarpPointer(self.display,0,active,0,0,0,0,cx,cy);self.x.XFlush(self.display)

    def close(self):
        self.release();self.x.XCloseDisplay(self.display)
