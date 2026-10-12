"""Click-through, translucent overlay attached only to the CARLA game window.

No scene geometry is created. Camera, LiDAR and radar cannot capture these
pixels: Qt composites them onto the desktop after CARLA renders its viewport.
"""
import argparse
import ctypes as ct
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vanet.common.paths import LINCOLN_DIR
sys.path.insert(0, str(LINCOLN_DIR))
from shared_sumo import alive

COLORS = {
    'idle': (145, 145, 145, 110),
    'tx': (35, 130, 255, 125),
    'rx': (255, 220, 45, 125),
    'duplex': (40, 210, 95, 125),
}


def colored_segments(links, nodes):
    for a, b, directions in links:
        if a not in nodes or b not in nodes:
            continue
        first, second = nodes[a], nodes[b]
        if directions in (0, 3):
            yield first, second, COLORS['idle' if directions == 0 else 'duplex']
        else:
            midpoint = [(x + y) / 2 for x, y in zip(first, second)]
            first_color, second_color = ('tx', 'rx') if directions == 1 else ('rx', 'tx')
            yield first, midpoint, COLORS[first_color]
            yield midpoint, second, COLORS[second_color]


def project_segment(first, second, camera, width, height, fov=90):
    """Project world endpoints, clipping against the view's near plane."""
    def local(point):
        delta = [p - q for p, q in zip(point, camera['location'])]
        return [sum(p*q for p, q in zip(delta, camera[key])) for key in ('forward', 'right', 'up')]
    a, b = local(first), local(second)
    near = .05
    if a[0] < near and b[0] < near:
        return None
    if a[0] < near or b[0] < near:
        behind, front = (a, b) if a[0] < near else (b, a)
        fraction = (near-behind[0])/(front[0]-behind[0])
        behind[:] = [x+(y-x)*fraction for x, y in zip(behind, front)]
    focal = width/(2*math.tan(math.radians(fov)/2))
    return [(width/2 + p[1]*focal/p[0], height/2 - p[2]*focal/p[0]) for p in (a, b)]


class X11Window:
    """Find the game's client window and follow its desktop geometry."""
    class ClassHint(ct.Structure):
        _fields_ = [('name', ct.c_void_p), ('klass', ct.c_void_p)]
    class Attributes(ct.Structure):
        _fields_ = [(n, ct.c_int) for n in ('x','y','width','height','border_width','depth')] + [
            ('visual',ct.c_void_p), ('root',ct.c_ulong), ('klass',ct.c_int),
            ('bit_gravity',ct.c_int), ('win_gravity',ct.c_int), ('backing_store',ct.c_int),
            ('backing_planes',ct.c_ulong), ('backing_pixel',ct.c_ulong),
            ('save_under',ct.c_int), ('colormap',ct.c_ulong), ('map_installed',ct.c_int),
            ('map_state',ct.c_int), ('all_event_masks',ct.c_long), ('your_event_mask',ct.c_long),
            ('do_not_propagate_mask',ct.c_long), ('override_redirect',ct.c_int), ('screen',ct.c_void_p)]

    def __init__(self):
        self.x = ct.CDLL('libX11.so.6')
        self.x.XOpenDisplay.argtypes = [ct.c_char_p]
        self.x.XOpenDisplay.restype = ct.c_void_p
        self.x.XDefaultRootWindow.argtypes = [ct.c_void_p]
        self.x.XDefaultRootWindow.restype = ct.c_ulong
        self.x.XGetClassHint.argtypes = [ct.c_void_p,ct.c_ulong,ct.POINTER(self.ClassHint)]
        self.x.XGetWindowAttributes.argtypes = [ct.c_void_p,ct.c_ulong,ct.POINTER(self.Attributes)]
        self.x.XQueryTree.argtypes = [ct.c_void_p,ct.c_ulong,ct.POINTER(ct.c_ulong),ct.POINTER(ct.c_ulong),
                                    ct.POINTER(ct.POINTER(ct.c_ulong)),ct.POINTER(ct.c_uint)]
        self.x.XTranslateCoordinates.argtypes = [ct.c_void_p,ct.c_ulong,ct.c_ulong,ct.c_int,ct.c_int,
                                                ct.POINTER(ct.c_int),ct.POINTER(ct.c_int),ct.POINTER(ct.c_ulong)]
        self.x.XFree.argtypes = [ct.c_void_p]
        self.x.XSetErrorHandler.argtypes = [ct.c_void_p]
        # A window can disappear between queries. Ignore BadWindow rather than
        # letting Xlib terminate the overlay or interfere with CARLA.
        self.error_handler = ct.CFUNCTYPE(ct.c_int,ct.c_void_p,ct.c_void_p)(lambda *_: 0)
        self.x.XSetErrorHandler(ct.cast(self.error_handler,ct.c_void_p))
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('CARLA communication overlay requires an X11 display')
        self.root = self.x.XDefaultRootWindow(self.display)

    def find(self, parent=None, depth=0):
        if depth>4:
            return None
        parent = self.root if parent is None else parent
        hint = self.ClassHint()
        if self.x.XGetClassHint(self.display,parent,ct.byref(hint)):
            klass = ct.string_at(hint.klass).decode(errors='replace') if hint.klass else ''
            if hint.name:
                self.x.XFree(hint.name)
            if hint.klass:
                self.x.XFree(hint.klass)
            if klass.startswith(('CarlaUE4','CarlaUnreal')):
                return parent
        root, owner, count = ct.c_ulong(), ct.c_ulong(), ct.c_uint()
        children = ct.POINTER(ct.c_ulong)()
        if self.x.XQueryTree(self.display,parent,ct.byref(root),ct.byref(owner),ct.byref(children),ct.byref(count)):
            try:
                for i in range(count.value):
                    found = self.find(children[i],depth+1)
                    if found:
                        return found
            finally:
                if children:
                    self.x.XFree(children)
        return None

    def geometry(self, window):
        attrs = self.Attributes()
        if not self.x.XGetWindowAttributes(self.display,window,ct.byref(attrs)) or attrs.map_state!=2:
            return None
        x, y, child = ct.c_int(), ct.c_int(), ct.c_ulong()
        if not self.x.XTranslateCoordinates(self.display,window,self.root,0,0,ct.byref(x),ct.byref(y),ct.byref(child)):
            return None
        return x.value,y.value,attrs.width,attrs.height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    args = parser.parse_args()
    # The project CARLA venv and Ubuntu's Qt bindings use the same Python ABI.
    # Reuse installed PyQt5 rather than installing a second GUI dependency.
    try:
        import PyQt5
    except ModuleNotFoundError:
        sys.path.append('/usr/lib/python3/dist-packages')
    from PyQt5.QtCore import Qt, QTimer, QPointF
    from PyQt5.QtGui import QColor, QPainter, QPen, QWindow
    from PyQt5.QtWidgets import QApplication, QWidget
    import carla
    QApplication.setAttribute(Qt.AA_DisableHighDpiScaling)
    app = QApplication(['vanet-communication-overlay'])
    app.setQuitOnLastWindowClosed(False)
    native = X11Window()
    client = carla.Client(args.host,args.port)
    client.set_timeout(.5)
    spectator = client.get_world().get_spectator()

    class Overlay(QWidget):
        def __init__(self):
            super().__init__(None,Qt.Tool | Qt.FramelessWindowHint |
                             Qt.WindowTransparentForInput | Qt.WindowDoesNotAcceptFocus)
            self.setAttribute(Qt.WA_TranslucentBackground)
            self.setAttribute(Qt.WA_ShowWithoutActivating)
            self.setAttribute(Qt.WA_TransparentForMouseEvents)
            self.setWindowTitle('VANET communication overlay')
            self.winId()  # Create the native window before setting transient ownership.
            self.target, self.foreign = None, None
            self.segments, self.camera = [], None
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(33)

        def refresh(self):
            try:
                state = json.loads(args.state.read_text())
                if not alive(state.get('owner')):
                    app.quit()
                    return
                if self.target is None:
                    self.target = native.find()
                    if self.target:
                        self.foreign = QWindow.fromWinId(self.target)
                        self.windowHandle().setTransientParent(self.foreign)
                geometry = native.geometry(self.target) if self.target else None
                self.segments = list(colored_segments(state['links'],state['nodes']))
                if geometry is None or not self.segments:
                    self.hide()
                    if geometry is None:
                        self.target = None
                    return
                transform = spectator.get_transform()
                def vector(v):
                    return [v.x,v.y,v.z]
                self.camera = dict(location=vector(transform.location),
                                   forward=vector(transform.get_forward_vector()),
                                   right=vector(transform.get_right_vector()),
                                   up=vector(transform.get_up_vector()))
                self.setGeometry(*geometry)
                if not self.isVisible():
                    self.show()
                self.update()
            except FileNotFoundError:
                app.quit()
            except (OSError,ValueError,RuntimeError):
                self.hide()

        def paintEvent(self, event):
            if self.camera is None:
                return
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setClipRect(self.rect())
            for first, second, color in self.segments:
                points = project_segment(first,second,self.camera,self.width(),self.height())
                if points:
                    painter.setPen(QPen(QColor(*color),1.5,Qt.SolidLine,Qt.RoundCap))
                    painter.drawLine(QPointF(*points[0]),QPointF(*points[1]))
    overlay = Overlay()
    app.exec_()


if __name__ == '__main__':
    main()
