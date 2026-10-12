"""Optional loopback lockstep endpoint owned by the existing SUMO bridge.

Wire protocol: FRAME2 <frame> <step> <vehicle-count> <rsu-count>, then
ID NAME X Y Z per Lincoln and NAME X Y Z per live RSU.
Legacy FRAME is retained for synthetic tests with static RSU definitions.
ACK <frame> releases the next CARLA/SUMO step. No second TraCI connection.
"""
import json
import logging
import math
import socket
from vanet.common.paths import CONFIG

class VanetBridge:
    def __init__(self, port):
        self.cfg = json.loads(CONFIG.read_text())
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('127.0.0.1', int(port)))
        self.listener.listen(1)
        self.listener.setblocking(False)
        self.peer = None
        self.names = {}
        self.idle_callback = None
        self.rsu_provider = None
        self.visualization_callback = None
        self.last_visualization = None
        self.receive_buffer = bytearray()
        logging.info('VANET listening on 127.0.0.1:%s', port)

    def tick(self, world, step, sumo_ids=None):
        if self.peer is None:
            try:
                self.peer, _ = self.listener.accept()
                # Qtenv can pause; keep bounded waits so Ctrl+C remains responsive.
                self.peer.settimeout(1)
                logging.info('VANET connected; network ACK now gates each simulation step')
            except BlockingIOError:
                return
        snap = world.get_snapshot()
        vehicles = []
        for a in world.get_actors().filter('vehicle.*'):
            if not a.attributes.get('role_name', '').startswith('lincoln_'):
                continue
            state = snap.find(a.id)
            if state is None:
                continue
            p = state.get_transform().location
            name = (sumo_ids or {}).get(a.id)
            if name is None:
                name = self.names.setdefault(a.id, 'lincoln' + str(len(self.names)))
            vehicles.append((a.id, name, p.x+self.cfg['network']['offset_x'],
                             -p.y+self.cfg['network']['offset_y'], p.z+1.8))
        rsus = self.rsu_provider() if self.rsu_provider is not None else None
        payload = (f'FRAME {snap.frame} {step:.9f} {len(vehicles)}\n' if rsus is None else
                   f'FRAME2 {snap.frame} {step:.9f} {len(vehicles)} {len(rsus)}\n')
        payload += ''.join(f'{i} {name} {x:.9f} {y:.9f} {z:.9f}\n' for i,name,x,y,z in vehicles)
        if rsus is not None:
            payload += ''.join(f'{name} {x:.9f} {y:.9f} {z:.9f}\n' for name,x,y,z in rsus)
        try:
            self.peer.sendall(payload.encode('ascii'))
            state = self.receive_feedback(snap.frame)
            if state is not None:
                ox, oy = self.cfg['network']['offset_x'], self.cfg['network']['offset_y']
                state['nodes'] = {name: [x-ox, oy-y, z] for _,name,x,y,z in vehicles}
                state['nodes'].update({name: [x-ox, oy-y, z] for name,x,y,z in (rsus or [])})
                self.last_visualization = state
                if self.visualization_callback:
                    self.visualization_callback(state)
        except (OSError, ConnectionError, UnicodeError, ValueError) as e:
            logging.warning('VANET detached: %s; SUMO/CARLA continue', e)
            self.detach()

    def receive_line(self):
        # Keep leftovers: a VIS block and its ACK can span arbitrary TCP reads.
        # Never use a makefile whose buffer becomes unusable after a timeout.
        while b'\n' not in self.receive_buffer:
            try:
                chunk = self.peer.recv(4096)
            except socket.timeout:
                if self.idle_callback:
                    self.idle_callback()
                continue
            if not chunk:
                raise ConnectionError('network simulator disconnected')
            self.receive_buffer.extend(chunk)
            if len(self.receive_buffer) > 8192:
                raise ConnectionError('oversized VANET response line')
        line, _, rest = self.receive_buffer.partition(b'\n')
        self.receive_buffer = bytearray(rest)
        if len(line)>512:
            raise ConnectionError('oversized VANET response line')
        return line.decode('ascii')

    def receive_feedback(self, frame):
        line = self.receive_line()
        state = None
        if line.startswith('VIS '):
            fields = line.split()
            if len(fields)!=4 or int(fields[1])!=frame:
                raise ConnectionError('invalid visualization frame')
            timestamp, count = float(fields[2]), int(fields[3])
            if not math.isfinite(timestamp) or timestamp<0 or not 0<=count<=50000:
                raise ConnectionError('invalid visualization size/time')
            links = []
            for _ in range(count):
                row = self.receive_line().split()
                if len(row)!=3 or int(row[2]) not in (0,1,2,3) or row[0]==row[1]:
                    raise ConnectionError('invalid visualization link')
                links.append([row[0], row[1], int(row[2])])
            state = dict(frame=frame, time=timestamp, links=links)
            line = self.receive_line()
        if line != f'ACK {frame}':
            raise ConnectionError('unexpected frame ACK')
        return state

    def detach(self):
        if self.peer:
            self.peer.close()
        self.peer = None
        self.receive_buffer.clear()
        self.last_visualization = None
        if self.visualization_callback:
            self.visualization_callback(None)

    def close(self):
        self.detach()
        self.listener.close()
