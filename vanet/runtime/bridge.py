"""Optional loopback lockstep endpoint owned by the existing SUMO bridge.

Wire protocol: FRAME2 <frame> <step> <vehicle-count> <rsu-count>, then
ID NAME X Y Z per Lincoln and NAME X Y Z per live RSU.
Legacy FRAME is retained for synthetic tests with static RSU definitions.
ACK <frame> releases the next CARLA/SUMO step. No second TraCI connection.
"""
import json
import logging
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
            # recv directly rather than a timed-out buffered makefile.
            data = bytearray()
            while b'\n' not in data:
                try:
                    chunk = self.peer.recv(128)
                except socket.timeout:
                    if self.idle_callback:
                        self.idle_callback()
                    continue
                if not chunk:
                    raise ConnectionError('network simulator disconnected')
                data.extend(chunk)
                if len(data)>128:
                    raise ConnectionError('oversized ACK')
            if data.decode('ascii').strip() != f'ACK {snap.frame}':
                raise ConnectionError('unexpected frame ACK')
        except (OSError, ConnectionError, UnicodeError) as e:
            logging.warning('VANET detached: %s; SUMO/CARLA continue', e)
            self.detach()

    def detach(self):
        if self.peer:
            self.peer.close()
        self.peer = None

    def close(self):
        self.detach()
        self.listener.close()
