"""Run only on the existing bridge thread: no extra TraCI client or CARLA tick."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import json
import logging

from cobridge.common.paths import RSU_SUMO_DIR
from cobridge.common.ipc import alive, atomic_json, control_folder
from vanet.common.config import sumo_rsu
from scenes.rsu.carla.rsu_marker import draw_marker
from scenes.rsu.carla.rsu_body import RsuBody


class SceneMarkers:
    def __init__(self, world, host, port):
        self.world = world
        folder = control_folder(host, port)
        self.lease = folder / 'vanet-rsu.json'
        self.reply = folder / 'vanet-rsu-status.json'
        self.session = None
        self.config = None
        self.last_frame = None
        self.error = None
        self.markers = {}
        self.bodies = {}

    def status(self):
        return dict(protocol=5, session=self.session, error=self.error, rsus=list(self.markers))

    def network_nodes(self):
        if self.config is None:
            return []
        return [(name, *sumo_rsu(self.config, p)) for name, (p, _) in self.markers.items()]

    def clear(self):
        import traci
        had_scene = bool(self.session or self.markers or self.bodies)
        try:
            existing = set(traci.poi.getIDList()) if self.markers else set()
            for name in list(self.markers):
                if name in existing:
                    traci.poi.remove(name, layer=100)
                del self.markers[name]
        finally:
            # A broken TraCI connection must not strand CARLA collision actors.
            for name, body in list(self.bodies.items()):
                try:
                    body.destroy()
                    del self.bodies[name]
                except Exception as error:
                    logging.warning('%s', error)  # Keep ownership for a retry.
            self.session = self.config = None
            if had_scene:
                atomic_json(self.reply, dict(session=None, ok=not self.bodies, rsus=[]))
        # CARLA lines have a two-tick lifetime, and expire as the bridge resumes.

    def poll(self):
        import carla
        import traci
        try:
            lease = json.loads(self.lease.read_text())
        except FileNotFoundError:
            lease = None
        if not lease or not lease.get('owner') or not alive(lease['owner']):
            self.clear()
            return
        session = lease['session']
        if self.session != session:
            self.clear()
            try:
                cfg = lease['config']  # Immutable snapshot owned by run_rsu.sh.
                if self.bodies:
                    raise RuntimeError('Previous RSU actors could not be cleaned up')
                town = self.world.get_map()
                if town.name.rsplit('/', 1)[-1] != cfg['map']:
                    raise ValueError('RSU scene markers require ' + cfg['map'])
                positions = {}
                for p in cfg['rsus']:
                    name = p['id']
                    x, y, _ = sumo_rsu(cfg, p)
                    if name in traci.poi.getIDList():
                        raise ValueError('SUMO POI already exists: ' + name)
                    waypoint = town.get_waypoint(carla.Location(x=p['x'], y=p['y']), project_to_road=True)
                    base_z = p.get('base_z')
                    if base_z is None:
                        if waypoint is None:
                            raise ValueError(name + ': specify base_z; no nearby road found')
                        base_z = waypoint.transform.location.z + .1
                    body = self.bodies[name] = RsuBody()
                    body.spawn(self.world, p, base_z)
                    traci.poi.add(name, x, y, (255, 130, 0, 255),
                                  poiType='RSU', layer=100, width=8, height=8,
                                  imgFile=str(RSU_SUMO_DIR / 'rsu.png'))
                    self.markers[name] = (p, base_z)
                    traci.poi.setParameter(name, 'name', name)
                    positions[name] = [x, y]
                self.config, self.session = cfg, session
                self.last_frame, self.error = None, None
                atomic_json(self.reply, dict(session=session, ok=True, sumo_positions=positions))
            except Exception as error:
                self.clear()
                self.error = str(error)
                atomic_json(self.reply, dict(session=session, ok=False, error=self.error))
                logging.warning('RSU scene marker: %s', error)
                return
        snap = self.world.get_snapshot()
        if snap.frame != self.last_frame:
            step = snap.timestamp.delta_seconds or .05
            for name, (p, base_z) in self.markers.items():
                draw_marker(self.world, p, base_z, lifetime=2 * step, label=name)
            self.last_frame = snap.frame
