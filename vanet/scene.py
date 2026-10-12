"""Run only on the existing bridge thread: no extra TraCI client or CARLA tick."""
import json
import logging
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lincoln'))
from shared_sumo import alive, atomic_json, control_folder
from vanet.config import sumo_rsu
from vanet.rsu_marker import draw_marker


class SceneMarkers:
    POI = 'vanet_rsu'

    def __init__(self, world, host, port):
        self.world = world
        folder = control_folder(host, port)
        self.lease = folder / 'vanet-scene.json'
        self.reply = folder / 'vanet-scene-status.json'
        self.session = None
        self.config = None
        self.last_frame = None
        self.error = None

    def status(self):
        return dict(protocol=1, session=self.session, error=self.error)

    def clear(self):
        import traci
        if self.session is not None:
            if self.POI in traci.poi.getIDList():
                traci.poi.remove(self.POI, layer=100)
            self.session = self.config = None
        # CARLA lines have a two-tick lifetime, and expire as the bridge resumes.

    def poll(self):
        import carla
        import traci
        try:
            lease = json.loads(self.lease.read_text())
        except FileNotFoundError:
            lease = None
        if not lease or not lease.get('owner') or not lease.get('child') or not alive(lease['owner']) or not alive(lease['child']):
            self.clear()
            return
        session = lease['session']
        if self.session != session:
            self.clear()
            try:
                cfg = lease['config']  # Immutable snapshot read from town04.json by run_omnet.sh.
                town = self.world.get_map()
                if town.name.rsplit('/', 1)[-1] != cfg['map']:
                    raise ValueError('RSU scene markers require ' + cfg['map'])
                x, y, _ = sumo_rsu(cfg)
                if self.POI in traci.poi.getIDList():
                    traci.poi.remove(self.POI, layer=100)
                traci.poi.add(self.POI, x, y, (255, 130, 0, 255),
                              poiType='RSU', layer=100, width=8, height=8,
                              imgFile=str(Path(__file__).with_name('assets') / 'rsu.png'))
                traci.poi.setParameter(self.POI, 'name', 'RSU ' + str(cfg['junction']))
                p = cfg['rsu']
                waypoint = town.get_waypoint(carla.Location(x=p['x'], y=p['y']), project_to_road=True)
                self.base_z = p.get('base_z', waypoint.transform.location.z + .1)
                self.config, self.session = cfg, session
                self.last_frame, self.error = None, None
                atomic_json(self.reply, dict(session=session, ok=True, sumo_position=[x, y]))
            except Exception as error:
                if self.POI in traci.poi.getIDList():
                    traci.poi.remove(self.POI, layer=100)
                self.error = str(error)
                atomic_json(self.reply, dict(session=session, ok=False, error=self.error))
                logging.warning('RSU scene marker: %s', error)
                return
        snap = self.world.get_snapshot()
        if snap.frame != self.last_frame:
            step = snap.timestamp.delta_seconds or .05
            draw_marker(self.world, self.config['rsu'], self.base_z,
                        lifetime=2 * step, label='RSU ' + str(self.config['junction']))
            self.last_frame = snap.frame
