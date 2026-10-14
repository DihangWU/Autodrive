"""Isolate fixed spawns from SUMO, sensors and spectator control."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import carla
from cobridge.common.bridge_manager import bridges
from cobridge.common.paths import LINCOLN_DIR
sys.path.insert(0, str(LINCOLN_DIR))
from spawn import experiment_points

if __name__=='__main__':
    if bridges('127.0.0.1',2000):
        raise SystemExit('Stop the existing shared bridge first; diagnostic must own ticks exclusively.')
    c=carla.Client('127.0.0.1',2000);c.set_timeout(30)
    w=c.get_world()
    if w.get_map().name.rsplit('/',1)[-1]!='Town04':
        c.set_timeout(120);w=c.load_world('Town04');c.set_timeout(30)
    settings=w.get_settings()
    if settings.synchronous_mode:
        raise SystemExit('World is already synchronous without a known bridge; restart CARLA before this isolated diagnostic.')
    original=w.get_settings(); actors=[]
    settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
    try:
        w.apply_settings(settings)
        for name in ['A','B']:
            p=experiment_points(w,name)[0]
            print('SPAWN',name,p,flush=True)
            bp=w.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
            bp.set_attribute('role_name','vanet_diagnostic_'+name)
            a=w.try_spawn_actor(bp,p)
            if a is None:raise RuntimeError('Spawn rejected for '+name)
            actors.append(a)
            a.apply_control(carla.VehicleControl(brake=1))
            for _ in range(20):w.tick()
            print('PASS',name,'actor',a.id,'frame',w.get_snapshot().frame,flush=True)
        print('PASS two fixed spawns and 40 CARLA ticks; no SUMO, sensors or spectator changes.',flush=True)
    finally:
        for a in actors:
            try:a.destroy()
            except Exception as e:print('cleanup',e,flush=True)
        w.apply_settings(original)
