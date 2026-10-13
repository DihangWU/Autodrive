"""Opt-in live sensor check; requires an already advancing CARLA world.

Creates a temporary mast and sensors, then removes only its own actors.
Does not tick, reload the map, move vehicles, or change simulation settings.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from cobridge.paths import RSU_DIR
sys.path.insert(0, str(RSU_DIR))
import argparse
import json
import queue

from vanet.common.config import load_config
from rsu_body import RsuBody


def main():
    import carla
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    args = parser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(8)
    world = client.get_world()
    cfg = load_config()
    assert world.get_map().name.rsplit('/', 1)[-1] == cfg['map']
    assert cfg['rsus'], 'Configure at least one RSU'
    assert not any(a.attributes.get('role_name', '').startswith('vanet_rsu')
                   for a in world.get_actors()), 'Close OMNeT++ before this temporary sensor test'
    world.wait_for_tick(5)  # Observe the existing clock before creating anything.
    rsu = cfg['rsus'][0]
    base_z = rsu['base_z']
    body, sensors, queues = RsuBody(), [], {}
    report = {}
    try:
        # Aim horizontally at the lower mast from 10m away. This check assumes
        # the default Town04 rsu0 location; other locations may be obstructed.
        pose = carla.Transform(carla.Location(x=rsu['x'] + 10, y=rsu['y'], z=base_z + 2),
                               carla.Rotation(yaw=180))
        specs = {
            'other.radar': dict(horizontal_fov='2', vertical_fov='2', points_per_second='5000', range='20'),
            'lidar.ray_cast_semantic': dict(channels='8', upper_fov='2', lower_fov='-2',
                                           points_per_second='40000', rotation_frequency='20', range='20'),
        }
        for kind, attrs in specs.items():
            bp = world.get_blueprint_library().find('sensor.' + kind)
            for key, val in attrs.items():
                bp.set_attribute(key, val)
            sensor = world.spawn_actor(bp, pose)
            sensors.append(sensor)
            samples = queues[kind] = queue.Queue()
            sensor.listen(samples.put)

        def frames(kind, count=5):
            return [queues[kind].get(timeout=10) for _ in range(count)]

        baseline = frames('other.radar')
        report['radar_before_min_m'] = min((d.depth for f in baseline for d in f), default=None)
        body.spawn(world, rsu, base_z)
        ids = {a.id for a in body.actors}
        first_frame = world.get_snapshot().frame
        lidar, radar = [], []
        for kind, dest in [('lidar.ray_cast_semantic', lidar), ('other.radar', radar)]:
            while len(dest) < 5:
                sample = queues[kind].get(timeout=10)
                if sample.frame > first_frame:
                    dest.append(sample)
        hits = [d for f in lidar for d in f if d.object_idx in ids]
        radar_hits = [d.depth for f in radar for d in f if 9.3 < d.depth < 9.5]
        report.update(actors=len(ids), lidar_hits=len(hits), radar_hits=len(radar_hits),
                      radar_min_m=min(radar_hits, default=None))
        assert hits and radar_hits, report
        body.destroy()
        world.wait_for_tick(5)  # Actor lists update on the next server snapshot.
        report['actors_remaining_after_cleanup'] = len([a for a in world.get_actors() if a.id in ids])
        assert report['actors_remaining_after_cleanup'] == 0, report
        print(json.dumps(report, indent=2))
    finally:
        try:
            for sensor in sensors:
                try:
                    sensor.stop()
                finally:
                    sensor.destroy()
        finally:
            body.destroy()


if __name__ == '__main__':
    main()
