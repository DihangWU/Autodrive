"""Deterministic experiment slots, validated using the loaded CARLA map."""
import json
import math
import fcntl
from vanet.common.paths import CONFIG


def experiment_points(world, slot=None):
    import carla
    cfg = json.loads(CONFIG.read_text())
    town = world.get_map()
    if town.name.rsplit('/', 1)[-1] != cfg['map']:
        raise ValueError('VANET slots require Town04; load it before starting the bridge.')
    occupied = [a.get_location() for a in world.get_actors().filter('vehicle.*')]
    selected = [s for s in cfg['slots'] if slot is None or s['name'] == slot]
    points = []
    for s in selected:
        seed = carla.Location(x=s['x'], y=s['y'], z=0)
        wp = town.get_waypoint(seed, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or math.hypot(wp.transform.location.x-seed.x, wp.transform.location.y-seed.y) > 4:
            raise ValueError('Candidate slot ' + s['name'] + ' does not match this Town04 road geometry.')
        delta = (wp.transform.rotation.yaw - s['yaw'] + 180) % 360 - 180
        if abs(delta) > 35 or wp.is_junction:
            raise ValueError('Candidate slot ' + s['name'] + ' has wrong lane direction or lies inside a junction.')
        t = wp.transform
        t.location.z += .5
        if all(t.location.distance(v) > 7 for v in occupied):
            points.append(t)
    if not points:
        raise RuntimeError('VANET slots are occupied. Choose --vanet-slot A/B or move the existing vehicles.')
    return points


def reserve_slot(world, host, port, slot=None):
    """Reserve before exec; PID/start time remains valid in the root launcher.

    This covers the interval before the vehicle exists in CARLA, when occupancy
    checks alone cannot prevent simultaneous launches selecting the same slot.
    """
    from shared_sumo import control_folder, identity, alive, atomic_json
    folder = control_folder(host, port)
    state_path = folder / 'vanet-slots.json'
    cfg = json.loads(CONFIG.read_text())
    with (folder / 'vanet-slots.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        state = json.loads(state_path.read_text()) if state_path.exists() else {}
        reservations = state.get('slots', {}) if state.get('world') == world.id else {}
        reservations = {name: owner for name, owner in reservations.items() if owner and alive(owner)}
        for candidate in cfg['slots']:
            name = candidate['name']
            if (slot is not None and name != slot) or name in reservations:
                continue
            try:
                transform = experiment_points(world, name)[0]
            except RuntimeError:  # Physically occupied by another vehicle.
                continue
            owner = identity()
            if owner is None:
                raise RuntimeError('Cannot determine launcher process identity.')
            reservations[name] = owner
            atomic_json(state_path, {'world': world.id, 'slots': reservations})
            return name, transform
        atomic_json(state_path, {'world': world.id, 'slots': reservations})
        raise RuntimeError('VANET slots are occupied or reserved by running clients; close a client before reusing its slot.')
