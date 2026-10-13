"""Exercise the actual synchronization adapter with fake worlds and real TCP.

No connection to live CARLA or TraCI, and no additional simulation clock.
Run with ~/carla-venv/bin/python.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import copy
import os
import socket
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('SUMO_HOME', str(Path.home() / 'sumo-1.22.0'))
sys.path.insert(0, str(ROOT / 'cobridge/carla-source-official/Co-Simulation/Sumo'))
import carla
from cobridge.bridge.run_synchronization import SimulationSynchronization, BridgeHelper
from vanet.common.config import load_config


def main():
    actors = {}
    def actor(identifier, role):
        obj = NS(id=identifier, attributes={'role_name': role}, is_alive=True,
                 bounding_box=NS(extent=carla.Vector3D(2,1,1)),
                 get_transform=lambda: carla.Transform(carla.Location(199+identifier*5,-204,.5)))
        actors[identifier] = obj
        return obj
    actor(1,'lincoln_test')
    actor(2,'sumo_driver')
    actor(3,'sumo_driver')  # Not inserted/mapped yet: must not get a temporary OBU.
    world = Mock()
    world.get_settings.return_value = NS(synchronous_mode=False, fixed_delta_seconds=None)
    world.get_actors.side_effect = lambda: NS(filter=lambda _: list(actors.values()))
    frame = 0
    def tick():
        nonlocal frame
        frame += 1
    world.get_snapshot.side_effect = lambda: NS(frame=frame, find=lambda identifier: actors.get(identifier))
    simulator = Mock(world=world, step_length=.05, spawned_actors=set(), destroyed_actors=set())
    simulator.tick.side_effect = tick
    simulator.get_actor.side_effect = actors.get
    simulator.destroy_actor.side_effect = lambda identifier: actors.pop(identifier)
    simulator.spawn_actor.side_effect = lambda *_: actor(4,'sumo_driver').id
    sumo = Mock(spawned_actors=set(), destroyed_actors=set())
    sumo.get_net_offset.return_value = (503.02,423.76)
    sumo.get_actor.return_value = NS(transform=carla.Transform(),extent=carla.Vector3D(2,1,1))
    cfg = copy.deepcopy(load_config())
    cfg['network']['port'] = 0  # An isolated listener, never the live endpoint.
    with patch.dict(os.environ, VANET_PORT=''), patch('vanet.common.config.load_config', return_value=cfg):
        sync = SimulationSynchronization(sumo, simulator)
    assert sync.vanet is not None, 'NPC-first startup did not create a VANET endpoint'
    sync.carla2sumo_ids = {1:'lincoln0'}
    sync.sumo2carla_ids = {'npc_batch_0':2}
    sync.vanet.rsu_provider = lambda: []
    visuals = []
    sync.vanet.visualization_callback = visuals.append
    peer = socket.create_connection(sync.vanet.listener.getsockname(), timeout=5)
    errors = []
    snapshots = []
    def network():
        try:
            with peer.makefile('r') as reader:
                for expected in (1,2,3,4,5):
                    header = reader.readline().split()
                    assert header[:2] == ['FRAME2',str(expected)], header
                    names = [reader.readline().split()[1] for _ in range(int(header[3]))]
                    assert int(header[4]) == 0
                    snapshots.append(names)
                    link = f'{names[0]} {names[1]} 3\n' if len(names)==2 else ''
                    peer.sendall(f'VIS {expected} {expected*.05} {int(bool(link))}\n{link}ACK {expected}\n'.encode())
        except Exception as error:
            errors.append(error)
    worker = threading.Thread(target=network, daemon=True)
    worker.start()
    try:
        with patch.object(BridgeHelper,'get_carla_transform',return_value=carla.Transform()), \
             patch.object(BridgeHelper,'get_sumo_transform',return_value=carla.Transform()), \
             patch.object(BridgeHelper,'get_carla_blueprint',return_value=object()):
            sync.tick()
            sync.vanet_sumo_ids.add('npc_batch_0')
            sync.tick()
            sumo.destroyed_actors = {'npc_batch_0'}
            sync.tick()
            sumo.destroyed_actors = set()
            sumo.spawned_actors = {'npc_batch_1'}
            sync.tick()
            sync.vanet_sumo_ids.add('npc_batch_1')
            sync.tick()
        worker.join(timeout=5)
        assert not worker.is_alive() and not errors, errors
        assert snapshots == [['lincoln0'], ['lincoln0','npc_batch_0'], ['lincoln0'],
                             ['lincoln0'], ['lincoln0','npc_batch_1']], snapshots
        assert [set(s['nodes']) for s in visuals] == [set(names) for names in snapshots]
        assert visuals[1]['links'] == [['lincoln0','npc_batch_0',3]]
        assert sync.vanet_sumo_ids == {'npc_batch_1'}
        assert simulator.tick.call_count == sumo.tick.call_count == 5
        assert sync.sumo2carla_ids == {'npc_batch_1':4}
        print('PASS: NPC default off, batch opt-in, spawn/despawn cleanup and shared animation feedback; '
              'only the existing clock advances.')
    finally:
        peer.close()
        sync.vanet.close()


if __name__ == '__main__':
    main()
