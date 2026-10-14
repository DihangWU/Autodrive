#!/usr/bin/env python
"""Submit and clean up NPC batches through the shared co-simulation bridge."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import logging
import re
import time
import uuid

from cobridge.common.bridge_manager import ensure_bridge
from cobridge.common.ipc import request, alive

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


def selected_blueprints(vtypes, filter_pattern, safe):
    pattern = re.compile(filter_pattern)
    result = [name for name in vtypes if pattern.search(name)]
    if safe:
        excluded = ('microlino', 'carlacola', 'cybertruck', 't2', 'sprinter', 'firetruck', 'ambulance')
        result = [name for name in result if vtypes[name]['vClass'] not in ('motorcycle', 'bicycle')
                  and not name.endswith(excluded)]
    if not result:
        raise ValueError('No blueprints available due to user restrictions.')
    return result



def npc_client(args):
    if args.additional_traci_clients or args.sumo_host or args.sumo_port:
        raise ValueError('External/multi-client TraCI mode requires --standalone.')
    ready = ensure_bridge(host=args.host, port=args.port, step_length=args.step_length,
                          sumo_gui=args.sumo_gui, tls_manager=args.tls_manager,
                          sync_vehicle_color=True, sync_vehicle_lights=args.sync_vehicle_lights)
    status = request(args.host, args.port, dict(action='status'), timeout=8)
    if status.get('vanet_scene', {}).get('protocol') != 5 or status.get('npc_vanet_opt_in') is not True:
        raise ValueError('共享桥仍是旧版本，不支持 NPC 按批次选择是否联网。请正常关闭车辆、NPC、RSU 和 OMNeT++，'
                         '执行 bash cobridge/run_cobridge.sh --stop 后重新启动。')
    if abs(ready['step_length'] - args.step_length) > 1e-8 or ready['tls_manager'] != args.tls_manager:
        logging.warning('Using existing bridge settings: step=%s, tls=%s; not changing the running simulation.',
                        ready['step_length'], ready['tls_manager'])
    if args.number_of_walkers:
        logging.warning('Pedestrians are not supported. No walkers will be spawned.')
    session = uuid.uuid4().hex
    added = False
    try:
        result = request(args.host, args.port, dict(action='add', session=session,
                         count=args.number_of_vehicles, filterv=args.filterv,
                         safe=args.safe, vanet=args.vanet))
        added = True
        print(f'NPC_SESSION {session}: scheduled {len(result["vehicles"])} vehicles in shared SUMO '
              f'(VANET {"on" if args.vanet else "off"}). '
              'Ctrl+C removes only this batch; CARLA/SUMO and Lincoln keep running.', flush=True)
        started = time.monotonic()
        while not args.duration or time.monotonic() - started < args.duration:
            if not alive(ready['owner']):
                print('Shared SUMO bridge stopped.', flush=True)
                break
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        if added and alive(ready['owner']):
            try:
                request(args.host, args.port, dict(action='remove', session=session), timeout=10)
                print(f'NPC_SESSION {session} removed; shared simulation continues.', flush=True)
            except RuntimeError as error:
                logging.warning('Batch cleanup: %s', error)



def parse_args(argv=None, *, description=__doc__):
    argparser = argparse.ArgumentParser(description=description)
    argparser.add_argument('--host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           default=None,
                           help='IP of the sumo host server (default: None)')
    argparser.add_argument('--sumo-port',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: None)')
    argparser.add_argument('-n',
                           '--number-of-vehicles',
                           metavar='N',
                           default=10,
                           type=int,
                           help='number of vehicles (default: 10)')
    argparser.add_argument('-w',
                           '--number-of-walkers',
                           metavar='W',
                           default=0,
                           type=int,
                           help='number of walkers (default: 0)')
    argparser.add_argument('--safe',
                           action='store_true',
                           help='avoid spawning vehicles prone to accidents')
    argparser.add_argument('--vanet', action='store_true',
                           help='enable VANET communication for this NPC batch (default: off)')
    argparser.add_argument('--filterv',
                           metavar='PATTERN',
                           default='vehicle.*',
                           help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument('--filterw',
                           metavar='PATTERN',
                           default='walker.pedestrian.*',
                           help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--sumo-auto-start', action='store_true',
                           help='automatically start and close the SUMO GUI')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--additional-traci-clients',
                           metavar='TRACI_CLIENTS',
                           default=0,
                           type=int,
                           help='number of additional TraCI clients to wait for (default: 0)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    argparser.add_argument('--bridge-server', action='store_true', help=argparse.SUPPRESS)
    argparser.add_argument('--standalone', action='store_true', help='Legacy mode: this process owns and closes SUMO.')
    argparser.add_argument('--duration', type=float, default=0, help='NPC batch lifetime in real seconds; 0 waits for Ctrl+C.')
    args = argparser.parse_args(argv)

    if args.sync_vehicle_all is True:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    return args


def main():
    args = parse_args()
    if args.bridge_server or args.standalone:
        from cobridge.common.service import main as run_service
        run_service(args)
    else:
        npc_client(args)


if __name__ == '__main__':
    main()
