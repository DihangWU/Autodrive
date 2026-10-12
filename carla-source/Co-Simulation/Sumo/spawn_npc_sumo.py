#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""Spawn Sumo NPCs vehicles into the simulation"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import json
import logging
import os
import random
import re
import shutil
import sys
import tempfile
from pathlib import Path
import uuid
import time

import lxml.etree as ET  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' not in os.environ and (Path.home() / 'sumo-1.22.0').is_dir():
    os.environ['SUMO_HOME'] = str(Path.home() / 'sumo-1.22.0')
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import sumolib  # pylint: disable=wrong-import-position
import traci  # pylint: disable=wrong-import-position

from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

from run_synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position

from util.netconvert_carla import netconvert_carla

# Project-local launcher/control protocol; no ROS dependency here.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'lincoln'))
from shared_sumo import RequestService, control_folder, atomic_json, request, identity, alive
from ensure_sumo import ensure_bridge
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from vanet.scene import SceneMarkers


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
    if abs(ready['step_length'] - args.step_length) > 1e-8 or ready['tls_manager'] != args.tls_manager:
        logging.warning('Using existing bridge settings: step=%s, tls=%s; not changing the running simulation.',
                        ready['step_length'], ready['tls_manager'])
    if args.number_of_walkers:
        logging.warning('Pedestrians are not supported. No walkers will be spawned.')
    session = uuid.uuid4().hex
    added = False
    try:
        result = request(args.host, args.port, dict(action='add', session=session,
                         count=args.number_of_vehicles, filterv=args.filterv, safe=args.safe))
        added = True
        print(f'NPC_SESSION {session}: scheduled {len(result["vehicles"])} vehicles in shared SUMO. '
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


# ==================================================================================================
# -- main ------------------------------------------------------------------------------------------
# ==================================================================================================


def write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, additional_traci_clients=0):
    """
    Writes sumo configuration xml file.
    """
    root = ET.Element('configuration')

    input_tag = ET.SubElement(root, 'input')
    ET.SubElement(input_tag, 'net-file', {'value': net_file})
    ET.SubElement(input_tag, 'route-files', {'value': vtypes_file})

    gui_tag = ET.SubElement(root, 'gui_only')
    ET.SubElement(gui_tag, 'gui-settings-file', {'value': viewsettings_file})

    ET.SubElement(root, 'num-clients', {'value': str(additional_traci_clients+1)})

    tree = ET.ElementTree(root)
    tree.write(cfg_file, pretty_print=True, encoding='UTF-8', xml_declaration=True)


def main(args):

    # Temporal folder to save intermediate files.
    tmpdir = tempfile.mkdtemp()

    # ----------------
    # carla simulation
    # ----------------
    carla_simulation = CarlaSimulation(args.host, args.port, args.step_length)

    world = carla_simulation.client.get_world()
    current_map = world.get_map()

    xodr_file = os.path.join(tmpdir, current_map.name + '.xodr')
    current_map.save_to_disk(xodr_file)

    # ---------------
    # sumo simulation
    # ---------------
    net_file = os.path.join(tmpdir, current_map.name + '.net.xml')
    netconvert_carla(xodr_file, net_file, guess_tls=True)

    basedir = os.path.dirname(os.path.realpath(__file__))
    cfg_file = os.path.join(tmpdir, current_map.name + '.sumocfg')
    vtypes_file = os.path.join(basedir, 'examples', 'carlavtypes.rou.xml')
    viewsettings_file = os.path.join(basedir, 'examples', 'viewsettings.xml')
    write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, args.additional_traci_clients)

    sumo_net = sumolib.net.readNet(net_file)
    if args.sumo_gui:
        from vanet.sumo_gui.build import ensure_gui
        os.environ['GUISIM_BINARY'] = str(ensure_gui())
    sumo_simulation = SumoSimulation(cfg_file,
                                     args.step_length,
                                     host=args.sumo_host,
                                     port=args.sumo_port,
                                     sumo_gui=args.sumo_gui,
                                     client_order=args.client_order,
                                     auto_start=args.sumo_auto_start)

    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)
    scene = SceneMarkers(world, args.host, args.port)
    if synchronization.vanet:
        synchronization.vanet.idle_callback = scene.poll

    try:
        # ----------
        # Blueprints
        # ----------
        with open(os.path.join(basedir, 'data', 'vtypes.json')) as f:
            vtypes = json.load(f)['carla_blueprints']
        sumo_edges = sumo_net.getEdges()
        allowed_by_class = {}

        def remove_batch(vehicles):
            for vehicle_id in vehicles:
                carla_id = synchronization.sumo2carla_ids.pop(vehicle_id, None)
                try:
                    sumo_simulation.unsubscribe(vehicle_id)
                except traci.TraCIException:
                    pass
                try:
                    traci.vehicle.remove(vehicle_id)
                except traci.TraCIException:
                    pass  # Includes vehicles that already left the network.
                if carla_id is not None:
                    carla_simulation.destroy_actor(carla_id)

        def spawn_batch(session, command):
            count = int(command['count'])
            if count < 0:
                raise ValueError('number-of-vehicles must be nonnegative')
            blueprints = selected_blueprints(vtypes, command['filterv'], command['safe'])
            vehicles = []
            try:
                for i in range(count):
                    type_id = random.choice(blueprints)
                    vclass = vtypes[type_id]['vClass']
                    if vclass not in allowed_by_class:
                        allowed_by_class[vclass] = [edge for edge in sumo_edges if edge.allows(vclass)]
                    edges = allowed_by_class[vclass]
                    if not edges:
                        continue
                    vehicle_id = f'npc_{session}_{i}'
                    route_id = f'route_{session}_{i}'
                    traci.route.add(route_id, [random.choice(edges).getID()])
                    traci.vehicle.add(vehicle_id, route_id, typeID=type_id)
                    vehicles.append(vehicle_id)
            except Exception:
                remove_batch(vehicles)
                raise
            return vehicles

        def status():
            return dict(sumo_vehicles=list(traci.vehicle.getIDList()),
                        pending_vehicles=list(traci.simulation.getPendingVehicles()),
                        sumo2carla=dict(synchronization.sumo2carla_ids),
                        carla2sumo=dict(synchronization.carla2sumo_ids),
                        sessions=list(service.sessions),
                        vanet_scene=scene.status())

        service = RequestService(args.host, args.port, spawn_batch, remove_batch, status)
        spawn_batch('bridge_' + uuid.uuid4().hex,
                    dict(count=args.number_of_vehicles, filterv=args.filterv, safe=args.safe))
        ready_file = control_folder(args.host, args.port) / 'ready.json'
        if args.bridge_server:
            atomic_json(ready_file, dict(pid=os.getpid(), owner=identity(), step_length=args.step_length,
                                        tls_manager=args.tls_manager, map=current_map.name))

        # Cache each vehicle's class from its first subscription result.
        vehicle_classes = {}
        vehicle_routes = {}

        while True:
            start = time.time()

            service.process()
            scene.poll()
            synchronization.tick()

            # Drop departed vehicles so reused IDs get fresh cached data.
            vehicle_ids = traci.vehicle.getIDList()
            for vehicle_id in sumo_simulation.destroyed_actors:
                vehicle_classes.pop(vehicle_id, None)
                vehicle_routes.pop(vehicle_id, None)
            for vehicle_id in set(vehicle_classes).difference(vehicle_ids):
                del vehicle_classes[vehicle_id]
            for vehicle_id in set(vehicle_routes).difference(vehicle_ids):
                del vehicle_routes[vehicle_id]

            # CARLA-controlled vehicles may have been moved/rerouted after the
            # SUMO step, so their subscription snapshot can already be stale.
            carla_controlled_ids = set(synchronization.carla2sumo_ids.values())

            # Updates vehicle routes
            for vehicle_id in vehicle_ids:
                results = traci.vehicle.getSubscriptionResults(vehicle_id)
                if (not results or traci.constants.VAR_EDGES not in results
                        or traci.constants.VAR_ROUTE_INDEX not in results):
                    # Also cover vehicles that were not mirrored into CARLA.
                    sumo_simulation.subscribe(vehicle_id)
                    results = traci.vehicle.getSubscriptionResults(vehicle_id)

                if vehicle_id in carla_controlled_ids:
                    route = traci.vehicle.getRoute(vehicle_id)
                    index = traci.vehicle.getRouteIndex(vehicle_id)
                else:
                    route = results[traci.constants.VAR_EDGES]
                    index = results[traci.constants.VAR_ROUTE_INDEX]
                vehicle_routes[vehicle_id] = tuple(route)
                if vehicle_id not in vehicle_classes:
                    vehicle_classes[vehicle_id] = results[traci.constants.VAR_VEHICLECLASS]
                vclass = vehicle_classes[vehicle_id]

                if route and 0 <= index == (len(route) - 1):
                    current_edge = sumo_net.getEdge(route[index])
                    available_edges = list(current_edge.getAllowedOutgoing(vclass).keys())
                    if available_edges:
                        next_edge = random.choice(available_edges)

                        new_route = [current_edge.getID(), next_edge.getID()]
                        traci.vehicle.setRoute(vehicle_id, new_route)
                        vehicle_routes[vehicle_id] = tuple(new_route)

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        ready_file = control_folder(args.host, args.port) / 'ready.json'
        if args.bridge_server and ready_file.exists() and json.loads(ready_file.read_text()).get('pid') == os.getpid():
            ready_file.unlink()
        try:
            scene.clear()
        finally:
            synchronization.close()

        if os.path.exists(tmpdir):
            shutil.rmtree(tmpdir)



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
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
    args = argparser.parse_args()

    if args.sync_vehicle_all is True:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    if args.bridge_server or args.standalone:
        main(args)
    else:
        npc_client(args)
