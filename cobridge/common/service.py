#!/usr/bin/env python
"""Shared CARLA/SUMO/OMNeT++ co-simulation service."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from cobridge.common.paths import SUMO_CO_SIM_DIR
import os
os.environ.setdefault('SUMO_HOME', str(Path.home() / 'sumo-1.22.0'))
sys.path.insert(0, str(SUMO_CO_SIM_DIR))
sys.path.append(str(Path(os.environ['SUMO_HOME']) / 'tools'))

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import json
import logging
import random
import shutil
import tempfile
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
from cobridge.sumo.traci_client import SumoSimulation  # pylint: disable=wrong-import-position

from cobridge.common.synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position

from util.netconvert_carla import netconvert_carla

# Project-local launcher/control protocol; no ROS dependency here.

from cobridge.common.ipc import RequestService, control_folder, atomic_json, identity

from scene.rsu.manager import SceneMarkers
from cobridge.carla.communication_view import CommunicationView
from cobridge.sumo.npc_client import selected_blueprints, parse_args






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

    basedir = str(SUMO_CO_SIM_DIR)
    cfg_file = os.path.join(tmpdir, current_map.name + '.sumocfg')
    vtypes_file = os.path.join(basedir, 'examples', 'carlavtypes.rou.xml')
    viewsettings_file = os.path.join(basedir, 'examples', 'viewsettings.xml')
    write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, args.additional_traci_clients)

    sumo_net = sumolib.net.readNet(net_file)
    if args.sumo_gui:
        from sumo.gui.build import ensure_gui
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
    communication = CommunicationView(args.host, args.port)
    if synchronization.vanet:
        def poll_idle():
            service.process()
            scene.poll()
        synchronization.vanet.idle_callback = poll_idle
        synchronization.vanet.rsu_provider = scene.network_nodes
        synchronization.vanet.visualization_callback = communication.update

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
                synchronization.vanet_sumo_ids.discard(vehicle_id)
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
            vanet_enabled = command.get('vanet', False)
            if not isinstance(vanet_enabled, bool):
                raise ValueError('vanet must be a boolean')
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
            if vanet_enabled:
                synchronization.vanet_sumo_ids.update(vehicles)
            return vehicles

        def status():
            return dict(sumo_vehicles=list(traci.vehicle.getIDList()),
                        pending_vehicles=list(traci.simulation.getPendingVehicles()),
                        sumo2carla=dict(synchronization.sumo2carla_ids),
                        carla2sumo=dict(synchronization.carla2sumo_ids),
                        sessions=list(service.sessions),
                        npc_vanet_opt_in=True,
                        vanet_port=synchronization.vanet.listener.getsockname()[1],
                        vanet_scene=scene.status())

        service = RequestService(args.host, args.port, spawn_batch, remove_batch, status)
        spawn_batch('bridge_' + uuid.uuid4().hex,
                    dict(count=args.number_of_vehicles, filterv=args.filterv,
                         safe=args.safe, vanet=args.vanet))
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
            communication.close()
            scene.clear()
        finally:
            synchronization.close()

        if os.path.exists(tmpdir):
            shutil.rmtree(tmpdir)



if __name__ == '__main__':
    main(parse_args(description=__doc__))
