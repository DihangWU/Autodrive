"""Lincoln 紧急任务主文件：桥命令、规划、控制和状态报告。"""
import logging
import math
import queue
import threading
import time
import carla
from .task_state import EmergencyTaskState
from vanet.lincoln.emergency_driving_control.trajectory_controller import EmergencyTrajectoryController
from vanet.lincoln.emergency_route_planning.borrowing_planner import BorrowingPlanner
from modules.lincoln.driving.navigation import plan_route, path_with_lookahead
from modules.lincoln.driving.junction_local_controller import JunctionLocalController
from vanet.lincoln.emergency_driving_control.control_configuration import load_control_configuration, load_clearance_configuration
from vanet.lincoln.v2x_data_exchange.clearance_advice import ClearanceAdvice
from vanet.lincoln.v2x_data_exchange.detailed_logging import initialize_logging

LOG = logging.getLogger('vanet.lincoln')


class LincolnEmergencyService:
    def __init__(self, node):
        self.node = node
        self.task = EmergencyTaskState()
        self.lock = threading.RLock()
        self.commands = queue.Queue()
        self.road_map = node.world.get_map()
        self.controller = None
        self.maximum_speed,self.borrowing_configuration=load_control_configuration()
        self.borrowing_planner = BorrowingPlanner(node,self.road_map,self.borrowing_configuration)
        self.route = []
        self.goal = None
        self.prepared = None
        self.planning_result = None
        self.worker = None
        self.stopped = threading.Event()
        self.generation = 0
        self.ack_frame = -1
        self.position = None
        self.speed = 0.
        self.heading = 0.
        self.simulation_time = 0.
        self.next_advisory = 0.
        self.deviation_since = None
        self.clearance = None
        clearance_configuration,clearance_source=load_clearance_configuration()
        self.clearance_advice=ClearanceAdvice(clearance_configuration,clearance_source)
        self.last_advice_state=None
        self.network = None
        self.arrival_sent = False
        self.resume_after_plan = False
        self.log_handler=initialize_logging(self)
        LOG.info('SERVICE_READY maximum_speed_kmh=%s borrowing_configuration=%s',self.maximum_speed,self.borrowing_configuration)

    def receive_command(self, command):
        if not isinstance(command,dict) or not isinstance(command.get('id'),str):
            return
        with self.lock:
            if command['id'] in self.task.seen_commands:
                return
            self.task.seen_commands.add(command['id'])
        self.commands.put(dict(command))

    def receive_deliveries(self, deliveries):
        with self.lock:
            for delivery in deliveries:
                payload=delivery.get('payload',delivery)
                if self.clearance_advice.receive(payload,self.simulation_time,self.task.task_id,self.task.route_version):
                    self.clearance=payload
                    LOG.info('CLEARANCE %s',payload)

    def process_commands(self):
        """即使桥暂停世界步进，仍在唯一控制线程处理命令和规划完成。"""
        with self.node.control_lock:
            while True:
                try:
                    command=self.commands.get_nowait()
                except queue.Empty:
                    break
                with self.lock:
                    try:
                        self.execute(command)
                    except Exception as error:
                        self.task.state,self.task.error='error',str(error)
                        self.hold_vehicle()
                        LOG.exception('COMMAND_FAILED id=%s action=%s',command['id'],command.get('action'))
                    finally:
                        self.task.command_ack_id=command['id']
                LOG.info('COMMAND id=%s action=%s state=%s emergency=%s',command['id'],command.get('action'),self.task.state,self.task.emergency)
            with self.lock:
                if self.planning_result is not None:
                    generation,outcome,advisory=self.planning_result
                    self.planning_result=None
                    if generation==self.generation:
                        self.consume_plan(outcome,advisory)

    def hold_vehicle(self):
        self.node.driver.disable()
        self.node.autopilot=False
        self.node.active=None
        self.node.cruise_speed=None
        self.node.navigation.hold_brake=True
        self.node.car.apply_control(carla.VehicleControl(brake=1.))

    def execute(self, command):
        action=command['action']
        if action=='prepare':
            self.task.prepare(command)
            self.clearance=None
            self.clearance_advice.reset()
            self.node.navigation.cancel('VANET experiment preparation')
            self.hold_vehicle()
            self.node.navigation.submit_gta_mode(dict(id=command['id']+'-normal-lights',enabled=False))
            self.node.navigation.apply_gta_mode()
            if self.node.navigation.gta_snapshot().get('error'):
                raise RuntimeError('Unable to restore the ordinary traffic-light setting.')
            self.launch_plan(advisory=False)
        elif action=='start':
            self.task.start()
            if self.prepared is None:
                raise ValueError('No prepared route available.')
            route,goal,controller,local=self.prepared
            self.node.navigation.hold_brake=False
            self.node.active=None
            self.node.reverse=False
            if self.task.group=='normal':
                nav=self.node.navigation
                nav.status=dict(id=self.task.task_id,state='planning',detail='Prepared VANET normal route')
                nav.result=(self.task.task_id,(path_with_lookahead(route),goal,route,'',local))
                nav.consume_plan()
                if nav.snapshot()['state']!='navigating':
                    raise RuntimeError('Normal navigation did not acquire the prepared route.')
            else:
                self.node.driver.disable()
                self.controller=controller
                self.node.autopilot=True
                control=self.controller.run_step(self.node.world.get_snapshot())
                self.node.car.apply_control(control)
        elif action=='cancel':
            self.generation+=1
            self.task.cancel_from_sumo()
            self.clearance_advice.reset()
            self.node.navigation.cancel('Emergency cancelled in SUMO')
            self.controller=None
            self.route=[]
            self.prepared=None
            self.hold_vehicle()
        elif action=='reset':
            self.task.reset()
            self.clearance_advice.reset()
            self.generation+=1
            self.node.navigation.cancel('Experiment scene reset')
            self.controller=None
            self.route=[]
            self.prepared=None
            self.hold_vehicle()
            pose=command.get('pose')
            if pose:
                self.node.car.set_target_velocity(carla.Vector3D())
                self.node.car.set_target_angular_velocity(carla.Vector3D())
                self.node.car.set_transform(carla.Transform(carla.Location(x=float(pose['x']),y=float(pose['y']),z=float(pose['z'])),carla.Rotation(yaw=float(pose['yaw']))))
        else:
            raise ValueError('Unsupported Lincoln experiment command: '+str(action))

    def launch_plan(self, advisory=False, resume=False):
        self.generation+=1
        self.resume_after_plan=resume
        generation=self.generation
        destination=dict(self.task.destination)
        group=self.task.group
        origin=self.node.car.get_location()
        def cancelled():return self.stopped.is_set() or generation!=self.generation
        def work():
            try:
                target=carla.Location(**{key:float(destination[key]) for key in ('x','y','z')})
                route,goal=plan_route(self.road_map,origin,target,cancelled)
                controller=local=None
                if group!='normal':
                    controller=EmergencyTrajectoryController(self.node,route,self.road_map,self.maximum_speed)
                elif getattr(self.node.driver,'supports_junction_local_control',False):
                    lengths=[0.]
                    for a,b in zip(route,route[1:]):lengths.append(lengths[-1]+a.transform.location.distance(b.transform.location))
                    local=JunctionLocalController(self.node,route,self.road_map,lengths)
                    if not local.windows:local=None
                if not advisory and not cancelled():
                    ready=self.node.navigation_view.prepare_navigation(route,self.road_map,cancelled)
                    if not ready:return
                elif advisory and not cancelled():
                    self.node.navigation_view.show_route(route,self.road_map)
                outcome=(route,goal,controller,local)
            except Exception as error:
                LOG.exception('ROUTE_PREPARATION_FAILED task=%s',self.task.task_id)
                outcome=error
            with self.lock:
                if not cancelled():self.planning_result=(generation,outcome,advisory)
        self.worker=threading.Thread(target=work,name='vanet-lincoln-planning',daemon=True)
        self.worker.start()

    def consume_plan(self, outcome, advisory):
        if isinstance(outcome,Exception):
            self.task.error=str(outcome)
            if not advisory:self.task.state='error'
            return
        route,goal,controller,local=outcome
        self.route,self.goal=route,goal
        self.prepared=outcome
        if advisory:
            self.task.route_version+=1
        else:
            self.task.prepared()
        LOG.info('ROUTE_READY task=%s version=%s points=%s advisory=%s',self.task.task_id,self.task.route_version,len(route),advisory)
        if not advisory and self.resume_after_plan:
            self.resume_after_plan=False
            self.execute(dict(action='start'))

    def takeover(self):
        with self.lock:
            if self.task.state in ('running','preparing','prepared','arrived','cancelled','reset','error'):
                self.task.takeover()
                self.generation+=1
                self.controller=None
                self.clearance_advice.inactive('manual_control')
                self.next_advisory=0.
                LOG.info('MANUAL_TAKEOVER emergency=%s task=%s',self.task.emergency,self.task.task_id)

    def resume_manual(self):
        if self.task.state in ('preparing','prepared','running','arrived','error'):
            # SUMO 的 Start 才能启动准备好的实验；面板视角按钮也可能调用自动驾驶。
            return True
        if self.task.state!='manual' or self.task.destination is None:
            return False
        # 明确的自动驾驶按钮恢复同一任务，准备完成后自动继续。
        with self.lock:
            self.task.state,self.task.manual='preparing',False
            self.hold_vehicle()
            self.launch_plan(resume=True)
        return True

    def control(self, snapshot):
        with self.lock:
            state=self.task.state
            if state!='running' or self.task.group=='normal':
                self.clearance_advice.inactive('inactive_control')
            if state in ('preparing','prepared','reset','cancelled','error','arrived') and not self.task.manual:
                return True,carla.VehicleControl(brake=1.)
            if state=='manual':
                if self.task.destination and snapshot.timestamp.elapsed_seconds>=self.next_advisory and not (self.worker and self.worker.is_alive()):
                    self.next_advisory=snapshot.timestamp.elapsed_seconds+1.
                    self.launch_plan(advisory=True)
                return False,None
            if state!='running' or self.task.group=='normal':
                return False,None
            if self.controller is None:
                return True,carla.VehicleControl(brake=1.)
            ego=snapshot.find(self.node.car.id)
            if ego is None:
                return True,carla.VehicleControl(brake=1.)
            if self.borrowing_planner.evaluate(self.controller,snapshot):
                self.task.route_version+=1
                self.node.navigation_view.show_route(self.controller.display_waypoints(),None)
            location=ego.get_transform().location
            advice=self.clearance_advice.evaluate(self.simulation_time,self.task.task_id,self.task.route_version,
                (location.x,location.y,location.z),self.controller.samples,self.controller.distances,self.controller.progress)
            signature=(advice['active'],advice['reason'],tuple(advice['vehicles']))
            if signature!=self.last_advice_state:
                LOG.info('CLEARANCE_ADVICE %s',advice)
                self.last_advice_state=signature
            speed_limit=0. if self.borrowing_planner.predicted_conflict else advice['speed_limit_kmh']
            control=self.controller.run_step(snapshot,speed_limit)
            remaining=self.controller.distances[-1]-self.controller.progress
            distance=ego.get_transform().location.distance(self.goal.transform.location)
            if remaining<=4. and distance<=4. and ego.get_velocity().length()<.3:
                self.task.arrive()
                self.node.autopilot=False
                self.node.navigation.hold_brake=True
                LOG.info('ARRIVED task=%s emergency_identity_retained=%s',self.task.task_id,self.task.emergency)
                return True,carla.VehicleControl(brake=1.)
            _,deviation=self.controller.closest_sample(ego.get_transform().location)
            now=snapshot.timestamp.elapsed_seconds
            self.deviation_since=(self.deviation_since or now) if deviation>5. else None
            if self.deviation_since is not None and now-self.deviation_since>.8:
                self.task.state='preparing'
                self.hold_vehicle()
                self.launch_plan(resume=True)
                LOG.warning('REPLAN_REQUIRED deviation=%.2f',deviation)
                return True,carla.VehicleControl(brake=1.)
            return True,control

    def after_frame(self, snapshot):
        with self.lock:
            ego=snapshot.find(self.node.car.id)
            if ego is not None:
                transform=ego.get_transform();p=transform.location
                self.position=dict(x=p.x,y=p.y,z=p.z)
                self.speed=ego.get_velocity().length()
                self.heading=transform.rotation.yaw
            if self.task.group=='normal' and self.task.state=='running':
                nav=self.node.navigation.snapshot()
                if nav['state']=='arrived':self.task.arrive()
                elif nav['state'] in ('error','unreachable'):
                    self.task.state,self.task.error='error',nav.get('detail','')
                elif nav['state']=='cancelled':self.task.takeover()
            is_new_frame=snapshot.frame!=self.ack_frame
            self.ack_frame=snapshot.frame
            if is_new_frame:
                LOG.debug('CONTROL_FRAME state=%s emergency=%s position=%s speed_mps=%.3f controller=%s route_version=%s wait_reason=%s clearance_advice=%s',
                          self.task.state,self.task.emergency,self.position,self.speed,
                          'emergency' if self.controller else 'existing',self.task.route_version,
                          self.controller.wait_reason if self.controller else '',self.clearance_advice.state)

    def snapshot(self, include_route=True):
        with self.lock:
            route=[]
            remaining=0.
            if self.controller is not None:
                remaining=max(0.,self.controller.distances[-1]-self.controller.progress)
                if include_route:route=self.controller.route_payload()
            elif self.route:
                progress=self.node.navigation.route_progress if self.task.group=='normal' else 0.
                lengths=[0.]
                for a,b in zip(self.route,self.route[1:]):lengths.append(lengths[-1]+a.transform.location.distance(b.transform.location))
                remaining=max(0.,lengths[-1]-progress)
                if include_route:
                    stride=max(1,math.ceil(len(self.route)/600))
                    selected=self.route[::stride]
                    if selected[-1] is not self.route[-1]:selected.append(self.route[-1])
                    route=[dict(x=round(w.transform.location.x,3),y=round(w.transform.location.y,3),z=round(w.transform.location.z,3)) for w in selected]
            result=dict(emergency=self.task.emergency,state=self.task.state,group=self.task.group,
                        task_id=self.task.task_id,route_version=self.task.route_version,manual=self.task.manual,
                        eta=remaining/max(2.,self.speed),position=self.position,speed=self.speed,heading=self.heading,
                        remaining_distance=remaining,error=self.task.error,
                        controller='emergency' if self.controller else ('manual' if self.task.manual else 'normal'),
                        target_speed=self.controller.target_speed if self.controller else None,
                        borrowing=self.controller.borrowing if self.controller else False,
                        clearance_advice=dict(self.clearance_advice.state))
            if include_route:result['route']=route
            return result

    def close(self):
        self.stopped.set()
        self.generation+=1
        if self.network:self.network.close()
        if self.worker:self.worker.join(1.)
        LOG.removeHandler(self.log_handler)
        self.log_handler.close()
