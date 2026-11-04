"""CoBridge 内的实验协调器：人工任务、帧屏障、无线因果和结果记录。"""
import copy
import json
import logging
import time
import uuid
from pathlib import Path

from vanet.project_paths import EXPERIMENT_DEFAULT_CONFIGURATION_FILE, SCENARIO_TOWN04_CONFIGURATION_FILE, EXPERIMENT_OMNET_RELATIVE_DIRECTORY
from vanet.protocol.message_validation.messages import experiment_group
from vanet.experiments.metrics_and_event_recording.event_log import EventLog, new_result_directory
from vanet.experiments.metrics_and_event_recording.metrics import Metrics
from vanet.cobridge.mobility_snapshot import collect_mobility
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.experiments.scenario_and_parameter_configuration.scenario_controller import ScenarioController, load_scenario
from foundation.cobridge.lincoln.navigation import resolve_destination
from vanet.sumo.vehicle_state_display.vehicle_state_display import VehicleStateDisplay
from vanet.sumo.communication_range_display.lincoln_coverage_circle import LincolnCoverageCircle


class ExperimentCoordinator:
    def __init__(self, bridge):
        self.bridge=bridge
        self.config=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
        self.scenario_config=load_scenario()
        self.phase='waiting'
        self.detail='Select a group and reset the scene in SUMO.'
        self.group='normal'
        self.folder=new_result_directory('bridge')
        self.log=EventLog(self.folder,'CoBridge')
        logging.getLogger('vanet.npc').setLevel(logging.DEBUG)
        logging.getLogger('vanet.rsu').setLevel(logging.DEBUG)
        logging.getLogger('vanet.npc').addHandler(self.log.handler)
        logging.getLogger('vanet.rsu').addHandler(self.log.handler)
        self.log.save_config(dict(parameters=self.config,scenario=self.scenario_config))
        self.metrics=Metrics(self.log,self.config)
        self.scenario=None
        self.display=None
        self.lincoln_coverage=None
        self.network=None
        self.next_network_retry=0.
        self.network_reset_required=False
        self.network_time=0.
        self.run_start_network_time=0.
        self.step_id=0
        self.pending_network=False
        self.network_result=None
        self.lincoln_status={}
        self.ack_frame=-1
        self.command=None
        self.command_ack=None
        self.last_exchange=0.
        self.controller_wait_since=None
        self.npc={}
        self.communication_links={}
        self.rsu=RsuCooperation(self.config)
        self.rsu_counters=dict(received=0,notice_sent=0,feedback_received=0,clearance_sent=0,backfill_sent=0)
        self.deliveries=[]
        self.task_id=''
        self.sequence=0
        self.next_emergency=0.
        self.last_route_version=None
        self.outgoing=[]
        self.reset_steps=0
        self.destination=copy.deepcopy(self.scenario_config['lincoln']['destination'])
        self.starting=False
        self.ended=False
        self.log.emit('coordinator_created',parameters=self.config)

    @property
    def emergency(self):
        return bool(self.lincoln_status.get('emergency',False))

    def status(self):
        return dict(enabled=True,phase=self.phase,detail=self.detail,group=self.group,
                    prepared=self.phase=='prepared',emergency=self.emergency,
                    network=self.network.status if self.network else {},
                    simulation_time=self.network_time-self.run_start_network_time,
                    communication_ranges=dict(self.config.get('communication_ranges',{})),
                    destination=self.destination,results=str(self.folder),
                    lincoln={k:v for k,v in self.lincoln_status.items() if k not in ('route','position')},
                    scenario=self.scenario.snapshot() if self.scenario else None,
                    rsu=dict(self.rsu.snapshot(),counters=dict(self.rsu_counters)),
                    npc_states={name:component.snapshot() for name,component in self.npc.items()})

    def issue(self, action, **fields):
        self.command=dict(id=uuid.uuid4().hex,action=action,group=self.group,
                          task_id=self.task_id,destination=self.destination,**fields)
        self.log.emit('lincoln_command',self.network_time,command=self.command)

    def exchange(self, request):
        records=list(self.bridge.lincoln.records.values())
        if len(records)!=1 or any(request.get(k)!=records[0][k] for k in ('actor_id','world_id')):
            raise ValueError('VANET 只接受当前已登记的唯一 Lincoln。')
        state=request.get('status',{})
        if not isinstance(state,dict):raise ValueError('Lincoln status 必须为对象')
        old=dict(self.lincoln_status)
        if state.get('route_version')!=old.get('route_version') and 'route' not in state:
            self.lincoln_status.pop('route',None)
        self.lincoln_status.update(state)
        self.ack_frame=int(request.get('ack_frame') or -1)
        self.command_ack=request.get('command_ack_id')
        self.last_exchange=time.monotonic()
        if self.command and self.command_ack==self.command['id']:
            self.command=None
        significant=('state','emergency','manual','route_version','error')
        if any(old.get(k)!=state.get(k,old.get(k)) for k in significant):
            self.log.emit('lincoln_state',self.network_time,state={k:self.lincoln_status.get(k) for k in significant})
        received=self.deliveries;self.deliveries=[]
        return dict(command=self.command,simulation_time=self.network_time,deliveries=received)

    def handle(self, request):
        action=request['action']
        if action=='vanet.lincoln.exchange':return self.exchange(request)
        if action=='vanet.status':return self.status()
        if self.bridge.carla is None or self.bridge.sumo is None:raise RuntimeError('CARLA/SUMO 尚未接入。')
        if action=='vanet.scene.initialize':
            if self.phase=='waiting':self.reset()
            return self.status()
        if request.get('source') not in ('sumo_gui','operator_cli'):
            raise ValueError('请在 SUMO 实验操作栏或显式操作命令 vanet_control 执行此操作。')
        if Path(request.get('network_file','')).resolve()!=Path(self.bridge.sumo.network['network_file']).resolve():
            raise ValueError('SUMO 窗口路网与当前实验不一致。')
        self.log.emit('user_operation',self.network_time,request=request)
        if action=='vanet.reset':
            if self.emergency:raise RuntimeError('请先在 SUMO 取消紧急状态。')
            if self.phase=='running':raise RuntimeError('请先结束当前实验。')
            self.group=experiment_group(request.get('group',self.group))
            self.reset()
        elif action=='vanet.prepare':
            if self.phase not in ('ready','prepared','continuation_ready'):raise RuntimeError('请先恢复场景并等待就绪。')
            if self.ended and self.phase!='continuation_ready':raise RuntimeError('新一轮对照实验需先恢复初始场景。')
            group=experiment_group(request.get('group',self.group))
            if group!=self.group:raise RuntimeError('更换实验组后请先恢复场景。')
            if self.phase=='continuation_ready':
                previous=str(self.folder)
                self.close_log()
                self.folder=new_result_directory('continuation');self.log=EventLog(self.folder,'CoBridge')
                self.attach_log()
                self.log.save_config(dict(group=self.group,parameters=self.config,scenario=self.scenario_config,
                    destination=self.destination,continuation_of=previous,fixed_scene_comparison=False))
                self.metrics=Metrics(self.log,self.config);self.ended=False
            self.task_id=uuid.uuid4().hex
            self.issue('prepare')
            self.phase='preparing';self.detail='Planning route and waiting for RViz.'
        elif action=='vanet.start':
            if self.phase!='prepared':raise RuntimeError('路线、车辆和网络尚未全部准备完成。')
            self.issue('start');self.starting=True;self.detail='Waiting for Lincoln start acknowledgement.'
        elif action=='vanet.cancel':
            if not self.emergency:raise RuntimeError('Lincoln 当前没有紧急身份。')
            self.issue('cancel')
            if self.phase=='running':
                self.metrics.values['status']='user_cancelled'
                self.ended=True
            elif self.phase in ('preparing','prepared','ready'):
                self.metrics.values['status']='cancelled_before_start'
                self.ended=True
            self.queue_cancel('user_cancelled')
            # 继续仿真直至停车，并让取消消息通过无线传播。
            if self.phase in ('prepared','preparing','ready','running','arrived'):self.phase='cancelling'
        else:raise ValueError(f'未知实验操作 {action}')
        return self.status()

    def set_destination(self, request):
        self.bridge.lincoln.navigation.find_vehicle(request,self.bridge.carla,self.bridge.sumo)
        if self.phase in ('running','paused','cancelling'):
            raise RuntimeError('当前实验目的地已固定，请取消紧急状态并恢复场景后重新选择。')
        self.destination=resolve_destination(self.bridge.sumo.network,request['x'],request['y'])
        if self.phase=='prepared':self.phase='ready'
        if self.phase=='arrived':self.phase='continuation_ready'
        self.log.emit('destination_selected',self.network_time,destination=self.destination)
        return dict(state='idle',detail='Destination stored. Use VANET Prepare, then Start.')

    def reset(self):
        from foundation.cobridge.npcvehicules.official_tools import load_official_tools
        from foundation.cobridge.npcvehicules.vehicle_sync import NpcSumoAdapter,NpcCarlaAdapter
        from foundation.cobridge.npcvehicules.run_synchronization import SimulationSynchronization
        bridge=self.bridge
        if len(bridge.lincoln.records)!=1:raise RuntimeError('场景需要一辆已登记的 Lincoln。')
        if any(name!='vanet_scenario' for name in bridge.npc.batches):raise RuntimeError('请先关闭独立随机 NPC 批次，以保证固定场景。')
        if self.display:self.display.close()
        if self.lincoln_coverage:self.lincoln_coverage.close()
        self.display=VehicleStateDisplay(bridge.sumo.connection)
        previous=bridge.npc.batches.pop('vanet_scenario',None)
        if previous:previous['synchronization'].close()
        if self.scenario is None:self.scenario=ScenarioController(bridge.sumo.connection,self.scenario_config)
        self.scenario.reset(emergency_active=self.emergency)
        tools=load_official_tools();traffic=self.scenario.traffic
        synchronization=SimulationSynchronization(NpcSumoAdapter(bridge.sumo,traffic,tools),NpcCarlaAdapter(bridge.carla,tools),tools.helper)
        bridge.npc.batches['vanet_scenario']=dict(owner=self,traffic=traffic,synchronization=synchronization,
            carla=bridge.carla,sumo=bridge.sumo,spawn_per_step=traffic.count,state='starting',detail='',closing=False,active=0)
        self.close_log();self.folder=new_result_directory('run');self.log=EventLog(self.folder,'CoBridge')
        self.attach_log()
        self.log.save_config(dict(group=self.group,parameters=self.config,scenario=self.scenario_config,destination=self.destination))
        self.metrics=Metrics(self.log,self.config);self.npc={};self.rsu=RsuCooperation(self.config)
        self.communication_links={}
        self.rsu_counters=dict(received=0,notice_sent=0,feedback_received=0,clearance_sent=0,backfill_sent=0)
        self.pending_network=False;self.network_result=None;self.deliveries=[]
        self.network_reset_required=True
        self.task_id='';self.next_emergency=self.network_time;self.last_route_version=None;self.outgoing=[]
        self.ended=False;self.starting=False;self.reset_steps=20
        pose=self.scenario_config['lincoln']['spawn']
        self.issue('reset',pose=pose)
        self.phase='resetting';self.detail='Restoring Lincoln, fixed NPCs and traffic lights.'
        self.log.emit('scene_reset_requested',self.network_time,group=self.group)

    def connect_network(self):
        if self.network is None:
            from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
            self.network=OmnetConnection(timeout=self.config['network_timeout_seconds'])
            self.network.start()
        self.network.poll()
        if self.network.status.get('error') and self.phase in ('waiting','ready','resetting') and time.monotonic()>=self.next_network_retry:
            self.next_network_retry=time.monotonic()+1.
            self.network.close()
            from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
            self.network=OmnetConnection(timeout=self.config['network_timeout_seconds'])
            self.network.start()
            if self.phase=='resetting':self.network_reset_required=True
        return self.network.status

    def pause(self, reason):
        if self.phase!='paused':
            self.phase='paused';self.detail=reason
            self.metrics.values['status']='invalid_paused'
            self.log.emit('experiment_paused',self.network_time,level='error',reason=reason)
            self.log.save_metrics(self.metrics.values)
        return False

    def before_step(self):
        bridge=self.bridge
        try:
            network=self.connect_network()
            if self.phase=='waiting':return True
            if bridge.carla is None or bridge.sumo is None:return self.pause('Simulator disconnected.')
            if self.phase=='paused':return False
            if self.phase=='running' and self.lincoln_status.get('state')=='error':
                return self.pause('Lincoln: '+self.lincoln_status.get('error','control failed'))
            if self.phase=='resetting':
                if self.network_reset_required:
                    if not network.get('ready'):return False
                    self.network.reset(self.config['seed'],results_directory=self.folder/EXPERIMENT_OMNET_RELATIVE_DIRECTORY)
                    self.network_reset_required=False
                    return False
                if network.get('reset_pending'):return False
                reset_result=self.network.take_result()
                if reset_result is not None:
                    if not reset_result.get('reset'):raise RuntimeError('Expected network reset acknowledgement.')
                    self.network_time=float(reset_result['time'])
                    self.run_start_network_time=self.network_time
                    self.log.emit('network_reset',self.network_time,seed=self.config['seed'],artifacts=reset_result.get('artifacts',{}))
                if self.reset_steps>0:
                    self.reset_steps-=1;return True
                if self.command is None and self.lincoln_status.get('state')=='reset' and self.scenario.snapshot().get('initial_npcs_added'):
                    self.phase='ready';self.detail='Scene restored. Select Prepare / Enable.'
                return False
            if self.phase=='preparing':
                state=self.lincoln_status.get('state')
                if state=='error':self.detail=self.lincoln_status.get('error','Route preparation failed.');return False
                if state=='prepared' and network.get('ready'):
                    self.phase='prepared';self.detail='Ready. Click Start experiment in SUMO.'
                    self.log.emit('experiment_prepared',self.network_time)
                return False
            if self.phase=='prepared' and self.starting and self.command is None:
                if self.lincoln_status.get('state')=='error':
                    return self.pause('Lincoln: '+self.lincoln_status.get('error','start failed'))
                if self.lincoln_status.get('state') not in ('running','manual'):return False
                self.starting=False;self.phase='running'
                self.detail='Experiment running; CoBridge coordinates traffic and wireless time.'
                self.run_start_network_time=self.network_time
                if not self.scenario.started:self.scenario.start(0.)
                self.metrics.values['status']='running'
                self.log.emit('experiment_started',self.network_time,group=self.group)
            if self.phase in ('ready','prepared','continuation_ready'):return False
            if self.phase not in ('running','cancelling','arrived'):return False
            if network.get('error'):return self.pause('OMNeT++: '+str(network['error']))
            if not network.get('ready'):return False
            if self.pending_network:
                result=self.network.take_result()
                if result is None:return False
                if not result.get('ok',True):return self.pause(str(result))
                self.network_result=result;self.pending_network=False
                return True
            frame=bridge.carla.world.get_snapshot().frame
            if self.ack_frame<frame:
                if self.controller_wait_since is None:self.controller_wait_since=time.monotonic()
                if time.monotonic()-self.controller_wait_since>self.config['controller_timeout_seconds']:
                    return self.pause('Lincoln frame acknowledgement timed out.')
                return False
            self.controller_wait_since=None
            states,nodes=collect_mobility(bridge,self.config['rsu'])
            self.scenario.step(self.network_time-self.run_start_network_time)
            transmissions=self.application_step(states)
            self.step_id+=1
            self.network.submit(self.step_id,round(self.network_time+self.config['step_length'],8),nodes,transmissions)
            self.pending_network=True
            for message in transmissions:self.log.emit('wireless_send_requested',self.network_time,level='debug',message=message)
            if not self.ended:self.metrics.values['messages_sent']+=len(transmissions)
            return False
        except Exception as error:
            self.log.logger.exception('Coordinator failed')
            return self.pause(str(error))

    def queue_cancel(self, reason):
        if not self.task_id:return
        self.sequence+=1
        self.outgoing.append(dict(sender='lincoln0',recipient='broadcast',payload=dict(type='cancel',source='lincoln0',
            task_id=self.task_id,route_version=self.lincoln_status.get('route_version',0),sequence=self.sequence,
            sent_at=self.network_time,reason=reason)))

    def application_step(self, states):
        now=self.network_time;messages=self.outgoing;self.outgoing=[]
        for state in states:
            if state['role']=='npc':
                component=self.npc.setdefault(state['id'],NpcCooperation(state['id'],self.config))
                component.set_communication_links(self.communication_links.get(state['id'],[]),now)
                messages.extend(component.step(now,state,self.bridge.sumo.connection))
                if not self.ended:
                    self.metrics.note_yield_state(state['id'],component.status,now-self.run_start_network_time,
                        (component.task or {}).get('task_id'),component.reason)
                if self.display:self.display.npc(state['id'],component.status,component.reason,(component.task or {}).get('task_id'))
            elif state['role']=='lincoln' and self.display:
                self.display.lincoln(state.get('sumo_id',state['id']),self.emergency,self.lincoln_status.get('state',''))
        if self.display:self.display.prune(state.get('sumo_id',state['id']) for state in states)
        if self.group=='cooperative':
            messages.extend(self.rsu.step(now))
            version=self.lincoln_status.get('route_version',0)
            if self.emergency and (now+1e-8>=self.next_emergency or version!=self.last_route_version):
                route=self.lincoln_status.get('route',[])
                kind='emergency' if route and self.phase=='running' and not self.ended else 'identity'
                self.sequence+=1
                position=self.lincoln_status.get('position') or {}
                payload=dict(type=kind,source='lincoln0',task_id=self.task_id,route_version=version,
                    sequence=self.sequence,sent_at=now,route=route if kind=='emergency' else [],
                    position=position,eta=self.lincoln_status.get('eta'),speed=self.lincoln_status.get('speed',0.),heading=self.lincoln_status.get('heading',0.))
                messages.append(dict(sender='lincoln0',recipient='broadcast',payload=payload))
                self.next_emergency=now+self.config['emergency_interval'];self.last_route_version=version
        # 非协同组仅允许常规信标，不下发紧急协同消息。
        if self.group!='cooperative':messages=[m for m in messages if m['payload']['type']=='beacon']
        for message in messages:
            if message['sender']==self.config['rsu']['id']:
                kind=message['payload']['type']
                if kind in ('notice','clearance'):self.rsu_counters[kind+'_sent']+=1
                if message['payload'].get('backfill'):self.rsu_counters['backfill_sent']+=1
        return messages

    def after_step(self):
        if self.bridge.sumo is not None:
            if self.lincoln_coverage is None:
                self.lincoln_coverage=LincolnCoverageCircle(self.bridge.sumo.connection,
                    self.config['communication_ranges']['v2v_metres'])
            record=next(iter(self.bridge.lincoln.records.values()),{})
            self.lincoln_coverage.update(record.get('sumo_id'))
        if self.network_result is None:return
        result=self.network_result;self.network_result=None
        self.network_time=float(result['time'])
        self.communication_links=result.get('communication_links',{})
        for name,component in self.npc.items():component.set_communication_links(self.communication_links.get(name,[]),self.network_time)
        self.rsu.set_communication_links(self.communication_links.get(self.config['rsu']['id'],[]),self.network_time)
        self.log.network_boundary(result)
        self.log.emit('network_boundary',self.network_time,level='debug',step_id=self.step_id,stats=result.get('stats',{}))
        for delivery in result.get('deliveries',[]):
            self.log.emit('wireless_received',delivery['received_at'],level='debug',delivery=delivery)
            if not self.ended:self.metrics.delivery(delivery)
            message=delivery['payload'];recipient=delivery['recipient']
            if message.get('type') not in ('beacon','identity') and message.get('task_id')!=self.task_id:
                self.log.emit('stale_task_discarded',self.network_time,level='debug',message=message)
                continue
            if self.group=='cooperative':
                if recipient==self.config['rsu']['id']:
                    if self.rsu.receive(message,delivery['received_at']):
                        self.rsu_counters['received']+=1
                        if message.get('type')=='feedback':self.rsu_counters['feedback_received']+=1
                elif recipient in self.npc:
                    component=self.npc[recipient]
                    accepted=component.receive(message,delivery['received_at'])
                    if accepted and message.get('type') in ('emergency','notice') and not self.ended:
                        self.metrics.note_yield_state(recipient,'received',delivery['received_at']-self.run_start_network_time,
                            message.get('task_id'),'valid_wireless_message')
                elif recipient=='lincoln0':self.deliveries.append(delivery)
        states,_=collect_mobility(self.bridge,self.config['rsu'])
        elapsed=self.network_time-self.run_start_network_time
        if not self.ended:self.metrics.sample(elapsed,states,self.bridge.sumo.connection)
        if self.lincoln_status.get('state')=='arrived' and not self.ended:
            self.ended=True;self.phase='arrived';self.metrics.values.update(arrived=True,status='arrived',elapsed_seconds=elapsed)
            self.queue_cancel('route_arrived');self.log.emit('experiment_arrived',self.network_time,elapsed_seconds=elapsed)
            self.log.save_metrics(self.metrics.values);self.detail='Arrived. Emergency identity remains until cancelled in SUMO.'
        if self.phase=='cancelling' and not self.emergency and self.lincoln_status.get('speed',0.)<.3 and all(n.status=='normal' for n in self.npc.values()):
            self.phase='ready';self.detail='Emergency cancelled; restore scene before the next run.'
        if self.step_id%20==0:self.log.save_metrics(self.metrics.values)

    def attach_log(self):
        for name in ('vanet.npc','vanet.rsu'):
            logger=logging.getLogger(name);logger.setLevel(logging.DEBUG);logger.addHandler(self.log.handler)

    def close_log(self):
        for name in ('vanet.npc','vanet.rsu'):
            logging.getLogger(name).removeHandler(self.log.handler)
        self.log.close()

    def close(self):
        self.metrics.values['status']='closed' if not self.ended else self.metrics.values['status']
        self.log.save_metrics(self.metrics.values)
        if self.display:self.display.close()
        if self.lincoln_coverage:self.lincoln_coverage.close()
        if self.network:
            try:
                deadline=time.monotonic()+2.
                while self.network.pending and time.monotonic()<deadline:
                    self.network.poll();time.sleep(.005)
                if self.network.ready and not self.network.pending:
                    self.network.request_shutdown()
                    while time.monotonic()<deadline:
                        self.network.poll()
                        result=self.network.take_result()
                        if result and result.get('stopped'):break
                        time.sleep(.005)
            except Exception:
                self.log.logger.exception('Network shutdown incomplete')
            finally:self.network.close()
        self.log.emit('coordinator_closed',self.network_time)
        self.close_log()
