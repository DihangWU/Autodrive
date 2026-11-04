"""真实 Veins 检查直连/中转切换；SUMO 动作使用 mock，不接管现有实验。"""
import json
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock
from vanet.project_paths import (PROJECT_DIRECTORY,EXPERIMENT_OMNET_RELATIVE_DIRECTORY,
    COMMUNICATION_PATH_CHECK_SOCKET_FILE,COMMUNICATION_PATH_CHECK_SUMMARY_RELATIVE_FILE,
    COMMUNICATION_PATH_CHECK_LOG_RELATIVE_FILE,COMMUNICATION_PATH_CHECK_DELIVERIES_RELATIVE_FILE)
from vanet.experiments.metrics_and_event_recording.event_log import new_result_directory
from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
from vanet.omnet.build_and_launch.wireless_smoke import wait_for
from vanet.protocol.communication_path_selection.paths import select_communication_paths
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation
from vanet.lincoln.v2x_data_exchange.wireless_clearance_advice import WirelessClearanceAdvice


def mock_traffic():
    traffic=SimpleNamespace(vehicle=Mock(),edge=Mock(),lane=Mock())
    for method,value in dict(getSpeedMode=31,getLaneChangeMode=1621,getMaxSpeed=13.89,
        getLaneIndex=0,getRoadID='edge',getLaneID='edge_0',getLateralLanePosition=0.,couldChangeLane=True).items():
        getattr(traffic.vehicle,method).return_value=value
    traffic.edge.getLaneNumber.return_value=2;traffic.lane.getWidth.return_value=3.5
    return traffic


def main():
    folder=new_result_directory('communication_path_check')
    config=dict(communication_ranges=dict(v2v_metres=50.,rsu_metres=170.),
                beacon_phase_spread_seconds=.8,seed=20261104,
                rsu=dict(id='rsu0',x=100.,y=0.,z=6.))
    npc=NpcCooperation('npc0',config);rsu=RsuCooperation(config);advice=WirelessClearanceAdvice()
    traffic=mock_traffic();network=None;process=None;history=[];cases=[];step=0;sequence=0
    with (folder/COMMUNICATION_PATH_CHECK_LOG_RELATIVE_FILE).open('w') as log:
        try:
            process=subprocess.Popen([sys.executable,'-m','vanet.omnet.build_and_launch.run_network',
                '--ui','Cmdenv','--socket',str(COMMUNICATION_PATH_CHECK_SOCKET_FILE),
                '--results',str(folder/EXPERIMENT_OMNET_RELATIVE_DIRECTORY)],
                cwd=PROJECT_DIRECTORY,stdout=log,stderr=log)
            deadline=time.monotonic()+30.
            while not COMMUNICATION_PATH_CHECK_SOCKET_FILE.exists():
                if process.poll() is not None:raise RuntimeError('Network exited; inspect path check log.')
                if time.monotonic()>deadline:raise TimeoutError('Network startup timed out.')
                time.sleep(.01)
            network=OmnetConnection(COMMUNICATION_PATH_CHECK_SOCKET_FILE,timeout=10.)
            network.start();wait_for(network,lambda:network.ready,process)
            for version,(name,ego_x,npc_x,expected) in enumerate((
                ('direct_inside_rsu',0.,30.,'direct'),('relay_beyond_v2v',0.,80.,'rsu'),
                ('return_to_direct',0.,30.,'direct'),('direct_outside_rsu',300.,330.,'direct'),
                ('no_relay_without_lincoln_rsu_link',300.,100.,None)),1):
                observed=[];advice_received=False
                own=dict(id='npc0',role='npc',x=npc_x,y=0.,z=.6,speed=5.,heading=0.,width=1.8,lane_id='edge_0')
                states=[dict(id='lincoln0',role='lincoln',x=ego_x,y=0.,z=.6),own]
                nodes=[{k:v for k,v in state.items() if k in ('id','role','x','y','z','heading','speed')} for state in states]
                nodes.append(dict(config['rsu'],role='rsu'))
                paths,connected=select_communication_paths(states,config['rsu'],config['communication_ranges'])
                assert paths['npc0']==expected
                npc.set_communication_path(expected,network.network_time)
                rsu.set_relay_targets([target for target,path in paths.items() if path=='rsu'],network.network_time)
                advice.set_paths(paths,connected)
                for tick in range(60):
                    now=network.network_time
                    transmissions=npc.step(now,own,traffic)+rsu.step(now)
                    if tick%20==0:
                        sequence+=1
                        payload=dict(type='emergency',source='lincoln0',task_id='path-check',route_version=version,
                            sequence=sequence,sent_at=now,position=dict(x=ego_x,y=0.,z=.6),speed=10.,
                            route=[dict(x=ego_x,y=0.,z=.6),dict(x=ego_x+200.,y=0.,z=.6)])
                        transmissions.append(dict(sender='lincoln0',recipient='broadcast',payload=payload))
                    network.submit(step,round(now+.05,8),nodes,transmissions);step+=1
                    wait_for(network,lambda:network.pending is None,process)
                    result=network.take_result()
                    npc.set_communication_links(result['communication_links'].get('npc0',[]),result['time'])
                    rsu.set_communication_links(result['communication_links'].get('rsu0',[]),result['time'])
                    for delivery in result['deliveries']:
                        observed.append(delivery);history.append(dict(case=name,**delivery))
                        message=delivery['payload'];recipient=delivery['recipient']
                        if recipient=='npc0':npc.receive(message,delivery['received_at'])
                        elif recipient=='rsu0':rsu.receive(message,delivery['received_at'])
                        elif recipient=='lincoln0':
                            accepted=advice.receive(message,delivery['received_at'],'path-check',version)
                            advice_received|=accepted and message['type'] in ('feedback','clearance')
                received=lambda kind,target:[d for d in observed if d['payload']['type']==kind and d['recipient']==target]
                if expected=='direct':
                    assert received('emergency','npc0') and received('feedback','lincoln0'),name
                    assert not received('notice','npc0') and not received('feedback','rsu0'),name
                    assert not received('beacon','rsu0'),name
                    assert advice_received,name
                elif expected=='rsu':
                    assert not received('emergency','npc0') and not received('feedback','lincoln0'),name
                    assert received('notice','npc0') and received('feedback','rsu0'),name
                    assert received('clearance','lincoln0') and advice_received,name
                else:
                    assert not received('notice','npc0') and not received('feedback','lincoln0'),name
                    assert not received('feedback','rsu0'),name
                cases.append(dict(name=name,path=expected,deliveries=len(observed),feedback_used=advice_received))
            network.request_shutdown();wait_for(network,lambda:bool(network.results),process)
            network.take_result();process.wait(timeout=10.)
            summary=dict(passed=True,scope='Real OMNeT++/Veins with actual NPC/RSU/advice policies; SUMO actions mocked.',cases=cases)
            (folder/COMMUNICATION_PATH_CHECK_SUMMARY_RELATIVE_FILE).write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps(dict(**summary,results=str(folder)),ensure_ascii=False,indent=2))
        finally:
            (folder/COMMUNICATION_PATH_CHECK_DELIVERIES_RELATIVE_FILE).write_text(''.join(json.dumps(d)+'\n' for d in history))
            if network:network.close()
            if process and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
