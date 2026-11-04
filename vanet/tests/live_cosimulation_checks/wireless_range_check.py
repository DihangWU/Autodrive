"""真实 Veins 无线边界、出圈分片及可选 Qtenv 画面；不连接交通仿真。"""
import argparse
import json
import os
import subprocess
import sys
import time
from vanet.project_paths import (PROJECT_DIRECTORY,EXPERIMENT_DEFAULT_CONFIGURATION_FILE,
    WIRELESS_RANGE_CHECK_SUMMARY_RELATIVE_FILE,WIRELESS_RANGE_CHECK_SOCKET_FILE,
    WIRELESS_RANGE_CHECK_LOG_RELATIVE_FILE)
from vanet.experiments.metrics_and_event_recording.event_log import new_result_directory
from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
from vanet.omnet.build_and_launch.wireless_smoke import wait_for


def direct_command(connection,command):
    channel=connection.socket;channel.settimeout(10.)
    try:
        channel.sendall((json.dumps(command)+'\n').encode())
        response=bytearray()
        while b'\n' not in response:
            chunk=channel.recv(65536)
            if not chunk:raise RuntimeError('Network closed during direct status request.')
            response.extend(chunk)
        return json.loads(response)
    finally:channel.setblocking(False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ui',choices=('Cmdenv','Qtenv'),default='Cmdenv')
    parser.add_argument('--visible',action='store_true')
    args=parser.parse_args()
    folder=new_result_directory('wireless_range_check')
    env=dict(os.environ)
    if args.ui=='Qtenv' and not args.visible:env['QT_QPA_PLATFORM']='offscreen'
    process=None;network=None;report=dict(passed=False,ui=args.ui,scope='Independent real OMNeT++ / Veins; no CARLA or SUMO connection.')
    with (folder/WIRELESS_RANGE_CHECK_LOG_RELATIVE_FILE).open('w') as log:
        try:
            process=subprocess.Popen([sys.executable,'-m','vanet.omnet.build_and_launch.run_network',
                '--ui',args.ui,'--socket',str(WIRELESS_RANGE_CHECK_SOCKET_FILE),'--results',str(folder/'omnet')],
                cwd=PROJECT_DIRECTORY,env=env,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+30.
            while not WIRELESS_RANGE_CHECK_SOCKET_FILE.exists():
                if process.poll() is not None:raise RuntimeError((folder/WIRELESS_RANGE_CHECK_LOG_RELATIVE_FILE).read_text())
                if time.monotonic()>deadline:raise TimeoutError('Network startup timed out.')
                time.sleep(.02)
            network=OmnetConnection(WIRELESS_RANGE_CHECK_SOCKET_FILE,timeout=20.)
            network.start();wait_for(network,lambda:network.ready,process)
            idle_time=network.network_time;time.sleep(.2)
            assert direct_command(network,dict(action='status'))['time']==idle_time,'External clock advanced while idle.'
            sequence=0;step=0;cases=[]
            def node(name,x,y=0.,role='npc',z=.6):return dict(id=name,role=role,x=x,y=y,z=z)
            def advance(nodes,messages=()):
                nonlocal step
                network.submit(step,round(network.network_time+.05,8),nodes,list(messages));step+=1
                wait_for(network,lambda:network.pending is None,process)
                return network.take_result()
            def send(nodes,sender,recipient='broadcast',kind='emergency',padding=''):
                nonlocal sequence
                sequence+=1
                payload=dict(type=kind,task_id='range-check',route_version=1,sequence=sequence,
                    sent_at=network.network_time,source=sender,padding=padding)
                return advance(nodes,[dict(sender=sender,recipient=recipient,payload=payload)])
            if args.ui=='Cmdenv':
                for distance in (49.9,50.,50.1):
                    nodes=[node('lincoln0',0.,role='lincoln'),node('npc0',distance)]
                    for sender,receiver in (('lincoln0','npc0'),('npc0','lincoln0')):
                        result=send(nodes,sender,receiver)
                        received=any(d['recipient']==receiver for d in result['deliveries'])
                        assert received==(distance<=50.),(distance,sender,result)
                        cases.append(dict(kind='V2V',distance=distance,sender=sender,received=received))
                radius=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())['communication_ranges']['rsu_metres']
                for distance in (radius-.1,radius,radius+.1):
                    nodes=[node('lincoln0',distance,role='lincoln'),node('rsu0',0.,role='rsu',z=6.)]
                    for sender,receiver in (('lincoln0','rsu0'),('rsu0','lincoln0')):
                        result=send(nodes,sender,receiver)
                        received=any(d['recipient']==receiver for d in result['deliveries'])
                        assert received==(distance<=radius),(distance,sender,result)
                        cases.append(dict(kind='V2I',distance=distance,sender=sender,received=received))
                nodes=[node('lincoln0',0.,role='lincoln'),node('near',49.),node('outside',50.1)]
                result=send(nodes,'lincoln0')
                assert {d['recipient'] for d in result['deliveries']}=={'near'},result
                report['broadcast_recipient_specific']=True
                for source_role,inside,outside in (('lincoln',49.,51.),('rsu',radius-1.,radius+1.)):
                    sender='source';nodes=[node(sender,0.,role=source_role),node('receiver',inside)]
                    initial=send(nodes,sender,padding='x'*200000)
                    assert not initial['deliveries']
                    assert initial['stats']['fragments_rx']>0
                    nodes[1]['x']=outside;advance(nodes)
                    nodes[1]['x']=inside
                    for _ in range(22):
                        result=advance(nodes)
                        assert not result['deliveries'],'Old fragments resurrected after re-entry.'
                    assert result['stats']['range_reassemblies_discarded']>0
                    fresh=send(nodes,sender)
                    assert any(d['recipient']=='receiver' for d in fresh['deliveries'])
                report['partial_transmission_invalidated_on_exit']=True
                report['old_fragments_cannot_resume_on_reentry']=True
                result=send([node('lincoln0',0.,role='lincoln'),node('npc0',30.,40.,z=60.)],'lincoln0')
                assert any(d['recipient']=='npc0' for d in result['deliveries'])
                report['horizontal_3_4_5_boundary_with_different_heights']=True
            else:
                nodes=[node('lincoln0',170.,-179.,role='lincoln'),node('npc0',135.,-179.),
                       node('npc1',193.,-225.),node('outside_highway',15.,-80.),node('rsu0',193.,-179.,role='rsu',z=6.)]
                send(nodes,'lincoln0');send(nodes,'rsu0','npc0',kind='notice');send(nodes,'npc0','rsu0',kind='feedback')
                time.sleep(.2)
                screenshot=direct_command(network,dict(action='visualization.snapshot'))
                assert screenshot['ok'],screenshot
                report['qtenv_snapshot']=screenshot['snapshot']
            report.update(passed=True,cases=cases,external_time_frozen_while_idle=True)
            network.request_shutdown();wait_for(network,lambda:bool(network.results),process)
            network.take_result();process.wait(timeout=10.)
        finally:
            if network:network.close()
            if process and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3.)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            (folder/WIRELESS_RANGE_CHECK_SUMMARY_RELATIVE_FILE).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'],ui=args.ui,report=str(folder/WIRELESS_RANGE_CHECK_SUMMARY_RELATIVE_FILE),
                         screenshot=report.get('qtenv_snapshot')),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
