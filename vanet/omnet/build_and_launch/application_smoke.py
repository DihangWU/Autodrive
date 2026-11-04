"""Real Veins radio + actual RSU/NPC policies; mocked SUMO actions, no live traffic."""
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from types import SimpleNamespace
from unittest.mock import Mock
from vanet.project_paths import PROJECT_DIRECTORY, EXPERIMENT_RESULTS_DIRECTORY, OMNET_RUNTIME_DIRECTORY, EXPERIMENT_DEFAULT_CONFIGURATION_FILE
from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
from vanet.omnet.build_and_launch.wireless_smoke import wait_for
from vanet.npc.cooperation import NpcCooperation
from vanet.rsu.cooperation import RsuCooperation


def main():
    directory=EXPERIMENT_RESULTS_DIRECTORY / ('wireless_application_smoke_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    directory.mkdir(parents=True)
    OMNET_RUNTIME_DIRECTORY.mkdir(parents=True,exist_ok=True)
    socket_path=OMNET_RUNTIME_DIRECTORY / ('application_smoke_'+str(__import__('os').getpid())+'.sock')
    traffic=SimpleNamespace(vehicle=Mock(),edge=Mock(),lane=Mock())
    traffic.vehicle.getSpeedMode.return_value=31
    traffic.vehicle.getLaneChangeMode.return_value=1621
    traffic.vehicle.getMaxSpeed.return_value=13.89
    traffic.vehicle.getLaneIndex.return_value=0
    traffic.vehicle.getRoadID.return_value='edge'
    traffic.vehicle.getLaneID.return_value='edge_0'
    traffic.vehicle.getLateralLanePosition.return_value=0.
    traffic.edge.getLaneNumber.return_value=2
    traffic.lane.getWidth.return_value=3.5
    traffic.vehicle.couldChangeLane.return_value=True
    state=dict(id='npc0',x=30.,y=0.,z=.6,heading=0.,speed=5.,width=1.8,lane_id='edge_0')
    nodes=[dict(id='lincoln0',role='lincoln',x=0,y=0,z=.6),dict(state,role='npc'),dict(id='rsu0',role='rsu',x=50,y=0,z=6)]
    config={'beacon_interval':1.,'lease_seconds':10.,
            'communication_ranges':json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())['communication_ranges']}
    npc=NpcCooperation('npc0',config);rsu=RsuCooperation(config)
    history=[]
    with (directory/'process.log').open('w') as log:
        process=subprocess.Popen([sys.executable,'-m','vanet.omnet.build_and_launch.run_network','--ui','Cmdenv','--socket',str(socket_path),'--results',str(directory/'omnet')],cwd=PROJECT_DIRECTORY,stdout=log,stderr=subprocess.STDOUT)
        network=OmnetConnection(socket_path)
        try:
            deadline=time.monotonic()+30
            while not socket_path.exists():
                if process.poll() is not None:raise RuntimeError((directory/'process.log').read_text())
                if time.monotonic()>deadline:raise TimeoutError('Network startup timed out')
                time.sleep(.02)
            network.start();wait_for(network,lambda:network.ready,process)
            for step in range(60):
                now=round(step*.05,8)
                transmissions=npc.step(now,state,traffic)+rsu.step(now)
                if step%20==0:
                    payload=dict(type='emergency',task_id='radio-chain',route_version=1,sequence=step//20+1,sent_at=now,source='lincoln0',route=[dict(x=0,y=0,z=.6),dict(x=200,y=0,z=.6)],position=dict(x=0,y=0,z=.6),speed=10.)
                    # Deliver V2I only, so a subsequent NPC action proves real I2V delivery.
                    transmissions.append(dict(sender='lincoln0',recipient='rsu0',payload=payload))
                network.submit(step,round(now+.05,8),nodes,transmissions)
                wait_for(network,lambda:network.pending is None,process)
                result=network.take_result()
                npc.set_communication_links(result['communication_links'].get('npc0',[]),result['time'])
                rsu.set_communication_links(result['communication_links'].get('rsu0',[]),result['time'])
                for delivery in result['deliveries']:
                    history.append(delivery)
                    if delivery['recipient']=='rsu0':rsu.receive(delivery['payload'],delivery['received_at'])
                    elif delivery['recipient']=='npc0':npc.receive(delivery['payload'],delivery['received_at'])
            def received(kind,recipient):
                return [d for d in history if d['recipient']==recipient and d['payload']['type']==kind]
            assert received('beacon','rsu0')
            assert received('emergency','rsu0')
            assert received('notice','npc0')
            assert received('feedback','rsu0')
            assert any(d['payload'].get('vehicles',{}).get('npc0')=='yielding' for d in received('clearance','lincoln0'))
            traffic.vehicle.changeLane.assert_called_once_with('npc0',1,3.)
            assert not received('emergency','npc0'), 'NPC should act on the received RSU notice in this test'
            network.request_shutdown();wait_for(network,lambda:bool(network.results),process);network.take_result();process.wait(timeout=10)
            summary=dict(success=True,radio='Veins IEEE 802.11p',traffic_actions='mock SUMO interface',observed_chain=['NPC beacon -> RSU','Lincoln emergency -> RSU','RSU notice -> NPC','NPC yielding feedback -> RSU','RSU clearance summary -> Lincoln'],deliveries=len(history),npc_status=npc.status)
            (directory/'summary.json').write_text(json.dumps(summary,indent=2))
            (directory/'delivered_messages.jsonl').write_text(''.join(json.dumps(d)+'\n' for d in history))
            print(json.dumps(summary,indent=2));print('Results:',directory)
        finally:
            network.close()
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
