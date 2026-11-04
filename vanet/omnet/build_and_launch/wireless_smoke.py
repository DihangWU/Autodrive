"""Run a real-radio integration check without CARLA or SUMO."""
import argparse
import json
import subprocess
import sys
import time
import tempfile
from pathlib import Path
from datetime import datetime, timezone
from vanet.project_paths import PROJECT_DIRECTORY, EXPERIMENT_RESULTS_DIRECTORY, OMNET_RUNTIME_DIRECTORY
from vanet.cobridge.omnet_connection_management.connection import OmnetConnection


def wait_for(connection, condition, process, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        connection.poll()
        if connection.error:
            raise RuntimeError(connection.error)
        if condition():
            return
        if process.poll() is not None:
            raise RuntimeError(f'Network process exited with code {process.returncode}')
        time.sleep(.002)
    raise TimeoutError('Radio smoke check timed out')


def check_archive_path_rejection(connection, output):
    """Exercise both Python validation and the actual C++ protocol boundary."""
    outside = Path(tempfile.gettempdir()) / 'vanet_archive_outside_allowed_root'
    escape = output / 'escape_symlink'
    escape.symlink_to(Path(tempfile.gettempdir()), target_is_directory=True)
    try:
        for directory in (outside, escape):
            try:
                connection.reset(20261104, results_directory=directory)
            except ValueError:
                pass
            else:
                raise AssertionError('Python accepted an archive outside vanet/results')
            # Send directly to verify C++ independently rejects bypassing the client.
            channel = connection.socket
            channel.settimeout(10)
            try:
                channel.sendall((json.dumps(dict(action='reset', seed=20261104,
                                                results_directory=str(directory))) + '\n').encode())
                response = bytearray()
                while b'\n' not in response:
                    chunk = channel.recv(65536)
                    if not chunk:
                        raise RuntimeError('Network closed while rejecting an archive path')
                    response.extend(chunk)
                value = json.loads(response)
                assert value['ok'] is False and 'subdirectory' in value['error'], value
            finally:
                channel.setblocking(False)
    finally:
        escape.unlink()


def run_smoke(output=None):
    output = output or EXPERIMENT_RESULTS_DIRECTORY / ('wireless_smoke_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True, exist_ok=True)
    OMNET_RUNTIME_DIRECTORY.mkdir(parents=True, exist_ok=True)
    socket_path = OMNET_RUNTIME_DIRECTORY / ('smoke_' + str(__import__('os').getpid()) + '.sock')
    with (output / 'process.log').open('w') as log:
        process = subprocess.Popen([sys.executable, '-m', 'vanet.omnet.build_and_launch.run_network', '--ui','Cmdenv','--socket', str(socket_path), '--results', str(output / 'omnet')], cwd=PROJECT_DIRECTORY, stdout=log, stderr=subprocess.STDOUT)
        connection = OmnetConnection(socket_path, timeout=10)
        try:
            deadline = time.monotonic() + 30
            while not socket_path.exists():
                if process.poll() is not None:
                    raise RuntimeError((output / 'process.log').read_text())
                if time.monotonic() > deadline: raise TimeoutError('Network startup timed out')
                time.sleep(.02)
            connection.start()
            wait_for(connection, lambda: connection.ready, process)
            assert connection.capabilities.get('per_run_archives')
            check_archive_path_rejection(connection, output)
            nodes = [dict(id='lincoln0',role='lincoln',x=0,y=0,z=.6), dict(id='npc0',role='npc',x=20,y=0,z=.6), dict(id='rsu0',role='rsu',x=50,y=0,z=5), dict(id='far',role='npc',x=5000,y=0,z=.6)]
            payload = dict(type='emergency',task_id='smoke',route_version=1,sequence=1,sent_at=0,source='lincoln0',route=[dict(x=i,y=0,z=0) for i in range(350)])
            connection.submit(0,.05,nodes,[dict(sender='lincoln0',recipient='broadcast',payload=payload)])
            wait_for(connection,lambda:connection.pending is None,process)
            first=connection.take_result(); delivered=list(first['deliveries'])
            for step in range(1,10):
                connection.submit(step,(step+1)*.05,nodes,[])
                wait_for(connection,lambda:connection.pending is None,process)
                result=connection.take_result();delivered.extend(result['deliveries'])
            receivers={d['recipient'] for d in delivered}
            assert receivers=={'npc0','rsu0'}, receivers
            assert all(d['payload']==payload for d in delivered)
            assert all(0 < d['received_at'] <= .5 for d in delivered)
            direct=dict(type='feedback',source='npc0',task_id='smoke',state='yielding')
            connection.submit(10,.55,nodes,[dict(sender='npc0',recipient='rsu0',payload=direct)])
            wait_for(connection,lambda:connection.pending is None,process)
            reply=connection.take_result()
            assert [d['recipient'] for d in reply['deliveries']]==['rsu0'], reply
            original_stats=reply['stats']
            round_directory = output / 'round_two' / 'omnet'
            connection.reset(20261104, results_directory=round_directory)
            wait_for(connection,lambda:connection.ready,process)
            reset=connection.take_result()
            assert reset['reset'] and reset['time']==.55, reset
            assert reset['artifacts']['results_directory']==str(round_directory.resolve()), reset
            connection.submit(0,.60,nodes,[dict(sender='lincoln0',recipient='broadcast',payload=payload)])
            wait_for(connection,lambda:connection.pending is None,process)
            replay=connection.take_result()
            assert {d['recipient'] for d in replay['deliveries']}==receivers, replay
            replay_delays={d['recipient']:d['received_at']-.55 for d in replay['deliveries']}
            original_delays={d['recipient']:d['received_at'] for d in delivered}
            assert all(abs(replay_delays[k]-original_delays[k])<1e-9 for k in receivers), (original_delays,replay_delays)
            assert replay['stats']['messages_tx']==1 and replay['stats']['messages_rx']==2
            # Remove an actual radio node, then send again: no stale connection-manager entry.
            nodes=[n for n in nodes if n['id']!='npc0']
            connection.submit(1,.65,nodes,[dict(sender='lincoln0',recipient='broadcast',payload={'type':'identity','source':'lincoln0'})])
            wait_for(connection,lambda:connection.pending is None,process)
            removed=connection.take_result()
            assert [d['recipient'] for d in removed['deliveries']]==['rsu0'],removed
            # Switching archives closes the previous interval; legacy reset(seed) stays usable.
            last_directory = output / 'round_three' / 'omnet'
            connection.reset(20261104, results_directory=last_directory)
            wait_for(connection,lambda:connection.ready,process)
            connection.take_result()
            connection.reset(20261104)
            wait_for(connection,lambda:connection.ready,process)
            assert connection.take_result()['reset']
            connection.request_shutdown()
            wait_for(connection,lambda:bool(connection.results),process)
            connection.take_result()
            process.wait(timeout=10)
            archive_events = [json.loads(line) for line in (round_directory / 'wireless_events.jsonl').read_text().splitlines()]
            archive_steps = [json.loads(line) for line in (round_directory / 'network_steps.jsonl').read_text().splitlines()]
            archive_manifest = json.loads((round_directory / 'network_manifest.json').read_text())
            assert archive_events[0]['type']=='network.reset', archive_events[0]
            assert all(.55 <= event['sim_time'] <= .65 for event in archive_events)
            assert [step['step_id'] for step in archive_steps]==[0,1], archive_steps
            assert sum(event['type']=='message.tx' for event in archive_events)==2
            assert sum(event['type']=='message.rx' for event in archive_events)==3
            assert archive_manifest['network_start_time']==.55 and archive_manifest['network_end_time']==.65
            assert archive_manifest['archive_closed'] and archive_manifest['scalar_vector_scope']=='shared_network_process'
            assert Path(archive_manifest['process_scalar_file'])==(output / 'omnet' / 'network.sca').resolve()
            assert Path(archive_manifest['process_scalar_file']).exists()
            final_manifest = json.loads((last_directory / 'network_manifest.json').read_text())
            assert final_manifest['archive_closed'] and final_manifest['network_end_time']==.65
            summary=dict(success=True,radio='Veins IEEE 802.11p',broadcast_receivers=sorted(receivers),out_of_range_received=False,fragmented_payload_bytes=len(json.dumps(payload)),unicast_receivers=['rsu0'],stats=original_stats,reset_replays_same_delays=True,dynamic_node_removal=True,first_receive_times=[d['received_at'] for d in delivered],per_round_archives=True,archive_path_restrictions=True,legacy_reset_compatible=True,native_results_scope='shared_network_process')
            (output / 'summary.json').write_text(json.dumps(summary,indent=2))
            print(json.dumps(summary,indent=2));print('Results:',output)
            return output
        finally:
            connection.close()
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path)
    run_smoke(parser.parse_args().output)
