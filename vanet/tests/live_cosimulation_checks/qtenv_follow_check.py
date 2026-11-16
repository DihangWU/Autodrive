"""独立离屏 Qtenv 跟随检查；不连接或推进当前 CARLA/SUMO。"""
import json
import os
import subprocess
import sys
import time

from vanet.project_paths import (PROJECT_DIRECTORY, QTENV_FOLLOW_CHECK_SOCKET_FILE,
    QTENV_FOLLOW_CHECK_LOG_RELATIVE_FILE, QTENV_FOLLOW_CHECK_SUMMARY_RELATIVE_FILE)
from vanet.cobridge.omnet_connection_management import OmnetConnection
from vanet.experiments.metrics_and_event_recording.event_log import new_result_directory
from vanet.omnet.build_and_launch.wireless_smoke import wait_for
from vanet.tests.live_cosimulation_checks.wireless_range_check import direct_command


def main():
    folder = new_result_directory('qtenv_follow_check')
    report = dict(passed=False, scope='Independent offscreen Qtenv; no traffic simulator connection')
    network = process = None
    with (folder / QTENV_FOLLOW_CHECK_LOG_RELATIVE_FILE).open('w') as log:
        try:
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
            process = subprocess.Popen([sys.executable,'-m','vanet.omnet.build_and_launch.run_network',
                '--ui','Qtenv','--socket',str(QTENV_FOLLOW_CHECK_SOCKET_FILE),'--results',str(folder/'omnet')],
                cwd=PROJECT_DIRECTORY,env=env,stdout=log,stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 45.
            while not QTENV_FOLLOW_CHECK_SOCKET_FILE.exists():
                if process.poll() is not None:
                    raise RuntimeError((folder/QTENV_FOLLOW_CHECK_LOG_RELATIVE_FILE).read_text())
                if time.monotonic()>deadline:raise TimeoutError('Qtenv startup timed out')
                time.sleep(.02)
            network = OmnetConnection(QTENV_FOLLOW_CHECK_SOCKET_FILE,timeout=15.)
            network.start();wait_for(network,lambda:network.ready,process)
            step = 0
            def advance(x,y):
                nonlocal step
                nodes = [dict(id='lincoln0',role='lincoln',x=x,y=y,z=.6,speed=0.,heading=0.)]
                network.submit(step,round(network.network_time+.05,8),nodes,[]);step+=1
                wait_for(network,lambda:network.pending is None,process)
                network.take_result()
                time.sleep(.2)
                return direct_command(network,dict(action='visualization.follow'))
            results = []
            for x,y in [(-515.,240.),(-450.,200.),(-300.,100.),(100.,-179.),(285.,-118.)]:
                state = advance(x,y)
                assert state['ok'] and state['follow']['enabled'] and state['follow']['attached'], state
                assert state['follow']['control_visible'], state
                assert state['follow']['center_error_pixels']<=3., state
                results.append(state)
            before = direct_command(network,dict(action='visualization.follow',enabled=False))
            after = advance(-400.,-250.)
            assert not after['follow']['enabled'], after
            assert before['follow']['center']==after['follow']['center'], (before,after)
            assert after['follow']['center_error_pixels']>10., after
            direct_command(network,dict(action='visualization.follow',enabled=True))
            time.sleep(.15)
            restored = direct_command(network,dict(action='visualization.follow'))
            assert restored['follow']['center_error_pixels']<=3., restored
            frozen_time = restored['time'];time.sleep(.2)
            assert direct_command(network,dict(action='status'))['time']==frozen_time
            network.reset(20261104);wait_for(network,lambda:not network.reset_pending,process);network.take_result()
            state = advance(-515.,240.)
            assert state['follow']['enabled'] and state['follow']['center_error_pixels']<=3., state
            screenshot = direct_command(network,dict(action='visualization.snapshot'))
            assert screenshot['ok'], screenshot
            report.update(positions_followed=results,disabled_view_fixed=True,
                restored=True,idle_clock_frozen=True,reset_rebound=True,screenshot=screenshot['snapshot'])
            network.request_shutdown();wait_for(network,lambda:bool(network.results),process)
            assert network.take_result().get('stopped')
            process.wait(timeout=10.)
            report['passed'] = True
        finally:
            if network is not None:network.close()
            if process is not None and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5.)
            (folder/QTENV_FOLLOW_CHECK_SUMMARY_RELATIVE_FILE).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'],results=str(folder)),ensure_ascii=False))


if __name__=='__main__':main()
