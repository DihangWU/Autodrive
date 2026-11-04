"""按实际就绪状态逐项启动；复用已运行的 CARLA/Town04。"""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from vanet.project_paths import (
    PROJECT_DIRECTORY,PROJECT_SETUP_FILE,CARLA_PYTHON_EXECUTABLE_FILE,SUMO_INSTALLATION_DIRECTORY,
    SUMO_TOWN04_NETWORK_FILE,SUMO_TOWN04_CONFIGURATION_FILE,VANET_LAUNCH_LOCK_FILE,
    EXPERIMENT_ACTIVE_SESSION_FILE,SCENARIO_TOWN04_CONFIGURATION_FILE)
from vanet.experiments.metrics_and_event_recording.event_log import EventLog,new_result_directory


def wait_until(check,log,name,timeout=90):
    deadline=time.monotonic()+timeout;last=None
    while time.monotonic()<deadline:
        try:
            result=check()
            if result:
                log.emit('component_ready',component_name=name,result=result);return result
        except Exception as error:
            if str(error)!=last:
                last=str(error);log.emit('readiness_wait',level='debug',component_name=name,reason=last)
        time.sleep(.25)
    raise RuntimeError(f'{name} 就绪超时：{last}')


def carla_state():
    import carla
    client=carla.Client('127.0.0.1',2000);client.set_timeout(10)
    world=client.get_world()
    return dict(map=world.get_map().name,world_id=world.id)


def bridge_status(vanet=False):
    from foundation.cobridge.control import ControlClient
    client=ControlClient(timeout=1)
    try:return client.command('vanet.status' if vanet else 'status')
    finally:client.close()


def sumo_status():
    from foundation.sumo.client import SumoClient
    client=SumoClient(timeout=1)
    try:return client.status()
    finally:client.close()


def start_terminal(name,command,folder,log,no_terminals=False):
    process_args=[str(CARLA_PYTHON_EXECUTABLE_FILE),'-m','vanet.experiments.experiment_session_management.terminal_process',
                  '--folder',str(folder/'logs'),'--name',name,'--',*command]
    if not no_terminals:
        terminal=shutil.which('gnome-terminal')
        if terminal:process_args=[terminal,'--title='+name,'--',*process_args]
        elif shutil.which('xterm'):process_args=[shutil.which('xterm'),'-T',name,'-e',*process_args]
        else:raise RuntimeError('找不到终端程序；安装 gnome-terminal 或使用 --no-terminals。')
    log.emit('component_launch',component_name=name,command=command)
    return subprocess.Popen(process_args,cwd=PROJECT_DIRECTORY,env=dict(os.environ),start_new_session=True)


def run(args):
    folder=new_result_directory('launch');log=EventLog(folder,'launcher')
    os.environ['AUTODRIVE_VANET']='1'
    os.environ['VANET_SESSION_LOG_DIRECTORY']=str(folder/'logs')
    os.environ.setdefault('SUMO_HOME',str(SUMO_INSTALLATION_DIRECTORY))
    os.environ['PYTHONPATH']=str(PROJECT_DIRECTORY)+os.pathsep+os.environ.get('PYTHONPATH','')
    scenario=json.loads(SCENARIO_TOWN04_CONFIGURATION_FILE.read_text())
    log.save_config(dict(scenario=scenario,commands='setup.bash',no_terminals=args.no_terminals))
    def launch(name,command):return start_terminal(name,command,folder,log,args.no_terminals)
    try:
        log.emit('startup_step',step=1,purpose='CARLA connection')
        try:state=carla_state();log.emit('carla_reused',state=state)
        except Exception:
            try:
                with socket.create_connection(('127.0.0.1',2000),timeout=1):pass
                log.emit('carla_waiting',reason='RPC port already in use; waiting for the existing simulator.')
            except OSError:launch('CARLA',['run_carla'])
            state=wait_until(carla_state,log,'CARLA',120)
        if not state['map'].endswith('/Town04'):
            subprocess.run(['bash','-c','source "$1"; choose_town04','vanet',str(PROJECT_SETUP_FILE)],check=True)
        log.emit('startup_step',step=2,purpose='Town04 network')
        from foundation.cobridge.network import read_network
        import carla
        client=carla.Client('127.0.0.1',2000);client.set_timeout(5)
        fingerprint=hashlib.sha256(client.get_world().get_map().to_opendrive().strip().encode()).hexdigest()
        try:valid=read_network(str(SUMO_TOWN04_CONFIGURATION_FILE))['xodr_sha256']==fingerprint
        except Exception:valid=False
        if not valid:
            subprocess.run(['bash','-c','source "$1"; map_carla2sumo','vanet',str(PROJECT_SETUP_FILE)],check=True)
        log.emit('startup_step',step=3,purpose='Build SUMO experiment GUI and wireless network')
        from foundation.sumo.init.gui.build_sumo_gui import ensure_sumo_gui
        from vanet.omnet.build_and_launch.build_network import build_network
        ensure_sumo_gui();build_network()
        log.emit('startup_step',step=4,purpose='SUMO')
        try:existing=sumo_status()
        except Exception:existing=None
        if existing:raise RuntimeError('已有 SUMO 运行。请先退出该 SUMO，再用 run_vanet 启动带实验操作栏的版本。CARLA 可继续保留。')
        launch('SUMO',['run_sumo',str(SUMO_TOWN04_CONFIGURATION_FILE)])
        wait_until(sumo_status,log,'SUMO')
        log.emit('startup_step',step=5,purpose='CoBridge')
        launch('CoBridge',['run_cobridge','--vanet','--timeout','10'])
        wait_until(lambda:bridge_status().get('sumo_connected'),log,'CoBridge')
        log.emit('startup_step',step=6,purpose='OMNeT++ IEEE 802.11p')
        launch('OMNeT++',['run_omnet','--ui','Qtenv','--results',str(folder/'omnet'),'--seed',str(scenario['seed'])])
        wait_until(lambda:bridge_status(True).get('network',{}).get('ready'),log,'OMNeT++')
        log.emit('startup_step',step=7,purpose='RSU')
        launch('RSU',['run_rsu'])
        wait_until(lambda:bool(bridge_status().get('modules')),log,'RSU')
        log.emit('startup_step',step=8,purpose='One Lincoln without sensors')
        if bridge_status().get('lincolns'):raise RuntimeError('已有 Lincoln 登记，请退出后再启动固定实验车辆。')
        pose=scenario['lincoln']['spawn']
        launch('Lincoln',['add_lincoln','--vanet','--no-sensors','--pose',*[str(pose[k]) for k in ('x','y','z','yaw')]])
        wait_until(lambda:len(bridge_status().get('lincolns',[]))==1,log,'Lincoln')
        log.emit('startup_step',step=9,purpose='Fixed experiment NPCs')
        from foundation.cobridge.control import ControlClient
        def initialize():
            client=ControlClient(timeout=2)
            try:return client.command('vanet.scene.initialize')
            finally:client.close()
        wait_until(initialize,log,'Fixed NPC setup')
        wait_until(lambda:bridge_status(True).get('phase')=='ready',log,'Experiment scene')
        EXPERIMENT_ACTIVE_SESSION_FILE.parent.mkdir(parents=True,exist_ok=True)
        EXPERIMENT_ACTIVE_SESSION_FILE.write_text(json.dumps(dict(results=str(folder),status='ready'),indent=2))
        log.emit('startup_complete',results=str(folder),instruction='Use SUMO: select group → Reset scene → Prepare / Enable → Start experiment.')
    except BaseException as error:
        log.logger.exception('Startup failed')
        log.emit('startup_failed',level='error',reason=str(error),results=str(folder))
        raise
    finally:log.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-terminals',action='store_true',help='诊断模式：后台进程，文件保留完整输出')
    args=parser.parse_args()
    VANET_LAUNCH_LOCK_FILE.parent.mkdir(parents=True,exist_ok=True)
    with VANET_LAUNCH_LOCK_FILE.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run(args)

if __name__=='__main__':main()
