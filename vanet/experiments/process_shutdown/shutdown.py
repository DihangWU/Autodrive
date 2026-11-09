"""关闭本工程的联仿进程及终端，保留 CARLA、调用终端及无关进程。"""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import math
import os
import signal
import time
from vanet.project_paths import VANET_SHUTDOWN_LOCK_FILE,VANET_LAUNCH_LOCK_FILE,EXPERIMENT_ACTIVE_SESSION_FILE
from vanet.experiments.metrics_and_event_recording.event_log import EventLog,new_result_directory
from .process_inventory import read_process,scan_processes,discover,is_carla,descendants

STOP_ORDER=('launcher','Lincoln','NPC','RSU','Camera','CoBridge','OMNeT++','SUMO')


def alive(process):
    current=read_process(process.pid)
    return current is not None and current.started==process.started


def send_signal(process,signum,protected=()):
    if process.pid in protected:return False
    descriptor=None
    try:
        # pidfd pins the original process even if a PID is recycled after the
        # identity check; no process-group or terminal-server signals are used.
        if hasattr(os,'pidfd_open') and hasattr(signal,'pidfd_send_signal'):
            descriptor=os.pidfd_open(process.pid)
        current=read_process(process.pid)
        if current is None or current.started!=process.started or is_carla(current):return False
        if descriptor is not None:signal.pidfd_send_signal(descriptor,signum)
        else:os.kill(process.pid,signum)
        return True
    except ProcessLookupError:return False
    finally:
        if descriptor is not None:os.close(descriptor)


def wait_for_exit(processes,timeout):
    deadline=time.monotonic()+timeout
    remaining=[process for process in processes if alive(process)]
    while remaining and time.monotonic()<deadline:
        time.sleep(.05)
        remaining=[process for process in remaining if alive(process)]
    return remaining


def terminate(processes,log,protected,grace=8.,first_signal=signal.SIGINT):
    remaining=list(processes)
    for signum,timeout in ((first_signal,grace),(signal.SIGTERM,2.),(signal.SIGKILL,1.)):
        for process in remaining:
            try:
                if send_signal(process,signum,protected):
                    log.emit('process_signal',pid=process.pid,signal=signal.Signals(signum).name,
                             forced=signum==signal.SIGKILL)
            except OSError as error:
                log.emit('process_signal_failed',level='error',pid=process.pid,reason=str(error))
        remaining=wait_for_exit(remaining,timeout)
        if not remaining:break
    return remaining


def stop_component(component,owned,protected,log,timeout):
    members=[(process,kind) for process,name,kind in owned.values() if name==component and alive(process)]
    if not members:return
    log.emit('component_stop',component_name=component,pids=[p.pid for p,_ in members])
    primary=[p for p,kind in members if kind=='entry']
    if not primary:primary=[p for p,kind in members if kind=='helper']
    if primary:terminate(primary,log,protected,timeout)
    # Parent handlers get the first chance to flush logs and clean actors. Only
    # then terminate any children (including RViz in separate sessions) left over.
    remaining=[p for p,kind in members if kind!='terminal' and alive(p)]
    if remaining:terminate(remaining,log,protected,2.,signal.SIGTERM)
    terminals=[p for p,kind in members if kind=='terminal']
    terminals=wait_for_exit(terminals,1.)
    if terminals:terminate(terminals,log,protected,1.,signal.SIGTERM)


def preview():
    owned,shells,_=discover()
    return dict(carla_preserved=True,calling_terminal_preserved=True,
        components={name:dict(pids=sorted(p.pid for p,component,_ in owned.values() if component==name),
                             terminal_pids=sorted(p.pid for p,component,kind in owned.values() if component==name and kind=='terminal'))
                    for name in STOP_ORDER if any(component==name for _,component,_ in owned.values())},
        manual_terminal_shells=sorted(shells))


def shutdown(timeout):
    folder=new_result_directory('shutdown');log=EventLog(folder,'shutdown')
    try:
        owned,shells,protected=discover()
        log.emit('shutdown_begin',carla_preserved=True,results=str(folder))
        stop_component('launcher',owned,protected,log,timeout)
        VANET_LAUNCH_LOCK_FILE.parent.mkdir(parents=True,exist_ok=True)
        with VANET_LAUNCH_LOCK_FILE.open('a') as launch_lock:
            # Prevent another managed launch from racing this shutdown.
            fcntl.flock(launch_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            fresh,new_shells,new_protected=discover()
            owned.update(fresh);shells.update(new_shells);protected.update(new_protected)
            for component in STOP_ORDER[1:]:stop_component(component,owned,protected,log,timeout)
            residual=[p for p,_,_ in owned.values() if alive(p)]
            if residual:terminate(residual,log,protected,2.,signal.SIGTERM)
            # Close manual project tabs only after all their jobs have stopped.
            # A tab with another background job or CARLA is left open.
            processes=scan_processes()
            _,_,new_protected=discover(processes)
            protected.update(new_protected)
            for shell in shells.values():
                if not alive(shell) or shell.pid in protected:continue
                if descendants({shell.pid},processes)-{shell.pid}:
                    log.emit('terminal_preserved',pid=shell.pid,reason='Other jobs remain in this terminal.')
                else:terminate([shell],log,protected,1.,signal.SIGHUP)
            remaining=[p.pid for p,_,_ in owned.values() if alive(p)]
            if remaining:
                log.emit('shutdown_incomplete',level='error',pids=remaining)
                return 1
            if EXPERIMENT_ACTIVE_SESSION_FILE.exists():
                state=json.loads(EXPERIMENT_ACTIVE_SESSION_FILE.read_text())
                state.update(status='stopped',closed_at=datetime.now(timezone.utc).isoformat())
                EXPERIMENT_ACTIVE_SESSION_FILE.write_text(json.dumps(state,indent=2)+'\n')
            log.emit('shutdown_complete',carla_preserved=True,results=str(folder))
            return 0
    finally:log.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true',help='只列出将关闭的项目进程，不发送信号')
    parser.add_argument('--timeout',type=float,default=8.,help='每个模块正常退出的等待秒数，默认 8')
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<0:parser.error('--timeout 必须为非负有限数值')
    if args.dry_run:
        print(json.dumps(preview(),ensure_ascii=False,indent=2));return 0
    VANET_SHUTDOWN_LOCK_FILE.parent.mkdir(parents=True,exist_ok=True)
    with VANET_SHUTDOWN_LOCK_FILE.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return shutdown(args.timeout)


if __name__=='__main__':raise SystemExit(main())
