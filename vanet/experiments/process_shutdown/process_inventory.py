"""按项目入口和进程树识别归属；不通过进程名批量杀终端或 Python。"""
from dataclasses import dataclass
import os
from pathlib import Path
from vanet.project_paths import (PROJECT_DIRECTORY,EXPERIMENT_RESULTS_DIRECTORY,
    SHUTDOWN_PROCESS_DIRECTORY,SHUTDOWN_PROCESS_STAT_RELATIVE_FILE,
    SHUTDOWN_PROCESS_COMMAND_RELATIVE_FILE,SHUTDOWN_PROCESS_WORKING_DIRECTORY,
    SHUTDOWN_CARLA_ENTRY_FILE,SHUTDOWN_LINCOLN_RUNTIME_DIRECTORY,
    SHUTDOWN_SUMO_GUI_FILE,SHUTDOWN_SUMO_MAP_DIRECTORY,SHUTDOWN_PROJECT_ENTRY_FILES,OMNET_WIRELESS_LIBRARY_FILE)

TERMINAL_MODULE='vanet.experiments.experiment_session_management.terminal_process'
MODULE_COMPONENTS={
    'vanet.experiments.experiment_session_management.launcher':'launcher',
    'vanet.omnet.build_and_launch.run_network':'OMNeT++',
}


@dataclass(frozen=True)
class Process:
    pid:int
    parent:int
    started:int
    tty:int
    args:tuple
    cwd:Path


def read_process(pid):
    directory=SHUTDOWN_PROCESS_DIRECTORY/str(pid)
    try:
        if directory.stat().st_uid!=os.getuid():return None
        fields=(directory/SHUTDOWN_PROCESS_STAT_RELATIVE_FILE).read_text().rsplit(')',1)[1].split()
        if fields[0] in ('Z','X'):return None
        args=tuple(os.fsdecode(arg) for arg in (directory/SHUTDOWN_PROCESS_COMMAND_RELATIVE_FILE).read_bytes().split(b'\0') if arg)
        if not args:return None
        cwd=(directory/SHUTDOWN_PROCESS_WORKING_DIRECTORY).resolve(strict=True)
        return Process(pid,int(fields[1]),int(fields[19]),int(fields[4]),args,cwd)
    except (OSError,ValueError,IndexError):return None


def scan_processes():
    processes={}
    for directory in SHUTDOWN_PROCESS_DIRECTORY.iterdir():
        if directory.name.isdecimal():
            process=read_process(int(directory.name))
            if process:processes[process.pid]=process
    return processes


def under(path,root):
    try:Path(path).resolve().relative_to(root.resolve());return True
    except (ValueError,OSError):return False


def argument_value(args,option):
    try:return args[args.index(option)+1]
    except (ValueError,IndexError):return None


def module_name(process):
    if not Path(process.args[0]).name.startswith('python'):return None
    return argument_value(process.args,'-m')


def script_file(process):
    if not Path(process.args[0]).name.startswith(('python','bash','sh')):return None
    for index,arg in enumerate(process.args[1:],1):
        if arg in ('-c','-m') or (arg.startswith('-') and 'c' in arg[1:] and Path(process.args[0]).name in ('bash','sh')):return None
        if arg.startswith('-'):continue
        try:return (process.cwd/arg).resolve()
        except OSError:return None
    return None


def is_carla(process):
    if Path(process.args[0]).name.startswith('CarlaUE4'):return True
    if script_file(process)==SHUTDOWN_CARLA_ENTRY_FILE:return True
    return module_name(process)==TERMINAL_MODULE and argument_value(process.args,'--name')=='CARLA'


def classify(process):
    if is_carla(process):return ('CARLA','protected')
    module=module_name(process)
    if module==TERMINAL_MODULE:
        folder=argument_value(process.args,'--folder')
        name=argument_value(process.args,'--name')
        if folder and under(folder,EXPERIMENT_RESULTS_DIRECTORY) and name in (*SHUTDOWN_PROJECT_ENTRY_FILES,'OMNeT++'):
            return (name,'terminal')
    if module in MODULE_COMPONENTS and under(process.cwd,PROJECT_DIRECTORY):
        return (MODULE_COMPONENTS[module],'entry')
    script=script_file(process)
    for component,files in SHUTDOWN_PROJECT_ENTRY_FILES.items():
        if script in files:return (component,'entry')
    executable=Path(process.args[0]).name
    if executable=='opp_run' and str(OMNET_WIRELESS_LIBRARY_FILE) in process.args:return ('OMNeT++','helper')
    if executable=='rviz2' and any(under(arg,SHUTDOWN_LINCOLN_RUNTIME_DIRECTORY) for arg in process.args[1:] if arg.startswith('/')):
        return ('Lincoln','helper')
    if executable in ('sumo','sumo-gui') and (Path(process.args[0])==SHUTDOWN_SUMO_GUI_FILE or
        any(under(arg,SHUTDOWN_SUMO_MAP_DIRECTORY) for arg in process.args[1:] if arg.startswith('/'))):
        return ('SUMO','helper')
    return None


def ancestors(pid,processes):
    result=set()
    while pid in processes and pid not in result:
        result.add(pid);pid=processes[pid].parent
    return result


def descendants(roots,processes):
    result=set(roots)
    while True:
        added={p.pid for p in processes.values() if p.parent in result}-result
        if not added:return result
        result.update(added)


def discover(processes=None,caller_pid=None):
    processes=scan_processes() if processes is None else processes
    caller_pid=os.getpid() if caller_pid is None else caller_pid
    carla={p.pid for p in processes.values() if is_carla(p)}
    protected=ancestors(caller_pid,processes)|descendants(carla,processes)
    for pid in carla:protected.update(ancestors(pid,processes))
    classified={pid:classify(p) for pid,p in processes.items()}
    roots={pid:role for pid,role in classified.items() if role and role[0]!='CARLA' and pid not in protected}
    # A network launched manually from another directory is still ours if its
    # direct child loads this project's wireless library.
    for pid,role in list(roots.items()):
        parent=processes.get(processes[pid].parent)
        if parent and module_name(parent) in MODULE_COMPONENTS and parent.pid not in protected:
            roots[parent.pid]=(MODULE_COMPONENTS[module_name(parent)],'entry')
    owned={}
    for pid,role in roots.items():
        for child in descendants({pid},processes)-protected:
            owned.setdefault(child,(processes[child],role[0],'descendant'))
    for pid,(component,kind) in roots.items():owned[pid]=(processes[pid],component,kind)
    shells={}
    for pid in roots:
        parent_pid=processes[pid].parent;visited=set()
        while parent_pid in processes and parent_pid not in visited:
            visited.add(parent_pid)
            parent=processes[parent_pid]
            executable=Path(parent.args[0]).name.lstrip('-')
            if (parent_pid not in protected and parent_pid not in owned and parent.tty and
                executable in ('bash','zsh','fish') and all(a in ('-i','-l','--login') for a in parent.args[1:])):
                shells[parent_pid]=parent;break
            parent_pid=parent.parent
    return owned,shells,protected
