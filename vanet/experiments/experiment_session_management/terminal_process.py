"""每个独立终端的前台程序：记录完整输出、命令、PID 和退出状态。"""
import argparse
import json
import os
import signal
import subprocess
import sys
from datetime import datetime,timezone
from pathlib import Path
from vanet.project_paths import PROJECT_SETUP_FILE,PROJECT_DIRECTORY


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--name',required=True)
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    args.folder.mkdir(parents=True,exist_ok=True)
    command=args.command
    if command and command[0]=='--':command=command[1:]
    if not command:raise ValueError('缺少启动命令')
    # 参数通过 argv 传递；不把用户值插入 shell 程序文本。
    shell=['bash','-c','source "$1"; shift; "$@"','vanet',str(PROJECT_SETUP_FILE),*command]
    process=subprocess.Popen(shell,cwd=PROJECT_DIRECTORY,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                             text=True,bufsize=1,start_new_session=True)
    identity=dict(pid=process.pid,command=command,started=datetime.now(timezone.utc).isoformat())
    (args.folder/(args.name+'.process.json')).write_text(json.dumps(identity,indent=2))
    def stop(*_):
        if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,stop)
    with (args.folder/(args.name+'.stdout.log')).open('a',buffering=1) as log:
        print(f'[{args.name}] PID={process.pid} COMMAND={command}',flush=True)
        try:
            for line in process.stdout:
                log.write(datetime.now(timezone.utc).isoformat()+' '+line)
                print(line,end='',flush=True)
            code=process.wait()
        finally:
            stop()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    identity['exit_code']=process.returncode
    (args.folder/(args.name+'.process.json')).write_text(json.dumps(identity,indent=2))
    print(f'[{args.name}] EXIT={process.returncode}; logs: {args.folder}',flush=True)
    return code

if __name__=='__main__':raise SystemExit(main())
