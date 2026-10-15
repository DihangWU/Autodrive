"""其他模块通过这里请求 SUMO 管理端，不直接建立第二条 TraCI 连接。"""

import json
import os
from pathlib import Path
import socket


PROJECT_DIR = Path(__file__).resolve().parents[3]
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'sumo'
SOCKET_FILE = RUNTIME_DIR / 'control.sock'
# CARLA 和 SUMO 共用的仿真步长，单位为秒；不表示每帧必须等待这么久。
STEP_LENGTH = 0.05


def process_identity():
    """记录进程启动时刻，避免 PID 被复用后误认为原进程仍在。"""
    pid = os.getpid()
    started = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    return {'pid': pid, 'started': started}


def owner_alive(owner):
    try:
        state = Path(f'/proc/{owner["pid"]}/stat').read_text().rsplit(')', 1)[1].split()
        return state[0] != 'Z' and state[19] == owner['started']
    except (OSError, KeyError):
        return False


def request(command, timeout=5):
    """发送一条 JSON 请求，接收一条 JSON 回复。接口仅在本机使用。"""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(str(SOCKET_FILE))
            connection.sendall((json.dumps(command) + '\n').encode())
            with connection.makefile('rb') as stream:
                response = json.loads(stream.readline(262144))
    except (OSError, ValueError) as error:
        raise RuntimeError('SUMO 管理端未响应，请先运行 run_sumo；详情查看 .runtime/sumo/service.log。') from error
    if not response['ok']:
        raise RuntimeError(response['error'])
    return response['result']
