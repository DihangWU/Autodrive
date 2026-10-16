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
    """获取当前进程的身份信息，返回它的 PID 和启动时刻"""
    # /proc/{PID}/stat 是个文件，储存进程的状态信息，包括启动时刻等等
    pid = os.getpid()  # 获取执行到这一行代码的进程的pid
    started = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
        # f'/proc/{pid}/stat' 拼接路径，Path()转换为路径对象
        # .read_text() 读取路径的文件内容
        # .rsplit(')', 1) # 从右边开始寻找')',最多拆分1次，以 ) 为分隔符切开，) 本身不会保留在任何一份里
        # rsplit()[1] 取第二部分
        # split()[19] 按照 空白符切分，取第20个，第20个固定表示启动时间
    return {'pid': pid, 'started': started}
        # started 最终是一个数字字符串，表示该进程在系统启动后第多少个时钟节拍启动，不是年月日形式的时间。


def owner_alive(owner): # owner本身是一个字典
    """返回申请使用资源的程序的存在与否状态，从而决定是否清理它留下的资源"""
    try:
        state = Path(f'/proc/{owner["pid"]}/stat').read_text().rsplit(')', 1)[1].split()
        return state[0] != 'Z' and state[19] == owner['started']
            # 第一个字母：R=正在被执行；S=睡眠中可苏醒；D=睡眠中不可醒；T=已暂停；Z=僵尸进程，已经结束但是没有收到退出信息
    except (OSError, KeyError):
        return False


def request(command, timeout=5):
    """连接本机的 SUMO 管理服务，发送一条命令行请求，再读取服务返回的结果"""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            # socket.socket(...) as connection 创建一个 socket，用 connection 这个名字操作它；避免覆盖导入的 socket 模块名
			# socket.AF_UNIX：通过本机的 socket 文件路径连接，例如 .runtime/sumo/control.sock，用于同一台机器上的进程通信。
			# socket.SOCK_STREAM：以连续字节流传输，保证数据可靠、有顺序，但不自动划分每条消息的边界。
            connection.settimeout(timeout)  # 设置最高等待时间，单位为秒
            connection.connect(str(SOCKET_FILE))  # 指定连接地址簿的路径
            connection.sendall((json.dumps(command) + '\n').encode())
                # connection.sendall()发送括号内指定的全部字节
                # json.dumps(command) 把 command 转化为 json 格式指令，加上换行符表示指令结束，编码为字节
            with connection.makefile('rb') as stream: # 给 socket 创建一个文件式读取接口，命名为 stream
                    # r=读取，b=按照字节读取，with=代码块结束后自动关闭这个读取接口
                response = json.loads(stream.readline(262144))
                    # stream.readline(262144) 的意思是：从 stream 读取一行数据，最多读取 262144 字节，也就是 256 KiB。
                    # response = json.loads(data)     # 把 JSON 解析成 Python 可解析的数据类型
    except (OSError, ValueError) as error:
        raise RuntimeError('SUMO 管理端未响应，请先运行 run_sumo；详情查看 .runtime/sumo/service.log。') from error

    if not response['ok']: # response 是个字典，可以用 字典[键] 取出对应的值，response 的格式由 service .py 来自定义
        raise RuntimeError(response['error'])
    return response['result']
