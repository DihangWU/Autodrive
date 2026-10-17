"""Official TraCI connection only; no entity, clock or scenario policy."""
import importlib
import os
from pathlib import Path
import sys


# 路径与连接参数：SUMO 安装路径来自环境，不写死本机目录。
SUMO_HOME = Path(os.environ['SUMO_HOME']).expanduser() if os.environ.get('SUMO_HOME') else None
SUMO_TOOLS_DIR = SUMO_HOME / 'tools' if SUMO_HOME else None
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8813
DEFAULT_RETRIES = 10


def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, retries=DEFAULT_RETRIES):
    tools = SUMO_TOOLS_DIR
    if tools and str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    traci = importlib.import_module('traci')
    try:
        return traci.connect(host=host, port=port, numRetries=retries)
    except (OSError, traci.exceptions.FatalTraCIError) as error:
        raise ConnectionError(f'无法连接 TraCI 服务 {host}:{port}：{error}') from error
