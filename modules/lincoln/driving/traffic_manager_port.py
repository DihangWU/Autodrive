"""每辆 Lincoln 的 TM 服务拥有独立端口，退出一辆车不带走其他车的 TM。"""
from contextlib import contextmanager
import fcntl
import socket
from modules.lincoln.runtime.paths import RUNTIME_DIR

TM_PORT_LOCK_FILE = RUNTIME_DIR / 'traffic-manager-port.lock'
LOCAL_BIND_HOST = '0.0.0.0'
MAX_PORT = 65535


@contextmanager
def reserve_traffic_manager_port(preferred):
    """持有模块间锁直至官方 get_trafficmanager 完成，避免同时选中同一端口。"""
    TM_PORT_LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    with TM_PORT_LOCK_FILE.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for port in range(preferred, MAX_PORT + 1):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                try:
                    probe.bind((LOCAL_BIND_HOST, port))
                except OSError:
                    continue
            yield port
            return
    raise RuntimeError('没有可用的 Traffic Manager 端口。')
