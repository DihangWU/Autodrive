"""Local manual lifecycle transport; simulator operations stay in CoBridge."""
from pathlib import Path
from foundation.ipc import JsonServer, JsonClient, DEFAULT_TIMEOUT

PROJECT_DIR = Path(__file__).resolve().parents[2]
SOCKET_FILE = PROJECT_DIR / '.runtime' / 'cobridge' / 'control.sock'


class ControlServer(JsonServer):
    def __init__(self, path=SOCKET_FILE):
        super().__init__(path)


class ControlClient(JsonClient):
    def __init__(self, path=SOCKET_FILE, timeout=DEFAULT_TIMEOUT):
        try:
            super().__init__(path, timeout)
        except ConnectionError as error:
            raise RuntimeError('无法连接 CoBridge，请先启动 run_cobridge；更新代码后需重启旧进程。') from error
