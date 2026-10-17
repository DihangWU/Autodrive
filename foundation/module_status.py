"""Optional module status reporting; never owns module or simulator lifetime."""
from pathlib import Path
import threading

from foundation.ipc import JsonClient

# 通用通知参数与状态标签；没有具体业务模块的注册表。
PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SOCKET = PROJECT_DIR / '.runtime' / 'cobridge' / 'control.sock'
REPORT_INTERVAL = 1.0
RPC_TIMEOUT = 1.0
CLOSE_TIMEOUT = RPC_TIMEOUT + 0.5
MAX_NAME_LENGTH = 64
MAX_DETAIL_LENGTH = 512
STATE_LABELS = {
    'waiting': '等待依赖',
    'running': '运行中',
    'recovering': '等待恢复',
    'error': '状态异常',
    'stopped': '已停止',
}


class ModuleRegistry:
    def __init__(self, display):
        self.display = display
        self.records = {}

    def update(self, command, owner):
        name, state, detail = command['name'], command['state'], command.get('detail', '')
        if (not isinstance(name, str) or not name or len(name) > MAX_NAME_LENGTH
                or not isinstance(state, str) or state not in STATE_LABELS
                or not isinstance(detail, str) or len(detail) > MAX_DETAIL_LENGTH
                or any(char in name + detail for char in '\r\n')):
            raise ValueError('无效模块状态。')
        record = dict(name=name, state=state, detail=detail)
        if self.records.get(owner) != record:
            self.records[owner] = record
            suffix = f'（{detail}）' if detail else ''
            self.display(f'模块 {name}：{STATE_LABELS[state]}{suffix}')
        return dict(received=True)

    def disconnected(self, owner):
        record = self.records.pop(owner, None)
        if record is not None and record['state'] != 'stopped':
            self.display(f"模块 {record['name']}：状态连接已断开")

    def snapshot(self):
        return list(self.records.values())


class ModuleReporter:
    """Send cached status in a daemon thread; observers may start or exit anytime."""
    def __init__(self, name, path=DEFAULT_SOCKET):
        self.name, self.path = name, path
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.state = ('waiting', '')
        self.closing = False
        self.thread = threading.Thread(target=self._run, name=f'{name}-status', daemon=True)
        self.thread.start()

    def set_state(self, state, detail=''):
        if state not in STATE_LABELS:
            raise ValueError('无效模块状态。')
        detail = str(detail).replace('\r', ' ').replace('\n', ' ')[:MAX_DETAIL_LENGTH]
        with self.lock:
            changed = self.state != (state, detail)
            self.state = (state, detail)
        if changed:
            self.wake.set()

    def _run(self):
        client = None
        try:
            while True:
                self.wake.clear()
                with self.lock:
                    state, detail = self.state
                    closing = self.closing
                try:
                    if client is None:
                        client = JsonClient(self.path, RPC_TIMEOUT)
                    client.command('module.status', name=self.name, state=state, detail=detail)
                except Exception:
                    if client is not None:
                        client.close()
                        client = None
                if closing:
                    return
                self.wake.wait(REPORT_INTERVAL)
        finally:
            if client is not None:
                client.close()

    def close(self):
        with self.lock:
            self.state = ('stopped', '')
            self.closing = True
        self.wake.set()
        self.thread.join(CLOSE_TIMEOUT)
