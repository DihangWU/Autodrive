"""多个 Lincoln 进程共享同一 spectator；不依赖 SUMO 或 CoBridge。"""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from modules.lincoln.runtime.paths import CAMERA_RUNTIME_DIR, PROC_DIR

CAMERA_STATE_NAME = 'owner.json'
CAMERA_LOCK_NAME = 'owner.lock'


def current_process_identity():
    pid = os.getpid()
    fields = (PROC_DIR / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, started=fields[19])


def process_is_alive(process):
    try:
        fields = (PROC_DIR / str(process['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
        return fields[0] not in ('Z', 'X') and fields[19] == process['started']
    except (OSError, KeyError, ValueError, IndexError):
        return False


class CameraOwnership:
    def __init__(self, host, port, uid):
        endpoint = hashlib.sha256(f'{host}:{port}'.encode()).hexdigest()[:16]
        folder = CAMERA_RUNTIME_DIR / endpoint
        folder.mkdir(parents=True, exist_ok=True)
        self.path = folder / CAMERA_STATE_NAME
        self.lock_path = folder / CAMERA_LOCK_NAME
        self.owner = dict(process=current_process_identity(), uid=uid)

    @contextmanager
    def locked(self):
        with self.lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def read(self):
        try:
            owner = json.loads(self.path.read_text())
            return owner if process_is_alive(owner['process']) else None
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def claim(self, replace=False):
        with self.locked():
            owner = self.read()
            if owner and owner != self.owner and not replace:
                return False
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.owner))
            temporary.replace(self.path)
            return True

    @contextmanager
    def access(self):
        with self.locked():
            yield self.read() == self.owner

    def release(self):
        with self.locked():
            if self.read() == self.owner:
                self.path.unlink(missing_ok=True)
