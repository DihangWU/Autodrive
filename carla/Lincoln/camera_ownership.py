"""Exclusive spectator ownership shared by Lincoln processes on one server."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import uuid


def control_folder(host, port):
    host = 'localhost' if host in ('localhost', '127.0.0.1') else host
    key = hashlib.sha256(f'{os.getuid()}:{host}:{port}'.encode()).hexdigest()[:16]
    folder = Path(tempfile.gettempdir()) / ('carla-lincoln-' + key)
    folder.mkdir(mode=0o700, exist_ok=True)
    return folder


def identity(pid=None):
    pid = os.getpid() if pid is None else pid
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if stat[0] == 'Z':
            return None
        return {'pid': pid, 'start': stat[19]}
    except (OSError, IndexError):
        return None


def alive(owner):
    return owner is not None and identity(owner['pid']) == owner


def atomic_json(path, value):
    temp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temp.write_text(json.dumps(value))
    os.replace(temp, path)


class CameraOwnership:
    def __init__(self, host, port, uid):
        folder = control_folder(host, port)
        self.path = folder / 'camera-owner.json'
        self.lock_path = folder / 'camera-owner.lock'
        self.owner = {'process': identity(), 'uid': uid}

    @contextmanager
    def locked(self):
        with self.lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def read(self):
        try:
            owner = json.loads(self.path.read_text())
            return owner if alive(owner['process']) else None
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def claim(self, replace=False):
        with self.locked():
            owner = self.read()
            if owner and owner != self.owner and not replace:
                return False
            atomic_json(self.path, self.owner)
            return True

    @contextmanager
    def access(self):
        # Hold the lock through the camera write so a handoff cannot race it.
        with self.locked():
            yield self.read() == self.owner

    def release(self):
        with self.locked():
            if self.read() == self.owner:
                self.path.unlink(missing_ok=True)
