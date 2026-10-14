"""Exclusive spectator ownership shared by Lincoln processes on one server."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from contextlib import contextmanager
import fcntl
import json
from cobridge.common.ipc import control_folder, identity, alive, atomic_json


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
