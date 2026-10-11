"""Local control requests for the sole CARLA/SUMO tick owner.

Request files are atomic; only the bridge thread calls TraCI. Clients own NPC
sessions, not the service. Private per-user directories isolate control files.
"""
import hashlib
import json
import os
from pathlib import Path
import time
import uuid


def control_folder(host, port):
    host = 'localhost' if host in ('localhost', '127.0.0.1') else host
    key = hashlib.sha256(f'{os.getuid()}:{host}:{port}'.encode()).hexdigest()[:16]
    folder = Path('/tmp') / ('carla-lincoln-' + key)
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
    if owner is None:
        return True  # Bridge-owned persistent batch.
    return identity(owner['pid']) == owner


def atomic_json(path, value):
    temp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temp.write_text(json.dumps(value))
    os.replace(temp, path)


def request(host, port, payload, timeout=90):
    folder = control_folder(host, port)
    token = uuid.uuid4().hex
    command = folder / ('request-' + token + '.json')
    reply = folder / ('reply-' + token + '.json')
    payload = dict(payload, requester=identity())
    atomic_json(command, payload)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if reply.exists():
                value = json.loads(reply.read_text())
                if not value.get('ok'):
                    raise RuntimeError(value.get('error', 'SUMO request failed'))
                return value
            time.sleep(.1)
        raise RuntimeError('Shared SUMO bridge did not answer; check sumo.log and GUI playback.')
    finally:
        command.unlink(missing_ok=True)
        reply.unlink(missing_ok=True)


class RequestService:
    """Executed synchronously by the bridge, never from a TraCI client thread."""
    def __init__(self, host, port, spawn, remove, status):
        self.folder = control_folder(host, port)
        self.spawn, self.remove, self.status = spawn, remove, status
        self.sessions = {}
        self.last_check = 0.

    def process(self):
        for path in sorted(self.folder.glob('request-*.json')):
            reply = path.with_name(path.name.replace('request-', 'reply-', 1))
            try:
                command = json.loads(path.read_text())
                if not alive(command.get('requester')):
                    continue
                action = command['action']
                session = command.get('session')
                if action == 'add':
                    if session in self.sessions:
                        raise ValueError('NPC session already exists')
                    vehicles = self.spawn(session, command)
                    self.sessions[session] = {'owner': command['requester'], 'vehicles': vehicles}
                    result = {'session': session, 'vehicles': vehicles}
                elif action == 'remove':
                    batch = self.sessions.get(session)
                    if batch and batch['owner'] != command['requester']:
                        raise ValueError('NPC session belongs to another process')
                    if batch:
                        self.remove(batch['vehicles'])
                        del self.sessions[session]
                    result = {'session': session}
                elif action == 'status':
                    result = self.status()
                else:
                    raise ValueError('Unknown bridge action')
                atomic_json(reply, dict(ok=True, **result))
            except Exception as error:
                atomic_json(reply, {'ok': False, 'error': str(error)})
            finally:
                path.unlink(missing_ok=True)
        now = time.monotonic()
        if now - self.last_check >= 1.:
            self.last_check = now
            for session, batch in list(self.sessions.items()):
                if not alive(batch['owner']):
                    self.remove(batch['vehicles'])
                    del self.sessions[session]
            for reply in self.folder.glob('reply-*.json'):
                if time.time() - reply.stat().st_mtime > 120:
                    reply.unlink(missing_ok=True)
