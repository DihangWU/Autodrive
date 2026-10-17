"""SUMO runtime control; closing this client never closes SUMO."""
from pathlib import Path
from foundation.ipc import JsonClient
from foundation.sumo.traci.remote import RemoteConnection

PROJECT_DIR = Path(__file__).resolve().parents[2]
SOCKET_FILE = PROJECT_DIR / '.runtime' / 'sumo' / 'control.sock'
DEFAULT_TIMEOUT = 10.0


class SumoClient(JsonClient):
    def __init__(self, path=SOCKET_FILE, timeout=DEFAULT_TIMEOUT):
        super().__init__(path, timeout)
        self.connection = RemoteConnection(self)

    def status(self):
        return self.command('status')

    def claim_clock(self, step_length):
        return self.command('clock.claim', step_length=step_length)

    def release_clock(self):
        return self.command('clock.release')

    def poll_events(self):
        return self.command('events.poll')

    def acknowledge_events(self, ids, dropped=0):
        return self.command('events.ack', ids=ids, dropped=dropped)
