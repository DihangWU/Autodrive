"""Nonblocking Unix transport for an externally clocked OMNeT++ process."""
from collections import deque
import errno
import json
import math
import select
import socket
import time
from pathlib import Path
from vanet.project_paths import OMNET_COBRIDGE_COMMUNICATION_SOCKET_FILE, EXPERIMENT_RESULTS_DIRECTORY

MAX_MESSAGE_BYTES = 8 * 1024 * 1024


class OmnetConnection:
    def __init__(self, socket_path=None, timeout=10.0):
        self.path = socket_path or OMNET_COBRIDGE_COMMUNICATION_SOCKET_FILE
        self.timeout = timeout
        self.socket = None
        self.connected = False
        self.ready = False
        self.error = None
        self.network_time = 0.0
        self.capabilities = {}
        self.output = bytearray()
        self.input = bytearray()
        self.results = deque()
        self.pending = None
        self.deadline = None
        self.connecting = False
        self.reset_pending = False

    @property
    def status(self):
        return dict(connected=self.connected, ready=self.ready, error=self.error, capabilities=dict(self.capabilities),
                    time=self.network_time, pending_step=self.pending['step_id'] if self.pending else None, reset_pending=self.reset_pending)

    def start(self):
        if self.socket is not None:
            return self.status
        self.error = None
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.setblocking(False)
        result = self.socket.connect_ex(str(self.path))
        if result not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EAGAIN):
            self._fail(OSError(result, errno.errorcode.get(result, 'connect failed')))
            return self.status
        self.connecting = result != 0
        self.connected = not self.connecting
        self.deadline = time.monotonic() + self.timeout
        self._queue({'action': 'hello', 'protocol': 1})
        return self.status

    def _queue(self, command):
        encoded = (json.dumps(command, separators=(',', ':'), allow_nan=False) + '\n').encode()
        if len(encoded) > MAX_MESSAGE_BYTES:
            raise ValueError('Network command exceeds 8 MiB limit')
        self.output.extend(encoded)

    def submit(self, step_id, until, nodes, transmissions):
        if not self.ready or self.error:
            raise ConnectionError(self.error or 'OMNeT++ handshake is not ready')
        if self.pending:
            raise RuntimeError('A network advance is already pending')
        if isinstance(step_id, bool) or not isinstance(step_id, int):
            raise ValueError('step_id must be an integer')
        if not math.isfinite(until) or not self.network_time < until <= self.network_time + 1.0 + 1e-9:
            raise ValueError('Network advance must increase time by at most one second')
        command = dict(action='advance', step_id=step_id, until=until, nodes=nodes, transmissions=transmissions)
        self._queue(command)
        self.pending = dict(step_id=step_id, until=until)
        self.deadline = time.monotonic() + self.timeout

    def poll(self):
        if self.socket is None:
            return self.status
        try:
            readable, writable, exceptional = select.select([self.socket], [self.socket] if self.output or self.connecting else [], [self.socket], 0)
            if exceptional:
                raise ConnectionError('OMNeT++ socket failure')
            if self.connecting and writable:
                error = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                if error:
                    raise OSError(error, 'OMNeT++ connection failed')
                self.connecting = False
                self.connected = True
            if writable and self.output and not self.connecting:
                count = self.socket.send(self.output)
                del self.output[:count]
            if readable:
                data = self.socket.recv(65536)
                if not data:
                    raise ConnectionError('OMNeT++ disconnected; pause the experiment')
                self.input.extend(data)
                if len(self.input) > MAX_MESSAGE_BYTES:
                    raise ValueError('Network response exceeds 8 MiB limit')
                while b'\n' in self.input:
                    line, _, remaining = self.input.partition(b'\n')
                    self.input = bytearray(remaining)
                    self._receive(json.loads(line))
            if self.deadline is not None and time.monotonic() > self.deadline:
                raise TimeoutError('OMNeT++ response timed out; pause the experiment')
        except (OSError, ValueError, KeyError, TypeError) as error:
            self._fail(error)
        return self.status

    def _receive(self, response):
        if not response.get('ok'):
            raise ValueError('OMNeT++ rejected command: ' + str(response.get('error')))
        if 'protocol' in response:
            if response['protocol'] != 1:
                raise ValueError('Unsupported OMNeT++ protocol version')
            self.ready = True
            self.capabilities = dict(response.get('capabilities', {}))
            self.network_time = float(response['time'])
            self.deadline = None
            return
        if response.get('reset'):
            if not self.reset_pending:
                raise ValueError('Unexpected reset acknowledgement')
            self.network_time = float(response['time'])
            self.reset_pending = False
            self.ready = True
            self.deadline = None
            self.results.append(response)
            return
        if response.get('stopped'):
            self.results.append(response)
            self.deadline = None
            return
        if self.pending is None or response.get('step_id') != self.pending['step_id']:
            raise ValueError('Unexpected network result step_id')
        value = float(response['time'])
        if not math.isfinite(value) or abs(value - self.pending['until']) > 1e-8:
            raise ValueError('Network returned an incorrect time boundary')
        for delivery in response.get('deliveries', []):
            sent, received = float(delivery['sent_at']), float(delivery['received_at'])
            if not math.isfinite(sent + received) or sent > received or received > value + 1e-9:
                raise ValueError('Network delivery violates causal time bounds')
        self.network_time = value
        self.pending = None
        self.deadline = None
        self.results.append(response)

    def take_result(self):
        return self.results.popleft() if self.results else None

    def reset(self, seed, results_directory=None):
        if not self.ready or self.pending or self.reset_pending:
            raise RuntimeError('Network must be idle and ready before resetting')
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= 2147483647:
            raise ValueError('seed must be an integer in 0..2147483647')
        command = {'action': 'reset', 'seed': seed}
        if results_directory is not None:
            directory = Path(results_directory).resolve()
            root = EXPERIMENT_RESULTS_DIRECTORY.resolve()
            if directory == root or not directory.is_relative_to(root):
                raise ValueError('results_directory must be a subdirectory of vanet/results')
            for name in ('wireless_events.jsonl', 'network_steps.jsonl', 'network_manifest.json'):
                if not (directory / name).resolve().is_relative_to(root):
                    raise ValueError('Result file resolves outside vanet/results')
            if not self.capabilities.get('per_run_archives'):
                raise RuntimeError('Restart OMNeT++ before using per-run wireless archives')
            command['results_directory'] = str(directory)
        self._queue(command)
        self.reset_pending = True
        self.ready = False
        self.deadline = time.monotonic() + self.timeout

    def request_shutdown(self):
        if self.pending:
            raise RuntimeError('Wait for pending network advance before shutdown')
        self._queue({'action': 'shutdown'})
        self.deadline = time.monotonic() + self.timeout

    def _fail(self, error):
        self.error = str(error)
        self.close()

    def close(self):
        if self.socket is not None:
            self.socket.close()
        self.socket = None
        self.connected = self.ready = self.connecting = False
        self.output.clear()
        self.input.clear()
        self.deadline = None
