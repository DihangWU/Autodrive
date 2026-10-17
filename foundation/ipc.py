"""JSON messages over local sockets; no simulator or entity policy."""
import json
from pathlib import Path
import socket

DEFAULT_TIMEOUT = 10.0
SOCKET_MODE = 0o600
MAX_MESSAGE_BYTES = 65536
RECEIVE_BYTES = 4096
LISTEN_BACKLOG = 8
MAX_CLIENTS = 16


class JsonServer:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.socket.bind(str(self.path))
            self.path.chmod(SOCKET_MODE)
            self.socket.listen(LISTEN_BACKLOG)
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            self.path.unlink(missing_ok=True)
            raise
        self.clients = {}

    def poll(self, handler, disconnected):
        # Never wait for a request: control messages are processed at frame boundaries.
        for _ in range(MAX_CLIENTS):
            try:
                peer, _ = self.socket.accept()
            except BlockingIOError:
                break
            if len(self.clients) >= MAX_CLIENTS:
                peer.close()
                continue
            peer.setblocking(False)
            self.clients[peer] = bytearray()
        for peer, buffer in list(self.clients.items()):
            try:
                data = peer.recv(RECEIVE_BYTES)
            except BlockingIOError:
                data = None
            except OSError:
                data = b''
            if data == b'' or len(buffer) + len(data or b'') > MAX_MESSAGE_BYTES:
                self.drop(peer, disconnected)
                continue
            if data:
                buffer.extend(data)
            if b'\n' not in buffer:
                continue
            line, _, rest = buffer.partition(b'\n')
            buffer[:] = rest
            try:
                result = handler(json.loads(line), peer)
                payload = json.dumps(dict(ok=True, result=result)).encode() + b'\n'
            except Exception as error:
                payload = json.dumps(dict(ok=False, error=str(error))).encode() + b'\n'
            try:
                peer.sendall(payload)
            except OSError:
                self.drop(peer, disconnected)

    def drop(self, peer, disconnected):
        self.clients.pop(peer, None)
        peer.close()
        disconnected(peer)

    def close(self):
        for peer in self.clients:
            peer.close()
        self.clients.clear()
        self.socket.close()
        self.path.unlink(missing_ok=True)


class JsonClient:
    def __init__(self, path, timeout=DEFAULT_TIMEOUT):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(timeout)
        try:
            self.socket.connect(str(path))
        except OSError as error:
            self.socket.close()
            raise ConnectionError(f'无法连接本地服务 {path}：{error}') from error
        self.stream = self.socket.makefile('rb')

    def command(self, action, **fields):
        self.socket.sendall(json.dumps(dict(action=action, **fields)).encode() + b'\n')
        line = self.stream.readline(MAX_MESSAGE_BYTES + 1)
        if not line:
            raise ConnectionError('本地服务已停止或控制连接已断开。')
        if len(line) > MAX_MESSAGE_BYTES:
            raise RuntimeError('服务回复过大。')
        response = json.loads(line)
        if not response['ok']:
            raise RuntimeError(response['error'])
        return response['result']

    def close(self):
        self.stream.close()
        self.socket.close()
