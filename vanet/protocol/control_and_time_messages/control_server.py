"""VANET 控制入口的有界异步收发；大路线不会阻塞控制轮询。"""
import json
import socket
from foundation.ipc import JsonServer

MAX_CONTROL_BYTES = 4 * 1024 * 1024
MAX_PEERS = 32


class VanetControlServer(JsonServer):
    def __init__(self, path):
        super().__init__(path)
        self.outgoing = {}

    def poll(self, handler, disconnected):
        for _ in range(MAX_PEERS):
            try: peer, _ = self.socket.accept()
            except BlockingIOError: break
            if len(self.clients) >= MAX_PEERS:
                peer.close(); continue
            peer.setblocking(False)
            self.clients[peer] = bytearray()
        for peer, incoming in list(self.clients.items()):
            try:
                try: data = peer.recv(65536)
                except BlockingIOError: data = None
                if data == b'':
                    self.drop(peer, disconnected); continue
                if data: incoming.extend(data)
                if len(incoming) > MAX_CONTROL_BYTES:
                    self.drop(peer, disconnected); continue
                if peer not in self.outgoing and b'\n' in incoming:
                    line, _, tail = incoming.partition(b'\n');incoming[:] = tail
                    try: response = dict(ok=True,result=handler(json.loads(line),peer))
                    except Exception as error: response = dict(ok=False,error=str(error))
                    payload = json.dumps(response,ensure_ascii=False,allow_nan=False).encode()+b'\n'
                    if len(payload)>MAX_CONTROL_BYTES:raise ValueError('控制响应过大')
                    self.outgoing[peer] = memoryview(payload)
                if peer in self.outgoing:
                    try: count = peer.send(self.outgoing[peer])
                    except BlockingIOError: continue
                    remaining = self.outgoing[peer][count:]
                    if remaining:self.outgoing[peer]=remaining
                    else:del self.outgoing[peer]
            except (OSError,ValueError):self.drop(peer,disconnected)

    def drop(self, peer, disconnected):
        self.outgoing.pop(peer,None)
        super().drop(peer,disconnected)
