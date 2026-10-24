"""只调用官方 TM 的同步步进接口；不推进 CARLA 世界，不改官方源码。

CARLA 0.9.16 的 TrafficManagerRemote.SynchronousTick() 返回 false，
因此不能只在桥内取得远程 TM 句柄。这里调用 TM 服务端公开的
synchronous_tick RPC。协议是固定范围的 MessagePack-RPC，无额外依赖。
"""
import logging
import socket

TM_RPC_TIMEOUT = 2.0
LOCAL_TM_HOST = '127.0.0.1'
MESSAGE_ID = 1
REPLY_BYTES = 5  # [response=1, id=1, error=nil, result=bool/nil] 的编码长度。
REPLY_PREFIX = bytes([0x94, 1, MESSAGE_ID, 0xc0])


def build_tm_request(method, enabled=None):
    """仅编码本模块需要的两个官方方法；不提供任意远程调用。"""
    if method not in ('synchronous_tick', 'set_synchronous_mode'):
        raise ValueError('不支持的 Traffic Manager 操作。')
    name = method.encode('ascii')
    # 0x94: 四项数组；0: 请求；1: 请求编号；0xa0|n: 长度 n 的短字符串。
    prefix = bytes([0x94, 0, MESSAGE_ID, 0xa0 | len(name)]) + name
    if method == 'synchronous_tick':
        return prefix + bytes([0x90])  # 无参数数组。
    return prefix + bytes([0x91, 0xc3 if enabled else 0xc2])


def receive_tm_reply(connection):
    """TCP 可以拆包，不能假定一次 recv 就读到完整回复。"""
    reply = bytearray()
    while len(reply) < REPLY_BYTES:
        data = connection.recv(REPLY_BYTES - len(reply))
        if not data:
            raise ConnectionError('Traffic Manager 连接已关闭。')
        reply.extend(data)
    if reply[:4] != REPLY_PREFIX or reply[4] not in (0xc0, 0xc2, 0xc3):
        raise RuntimeError(f'Traffic Manager 回复不符合当前协议：{bytes(reply)!r}')
    return reply[4] == 0xc3


class TrafficManagerTickConnection:
    def __init__(self, host, port):
        self.connection = socket.create_connection((host, port), timeout=TM_RPC_TIMEOUT)
        try:
            self.connection.sendall(build_tm_request('set_synchronous_mode', True))
            receive_tm_reply(self.connection)
        except BaseException:
            self.connection.close()
            raise

    def advance_controller(self):
        self.connection.sendall(build_tm_request('synchronous_tick'))
        if not receive_tm_reply(self.connection):
            raise RuntimeError('Traffic Manager 未完成同步控制计算。')

    def close(self, restore_mode=True):
        try:
            if restore_mode:
                self.connection.sendall(build_tm_request('set_synchronous_mode', False))
                receive_tm_reply(self.connection)
        finally:
            self.connection.close()


class LincolnTrafficManagerClock:
    def __init__(self):
        self.carla_adapter = None
        self.managers = {}

    def attach_registered_managers(self, carla, records):
        if carla is not self.carla_adapter:
            self.release()
            self.carla_adapter = carla
        if carla is None:
            return
        ports = {record['tm_port'] for record in records
                 if record.get('tm_port') is not None and record['world_id'] == carla.world.id}
        for port in list(self.managers.keys() - ports):
            manager = self.managers.pop(port)
            try:
                manager.close()
            except Exception:
                logging.debug('已退出车辆的 Traffic Manager 已不可用：%s', port)
        for port in ports - self.managers.keys():
            try:
                # 当前模块和共享桥在同一主机运行，TM 服务由 Lincoln 进程创建。
                self.managers[port] = TrafficManagerTickConnection(LOCAL_TM_HOST, port)
            except Exception:
                logging.exception('桥等待本机 Traffic Manager 端口 %s', port)

    def advance_controllers(self):
        for port, manager in list(self.managers.items()):
            try:
                manager.advance_controller()
            except Exception:
                logging.exception('Traffic Manager 同步步进失败：%s', port)
                self.managers.pop(port)
                manager.close(restore_mode=False)

    def release(self):
        for port, manager in self.managers.items():
            try:
                manager.close()
            except Exception:
                logging.debug('释放时 Traffic Manager 已不可用：%s', port)
        self.managers.clear()
        self.carla_adapter = None
