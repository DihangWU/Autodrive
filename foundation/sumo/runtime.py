"""SUMO owns its TraCI connection and advances autonomously without a bridge."""
import time
from foundation.sumo.traci.remote import invoke
from foundation.sumo.events import OperationEvents

STEP_TOLERANCE = 1e-9
ROOT_METHODS = frozenset({'simulationStep', 'getVersion'})


class SumoRuntime:
    def __init__(self, connection, config, step_length, on_clock_change=None, on_operation_confirmed=None):
        self.connection = connection
        self.config = str(config)
        self.step_length = step_length
        self.clock_owner = None
        self.next_step = time.monotonic()
        self.on_clock_change = on_clock_change
        self.events = OperationEvents(on_operation_confirmed)

    def set_clock_owner(self, owner):
        changed = self.clock_owner is not owner
        self.clock_owner = owner
        if owner is None:
            self.next_step = time.monotonic()
        if changed and self.on_clock_change is not None:
            self.on_clock_change(owner is not None)

    def handle(self, command, owner):
        action = command['action']
        if action == 'status':
            return dict(config=self.config, step_length=self.step_length,
                        time=self.connection.simulation.getTime(),
                        clock_claimed=self.clock_owner is not None)
        if action == 'clock.claim':
            if abs(float(command['step_length']) - self.step_length) > STEP_TOLERANCE:
                raise ValueError('CoBridge 与 SUMO 步长不一致。')
            if self.clock_owner is not None and self.clock_owner is not owner:
                raise RuntimeError('SUMO 已由另一个时钟接管。')
            self.set_clock_owner(owner)
            return dict(time=self.connection.simulation.getTime())
        if action == 'clock.release':
            if self.clock_owner is not None and self.clock_owner is not owner:
                raise RuntimeError('不能释放其他时钟的控制权。')
            self.set_clock_owner(None)
            return dict(released=True)
        if action in ('events.poll', 'events.ack'):
            if self.clock_owner is not owner:
                raise RuntimeError('操作通知需由当前时钟连接接收和确认。')
            if action == 'events.poll':
                return self.events.poll()
            return self.events.acknowledge(command['ids'], command.get('dropped', 0))
        if action == 'call':
            method = command['method']
            if method == 'simulationStep' and self.clock_owner is not owner:
                raise RuntimeError('只有当前时钟持有者可以推进 SUMO。')
            if '.' not in method and method not in ROOT_METHODS:
                raise ValueError('不允许远程修改 SUMO 生命周期。')
            args = command.get('args', [])
            result = invoke(self.connection, method, args, command.get('kwargs', {}))
            self.events.record_success(method, args)
            return result
        raise ValueError(f'未知 SUMO 控制请求：{action}')

    def disconnected(self, owner):
        if self.clock_owner is owner:
            self.set_clock_owner(None)

    def autonomous_step(self):
        now = time.monotonic()
        if self.clock_owner is None and now >= self.next_step:
            self.connection.simulationStep()
            self.next_step = now + self.step_length
