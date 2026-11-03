"""紧急身份和一次导航任务分开保存；只有 SUMO cancel 清除紧急身份。"""
from dataclasses import dataclass, field

GROUPS = frozenset({'normal', 'emergency', 'cooperative'})


@dataclass
class EmergencyTaskState:
    emergency: bool = False
    state: str = 'idle'
    group: str = 'normal'
    task_id: str | None = None
    destination: dict | None = None
    route_version: int = 0
    manual: bool = False
    error: str = ''
    command_ack_id: str | None = None
    seen_commands: set = field(default_factory=set)

    def prepare(self, command):
        group = command.get('group', self.group)
        if group not in GROUPS:
            raise ValueError('Unknown experiment group: ' + str(group))
        if self.emergency and group == 'normal':
            raise ValueError('Cancel emergency in SUMO before selecting the normal group.')
        self.group = group
        self.emergency = self.emergency or group != 'normal'
        self.task_id = str(command.get('task_id') or command['id'])
        self.destination = dict(command['destination'])
        self.state, self.manual, self.error = 'preparing', False, ''

    def prepared(self):
        self.state = 'prepared'
        self.route_version += 1

    def start(self):
        if self.state != 'prepared':
            raise ValueError('The route and navigation display are not ready.')
        self.state, self.manual = 'running', False

    def takeover(self):
        if self.state in ('running', 'preparing', 'prepared', 'arrived', 'cancelled', 'reset', 'error'):
            self.state, self.manual = 'manual', True

    def arrive(self):
        self.state, self.manual = 'arrived', False

    def cancel_from_sumo(self):
        self.emergency, self.manual = False, False
        self.state, self.destination = 'cancelled', None

    def reset(self):
        if self.emergency:
            raise ValueError('Cancel emergency in SUMO before resetting the scene.')
        self.state, self.manual, self.error = 'reset', False, ''
        self.task_id, self.destination = None, None
        self.route_version = 0
