"""Successful public object operations await bridge receipt before local display."""
from collections import OrderedDict
from itertools import islice
import uuid

# 通用事件参数；方法标签不包含具体业务模块。
EVENT_CAPACITY = 2048
EVENT_BATCH_SIZE = 64
MAX_ENTITY_LENGTH = 128
OPERATION_LABELS = {'add': '创建成功', 'remove': '删除成功'}


class OperationEvents:
    def __init__(self, display=None):
        self.display = display
        self.pending = OrderedDict()
        self.dropped = 0
        self.sequence = 0
        self.session = uuid.uuid4().hex

    def record_success(self, method, args):
        domain, separator, operation = method.rpartition('.')
        if not separator or operation not in OPERATION_LABELS:
            return
        self.sequence += 1
        entity = str(args[0]) if args else ''
        entity = entity.replace('\r', ' ').replace('\n', ' ')[:MAX_ENTITY_LENGTH]
        label = OPERATION_LABELS[operation]
        message = f'{domain} {entity}：{label}。' if entity else f'{domain}：{label}。'
        event = dict(id=f'{self.session}:{self.sequence}', method=method,
                     entity=entity, message=message)
        if len(self.pending) >= EVENT_CAPACITY:
            self.pending.popitem(last=False)
            self.dropped += 1
        self.pending[event['id']] = event

    def poll(self):
        return dict(events=list(islice(self.pending.values(), EVENT_BATCH_SIZE)), dropped=self.dropped)

    def acknowledge(self, event_ids, dropped=0):
        if not isinstance(event_ids, list) or not all(isinstance(item, str) for item in event_ids):
            raise ValueError('无效事件确认列表。')
        if not isinstance(dropped, int) or dropped < 0:
            raise ValueError('无效丢弃事件计数。')
        confirmed = 0
        for event_id in event_ids:
            event = self.pending.get(event_id)
            if event is None:
                continue
            if self.display is not None:
                self.display(event['message'])
            self.pending.pop(event_id)
            confirmed += 1
        lost = min(dropped, self.dropped)
        if lost:
            if self.display is not None:
                self.display(f'有 {lost} 条操作通知因缓冲区满而丢弃。')
            self.dropped -= lost
        return dict(confirmed=confirmed)
