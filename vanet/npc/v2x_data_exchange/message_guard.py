"""业务接收校验：拒绝未来、过期、重复和回退版本，独立于传输协议。"""
import math


class MessageGuard:
    def __init__(self, max_age=10.):
        self.max_age = max_age
        self.sequences = {}
        self.versions = {}
        self.cancelled = set()

    def accept(self, message, now):
        try:
            for field in ('type', 'task_id', 'route_version', 'sequence', 'sent_at', 'source'):
                if field not in message:
                    return False
            version, sequence = message['route_version'], message['sequence']
            if isinstance(version, bool) or isinstance(sequence, bool) or not isinstance(version, int) or not isinstance(sequence, int):
                return False
            timestamp = float(message['sent_at'])
            if not math.isfinite(timestamp) or timestamp > now+1e-8 or now-timestamp > self.max_age:
                return False
            task, source, kind = str(message['task_id']), str(message['source']), message['type']
            if min(version, sequence) < 0 or not source:
                return False
            if kind in ('emergency', 'notice', 'cancel'):
                if (kind != 'cancel' and task in self.cancelled) or version < self.versions.get(task, -1):
                    return False
            key = source, task, version, kind
            if sequence <= self.sequences.get(key, -1):
                return False
            self.sequences[key] = sequence
            if kind in ('emergency', 'notice', 'cancel'):
                self.versions[task] = max(version, self.versions.get(task, -1))
            if kind == 'cancel' and message.get('scope', 'task') == 'task':
                self.cancelled.add(task)
            return True
        except (TypeError, ValueError, OverflowError):
            return False
