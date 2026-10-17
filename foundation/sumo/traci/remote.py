"""Generic public TraCI method RPC; no entity or clock policy."""


def invoke(connection, method, args, kwargs):
    parts = method.split('.')
    if not parts or any(not part.isidentifier() or part.startswith('_') for part in parts):
        raise ValueError('无效 TraCI 方法。')
    target = connection
    for part in parts:
        target = getattr(target, part)
    if not callable(target):
        raise ValueError('目标不是 TraCI 方法。')
    return target(*args, **kwargs)


class RemoteConnection:
    def __init__(self, client, path=''):
        self.client, self.path = client, path

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)
        return RemoteConnection(self.client, f'{self.path}.{name}' if self.path else name)

    def __call__(self, *args, **kwargs):
        return self.client.command('call', method=self.path, args=args, kwargs=kwargs)
