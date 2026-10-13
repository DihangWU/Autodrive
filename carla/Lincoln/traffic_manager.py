"""Identify a locally owned CARLA Traffic Manager before shutting it down."""
from pathlib import Path


def owns_listener(port):
    """Whether this Linux process owns the listening socket on ``port``.

    CARLA's Python API exposes shut_down() but no server/client ownership flag.
    Calling shut_down() on a TM client would also stop the remote TM server.
    """
    sockets = set()
    for fd in Path('/proc/self/fd').iterdir():
        try:
            target = str(fd.readlink())
        except OSError:
            continue
        if target.startswith('socket:[') and target.endswith(']'):
            sockets.add(target[8:-1])
    for name in ('tcp', 'tcp6'):
        table = Path('/proc/self/net') / name
        if not table.exists():
            continue
        for line in table.read_text().splitlines()[1:]:
            fields = line.split()
            if (fields[3] == '0A' and int(fields[1].rsplit(':', 1)[1], 16) == port
                    and fields[9] in sockets):
                return True
    return False
