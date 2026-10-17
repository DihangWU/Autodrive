"""Process-owned terminal and log output; no registry of business modules."""
import logging
import sys

# 输出格式与级别配置
MESSAGE_FORMAT = '[{component}] {message}'
STDERR_LEVEL = logging.WARNING


class Console:
    def __init__(self, component):
        self.component = component
        self.logger = logging.getLogger(component)
        # Before an entry point configures file logging, avoid lastResort also
        # printing the same warning/error; terminal output is owned here.
        if not self.logger.handlers:
            self.logger.addHandler(logging.NullHandler())

    def write(self, message, level=logging.INFO, *, exc_info=False):
        rendered = MESSAGE_FORMAT.format(component=self.component, message=message)
        self.logger.log(level, rendered, exc_info=exc_info)
        stream = sys.stderr if level >= STDERR_LEVEL else sys.stdout
        print(rendered, file=stream, flush=True)

    def info(self, message):
        self.write(message)

    def warning(self, message, *, exc_info=False):
        self.write(message, logging.WARNING, exc_info=exc_info)

    def error(self, message, *, exc_info=False):
        self.write(message, logging.ERROR, exc_info=exc_info)
