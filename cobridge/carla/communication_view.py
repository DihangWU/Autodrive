"""Bridge-owned GUI visualization of confirmed Veins peer links."""
import logging
import os
from pathlib import Path
import subprocess
import sys
import time

from cobridge.common.ipc import atomic_json, control_folder, identity


class CommunicationView:
    def __init__(self, host, port):
        self.host, self.port = host, port
        folder = control_folder(host,port)
        self.state = folder / 'vanet-communication.json'
        self.log = folder / 'vanet-communication.log'
        self.process = None
        self.retry_at = 0

    def update(self, state):
        if state is None:
            self.close()
            return
        try:
            if not state['links'] and self.process is None:
                return
            atomic_json(self.state,dict(state,owner=identity()))
            if state['links'] and (self.process is None or self.process.poll() is not None) and time.monotonic()>=self.retry_at:
                env = dict(os.environ)
                for name in ('QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH'):
                    env.pop(name,None)
                with self.log.open('a') as log:
                    self.process = subprocess.Popen([sys.executable,str(Path(__file__).resolve().with_name('communication_overlay.py')),
                                                     '--state',str(self.state),'--host',self.host,'--port',str(self.port)],
                                                    env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,
                                                    close_fds=True,start_new_session=True)
                self.retry_at = time.monotonic()+5
        except (OSError,ValueError) as error:
            logging.warning('CARLA communication overlay: %s',error)

    def close(self):
        self.state.unlink(missing_ok=True)
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process = None
