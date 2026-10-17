"""Check foreground launcher signal cleanup with a fake Docker CLI."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

PROJECT_DIR = Path(__file__).resolve().parents[1]
LAUNCHER = PROJECT_DIR / 'foundation/carla/init/carla_init.sh'
TEST_TIMEOUT = 5.0


class CarlaLauncherTests(unittest.TestCase):
    def test_ctrl_c_stops_only_created_container(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            commands = root / 'commands.log'
            scripts = {
                'sudo': '#!/bin/sh\nexec "$@"\n',
                'xhost': '#!/bin/sh\nexit 0\n',
                'docker': '''#!/usr/bin/env python3
import os, signal, sys, time
with open(os.environ['CARLA_TEST_LOG'],'a') as output:
    output.write(' '.join(sys.argv[1:])+'\\n')
if sys.argv[1]=='create':
    print('test-owned-container',flush=True)
elif sys.argv[1]=='start':
    with open(os.environ['CARLA_TEST_PID'],'w') as output:
        output.write(str(os.getpid()))
    time.sleep(60)
elif sys.argv[1]=='stop':
    try:
        with open(os.environ['CARLA_TEST_PID']) as source:
            os.kill(int(source.read()),signal.SIGTERM)
    except (FileNotFoundError,ProcessLookupError):
        pass
''',
            }
            for name, text in scripts.items():
                path = root / name
                path.write_text(text)
                path.chmod(0o755)
            env = dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH'], CARLA_TEST_LOG=str(commands), CARLA_TEST_PID=str(root / 'attach.pid'))
            process = subprocess.Popen(['bash', str(LAUNCHER)], env=env,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                       start_new_session=True)
            try:
                deadline = time.monotonic()+TEST_TIMEOUT
                while time.monotonic()<deadline:
                    if commands.exists() and 'start --attach' in commands.read_text():
                        break
                    time.sleep(.01)
                else:
                    self.fail('launcher did not attach in foreground')
                self.assertIsNone(process.poll())
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=TEST_TIMEOUT)
                text = commands.read_text()
                self.assertIn('stop --time 5 test-owned-container', text)
                self.assertNotIn('run --rm -d', text)
                self.assertEqual(process.returncode,130)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=TEST_TIMEOUT)


if __name__ == '__main__':
    unittest.main()
