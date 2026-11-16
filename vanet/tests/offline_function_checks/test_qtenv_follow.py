"""独立离屏 Qt 控件测试：居中、缩放、拖动暂停、恢复及图元销毁。"""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

from vanet.project_paths import OMNET_COBRIDGE_INTERFACE_DIRECTORY, OMNET_INCLUDE_DIRECTORY


class QtenvFollowTests(unittest.TestCase):
    def test_native_view_interactions(self):
        compiler=shutil.which('g++')
        if compiler is None:self.skipTest('Qt view test requires g++')
        flags=subprocess.run(['pkg-config','--cflags','--libs','Qt5Widgets'],capture_output=True,text=True)
        if flags.returncode:self.skipTest('Qt5Widgets development files unavailable')
        with tempfile.TemporaryDirectory(prefix='qtenv_follow_') as directory:
            binary=Path(directory)/'view_check'
            sources=[OMNET_COBRIDGE_INTERFACE_DIRECTORY/'QtLincolnFollow.cc',
                     Path(__file__).with_name('qt_lincoln_follow_check.cc')]
            command=[compiler,'-std=c++17','-fPIC','-I'+str(OMNET_COBRIDGE_INTERFACE_DIRECTORY),
                     '-I'+str(OMNET_INCLUDE_DIRECTORY),*map(str,sources),*shlex.split(flags.stdout),'-o',str(binary)]
            result=subprocess.run(command,capture_output=True,text=True,timeout=45)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=10,
                                  env=dict(os.environ,QT_QPA_PLATFORM='offscreen'))
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
