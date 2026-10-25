"""低延迟 GUI 修改：缩短 Linux 界面重绘的额外等待，不改变仿真步长。"""
from pathlib import Path

SOURCE_FILE = Path('gui/GUIRunThread.cpp')
ORIGINAL_REDRAW_WAIT = '''// ensure redraw event is successful at least once per second (#9028)
            sleep(100);'''
LOW_LATENCY_REDRAW_WAIT = '''// Keep redraw opportunities without blocking external TraCI clocks for 100 ms.
            sleep(10);'''


def source_patches():
    """由统一构建入口应用到源码副本，本模块不编译或启动进程。"""
    return {SOURCE_FILE: [(ORIGINAL_REDRAW_WAIT, LOW_LATENCY_REDRAW_WAIT)]}
