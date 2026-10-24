"""构建本工程专用的低延迟 SUMO GUI。

复制本机 SUMO 的 GUIRunThread.cpp，将 Linux 下用于界面重绘的
周期性休眠从 100 毫秒缩短为 10 毫秒，减少 TraCI 步进对共享时钟
和点云更新的阻塞。

仅编译修改后的源码副本，并复用已有 SUMO 编译库完成链接，
不修改系统 SUMO 的源码或可执行文件。

构建产物保存在工程根目录 .runtime/sumo/gui/。
首次使用或构建输入变化时重新构建，其余启动直接复用已有产物。
无窗口模式不调用本模块的构建函数。
"""
import fcntl
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess

# 项目路径、SUMO 编译文件位置和修复参数集中维护。
MODULE_FILE = Path(__file__).resolve()
PROJECT_DIR = MODULE_FILE.parents[3]
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'sumo' / 'gui'
WORK_DIR = RUNTIME_DIR / 'build'
GUI_BINARY = RUNTIME_DIR / 'sumo-gui'
LOCK_FILE = RUNTIME_DIR / 'build.lock'
SIGNATURE_FILE = RUNTIME_DIR / 'signature'
BUILD_LOG_FILE = RUNTIME_DIR / 'build.log'
SOURCE_RELATIVE = Path('src/gui/GUIRunThread.cpp')
FLAGS_RELATIVE = Path('gui/CMakeFiles/gui.dir/flags.make')
LINK_RELATIVE = Path('CMakeFiles/sumo-gui.dir/link.txt')
GUI_ARCHIVE_RELATIVE = Path('gui/libgui.a')
DEFAULT_BUILD_RELATIVE = Path('build/cmake-build')
ARCHIVE_TOOL = 'ar'
ORIGINAL_REDRAW_WAIT = '''// ensure redraw event is successful at least once per second (#9028)
            sleep(100);'''
LOW_LATENCY_REDRAW_WAIT = '''// Keep redraw opportunities without blocking external TraCI clocks for 100 ms.
            sleep(10);'''


def find_sumo_build_inputs():
    """读取 SUMO_HOME 与 SUMO_BUILD_DIR，检查完整源码和已有编译产物。"""
    home = os.environ.get('SUMO_HOME')
    if not home:
        raise RuntimeError('构建低延迟 SUMO GUI 需要设置 SUMO_HOME，指向包含 src 的 SUMO 源码目录。')
    home = Path(home).expanduser().resolve()
    build = Path(os.environ.get('SUMO_BUILD_DIR', home / DEFAULT_BUILD_RELATIVE)).expanduser().resolve()
    build_src = build / 'src'
    paths = dict(source=home / SOURCE_RELATIVE, flags=build_src / FLAGS_RELATIVE,
                 link=build_src / LINK_RELATIVE, archive=build_src / GUI_ARCHIVE_RELATIVE)
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise RuntimeError('缺少 SUMO 源码或 CMake 编译产物：' + ', '.join(missing)
                           + '。请检查 SUMO_HOME / SUMO_BUILD_DIR。')
    return build_src, paths


def read_sumo_link_command(link_file):
    """使用本机原链接配置，但先确认能替换输出路径和 GUI 静态库。"""
    command = shlex.split(link_file.read_text())
    if ('-o' not in command or command.index('-o') + 1 >= len(command)
            or GUI_ARCHIVE_RELATIVE.as_posix() not in command):
        raise RuntimeError(f'SUMO 链接配置不受支持：{link_file}')
    return command


def calculate_build_signature(build_src, paths, link_command):
    """脚本、源码、编译参数或所链接的库/对象变化时使缓存失效。"""
    digest = hashlib.sha256(MODULE_FILE.read_bytes())
    digest.update(str(build_src).encode())
    for key in ('source', 'flags', 'link'):
        digest.update(paths[key].read_bytes())
    dependencies = {build_src / arg for arg in link_command if arg.endswith(('.a', '.o', '.so'))}
    for path in sorted(dependencies):
        stat = path.stat()
        digest.update(f'{path}:{stat.st_size}:{stat.st_mtime_ns}'.encode())
    return digest.hexdigest()


def run_build_command(command, build_src, log):
    """编译输出写入独立日志，失败时向启动入口报告清晰错误。"""
    log.write(shlex.join(command) + '\n')
    log.flush()
    try:
        subprocess.run(command, cwd=build_src, stdout=log, stderr=subprocess.STDOUT, check=True)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f'低延迟 SUMO GUI 构建失败，请查看 {BUILD_LOG_FILE}') from error


def compile_redraw_source_copy(build_src, paths, compiler, log):
    """只复制和编译 GUIRunThread.cpp，系统源码保持原样。"""
    source = paths['source']
    text = source.read_text()
    if text.count(ORIGINAL_REDRAW_WAIT) != 1:
        raise RuntimeError('当前 SUMO 源码没有唯一的 100 ms 重绘休眠位置，拒绝盲目修改。')
    patched = WORK_DIR / source.name
    patched.write_text(text.replace(ORIGINAL_REDRAW_WAIT, LOW_LATENCY_REDRAW_WAIT, 1))
    flags = {}
    for line in paths['flags'].read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    if not all(key in flags for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS')):
        raise RuntimeError(f'SUMO 编译参数不完整：{paths["flags"]}')
    obj = WORK_DIR / (source.name + '.o')
    command = [compiler, *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
               '-I' + str(source.parent), '-c', str(patched), '-o', str(obj)]
    run_build_command(command, build_src, log)
    return obj


def link_project_sumo_gui(build_src, paths, link_command, obj, log):
    """在库副本中替换一个对象，链接到项目临时文件后原子替换。"""
    archive = WORK_DIR / paths['archive'].name
    shutil.copy2(paths['archive'], archive)
    run_build_command([ARCHIVE_TOOL, 'r', str(archive), str(obj)], build_src, log)
    command = [str(archive) if arg == GUI_ARCHIVE_RELATIVE.as_posix() else arg
               for arg in link_command]
    temporary = GUI_BINARY.with_suffix('.new')
    command[command.index('-o') + 1] = str(temporary)
    run_build_command(command, build_src, log)
    os.replace(temporary, GUI_BINARY)


def ensure_low_latency_sumo_gui():
    """返回项目 GUI 路径；输入未变则复用，否则独占构建并更新缓存。"""
    build_src, paths = find_sumo_build_inputs()
    command = read_sumo_link_command(paths['link'])
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        signature = calculate_build_signature(build_src, paths, command)
        if (GUI_BINARY.is_file() and os.access(GUI_BINARY, os.X_OK)
                and SIGNATURE_FILE.is_file() and SIGNATURE_FILE.read_text() == signature):
            return GUI_BINARY
        WORK_DIR.mkdir(exist_ok=True)
        print('[SUMO] 正在构建低延迟 GUI（首次使用或构建输入有变化）…', flush=True)
        with BUILD_LOG_FILE.open('a') as log:
            obj = compile_redraw_source_copy(build_src, paths, command[0], log)
            link_project_sumo_gui(build_src, paths, command, obj, log)
        temporary_stamp = SIGNATURE_FILE.with_suffix('.new')
        temporary_stamp.write_text(signature)
        temporary_stamp.replace(SIGNATURE_FILE)
        print(f'[SUMO] 低延迟 GUI 已就绪：{GUI_BINARY.relative_to(PROJECT_DIR.parent)}', flush=True)
        return GUI_BINARY


if __name__ == '__main__':
    try:
        print(ensure_low_latency_sumo_gui())
    except (RuntimeError, OSError) as error:
        raise SystemExit(f'错误：{error}')
