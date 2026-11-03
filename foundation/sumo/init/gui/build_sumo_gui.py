"""统一构建带低延迟、双坐标复制、仿真倍率、Lincoln 跟踪、导航和 GTA Mode的项目 SUMO GUI。

各功能模块仅提供源码修改。本文件合并同一源文件的修改、编译副本，
复用本机已有 SUMO 静态库完成链接，并维护 .runtime/sumo/gui/ 缓存。
系统源码、静态库和可执行文件保持不动；无窗口模式不调用本构建入口。
"""
import fcntl
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

MODULE_FILE = Path(__file__).resolve()
PROJECT_DIR = MODULE_FILE.parents[4]
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'sumo' / 'gui'
WORK_DIR = RUNTIME_DIR / 'build'
GUI_BINARY = RUNTIME_DIR / 'sumo-gui'
LOCK_FILE = RUNTIME_DIR / 'build.lock'
SIGNATURE_FILE = RUNTIME_DIR / 'signature'
BUILD_LOG_FILE = RUNTIME_DIR / 'build.log'
LINK_RELATIVE = Path('CMakeFiles/sumo-gui.dir/link.txt')
DEFAULT_BUILD_RELATIVE = Path('build/cmake-build')
ARCHIVE_TOOL = 'ar'
SOURCE_TARGETS = {
    Path('gui'): 'gui',
    Path('utils/gui/windows'): 'utils_gui_windows',
    Path('utils/gui/globjects'): 'utils_gui_globjects',
}

sys.path.insert(0, str(PROJECT_DIR))
from foundation.sumo.init.gui import low_latency_sumo_gui, coordinate_sumo_gui, simu_speed_sumo_gui, lincoln_track_sumo_gui, lincoln_navigation_sumo_gui, lincoln_gta_mode_sumo_gui

from vanet.sumo.gui_build_and_integration import experiment_gui

FEATURE_MODULES = (low_latency_sumo_gui, coordinate_sumo_gui, simu_speed_sumo_gui, lincoln_track_sumo_gui, lincoln_navigation_sumo_gui, lincoln_gta_mode_sumo_gui, experiment_gui)
BUILD_MODULE_FILES = (MODULE_FILE, *(Path(module.__file__).resolve() for module in FEATURE_MODULES))


def collect_source_patches():
    """同一 C++ 文件的多个功能按顺序合并，避免后一个功能覆盖前一个。"""
    patches = {}
    for feature in FEATURE_MODULES:
        for relative, edits in feature.source_patches().items():
            patches.setdefault(relative, []).extend(edits)
    return patches


def find_sumo_build_inputs(patches):
    """校验本机完整源码、CMake 编译参数和要替换的原静态库。"""
    home = os.environ.get('SUMO_HOME')
    if not home:
        raise RuntimeError('构建项目 SUMO GUI 需要设置 SUMO_HOME，指向包含 src 的源码目录。')
    home = Path(home).expanduser().resolve()
    build = Path(os.environ.get('SUMO_BUILD_DIR', home / DEFAULT_BUILD_RELATIVE)).expanduser().resolve()
    build_src = build / 'src'
    sources = []
    for relative, edits in patches.items():
        target = SOURCE_TARGETS[relative.parent]
        archive_relative = relative.parent / f'lib{target}.a'
        sources.append(dict(source=home / 'src' / relative, edits=edits,
            flags=build_src / relative.parent / 'CMakeFiles' / f'{target}.dir' / 'flags.make',
            archive=build_src / archive_relative, archive_relative=archive_relative.as_posix()))
    link_file = build_src / LINK_RELATIVE
    required = {link_file}
    for item in sources:
        required.update(item[key] for key in ('source', 'flags', 'archive'))
    missing = sorted(str(path) for path in required if not path.is_file())
    if missing:
        raise RuntimeError('缺少 SUMO 源码或 CMake 编译产物：' + ', '.join(missing)
                           + '。请检查 SUMO_HOME / SUMO_BUILD_DIR。')
    return build_src, link_file, sources


def read_sumo_link_command(link_file, sources):
    command = shlex.split(link_file.read_text())
    if ('-o' not in command or command.index('-o') + 1 >= len(command)
            or any(item['archive_relative'] not in command for item in sources)):
        raise RuntimeError(f'SUMO 链接配置不受支持：{link_file}')
    return command


def calculate_build_signature(build_src, link_file, sources, link_command):
    """构建器与所有功能 Python 文件、原源码、修改内容、参数和链接依赖都参加缓存校验。"""
    digest = hashlib.sha256(str(build_src).encode())
    for module in BUILD_MODULE_FILES:
        digest.update(module.read_bytes())
    digest.update(link_file.read_bytes())
    for item in sources:
        digest.update(str(item['source']).encode())
        digest.update(item['source'].read_bytes())
        digest.update(item['flags'].read_bytes())
        digest.update(repr(item['edits']).encode())
    dependencies = {build_src / arg for arg in link_command if arg.endswith(('.a', '.o', '.so'))}
    for path in sorted(dependencies):
        stat = path.stat()
        digest.update(f'{path}:{stat.st_size}:{stat.st_mtime_ns}'.encode())
    return digest.hexdigest()


def patch_source_text(item):
    """每个修改位置必须唯一；版本不匹配时明确报错，不盲目替换。"""
    text = item['source'].read_text()
    for before, after in item['edits']:
        if text.count(before) != 1:
            raise RuntimeError(f'SUMO GUI 修改位置缺失或不唯一：{item["source"]}；请检查源码版本。')
        text = text.replace(before, after, 1)
    return text


def run_build_command(command, build_src, log):
    log.write(shlex.join(command) + '\n')
    log.flush()
    try:
        subprocess.run(command, cwd=build_src, stdout=log, stderr=subprocess.STDOUT, check=True)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f'项目 SUMO GUI 构建失败，请查看 {BUILD_LOG_FILE}') from error


def compile_source_copy(build_src, item, patched_text, compiler, log):
    """每个有修改的源文件只编译一次，原文件不改写。"""
    source = item['source']
    patched = WORK_DIR / source.name
    patched.write_text(patched_text)
    flags = {}
    for line in item['flags'].read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    if not all(key in flags for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS')):
        raise RuntimeError(f'SUMO 编译参数不完整：{item["flags"]}')
    obj = WORK_DIR / (source.name + '.o')
    command = [compiler, *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
               '-I' + str(source.parent), '-c', str(patched), '-o', str(obj)]
    print(f'[SUMO] 编译 GUI：{source.name}', flush=True)
    run_build_command(command, build_src, log)
    return obj


def link_project_sumo_gui(build_src, sources, link_command, objects, log):
    """同一静态库只复制一次，合并对象后链接并原子替换项目程序。"""
    archives = {}
    for item, obj in zip(sources, objects):
        key = item['archive_relative']
        if key not in archives:
            archive = WORK_DIR / item['archive'].name
            shutil.copy2(item['archive'], archive)
            archives[key] = (archive, [])
        archives[key][1].append(obj)
    for archive, members in archives.values():
        run_build_command([ARCHIVE_TOOL, 'r', str(archive), *map(str, members)], build_src, log)
    command = [str(archives[arg][0]) if arg in archives else arg for arg in link_command]
    temporary = GUI_BINARY.with_suffix('.new')
    command[command.index('-o') + 1] = str(temporary)
    run_build_command(command, build_src, log)
    os.replace(temporary, GUI_BINARY)


def ensure_sumo_gui():
    """run_sumo 的唯一 GUI 构建入口；缓存命中时直接返回现有程序。"""
    patches = collect_source_patches()
    build_src, link_file, sources = find_sumo_build_inputs(patches)
    command = read_sumo_link_command(link_file, sources)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        signature = calculate_build_signature(build_src, link_file, sources, command)
        if (GUI_BINARY.is_file() and os.access(GUI_BINARY, os.X_OK)
                and SIGNATURE_FILE.is_file() and SIGNATURE_FILE.read_text() == signature):
            return GUI_BINARY
        patched_sources = [patch_source_text(item) for item in sources]
        WORK_DIR.mkdir(exist_ok=True)
        print('[SUMO] 正在构建 GUI：低延迟、双坐标与复制、仿真倍率、Lincoln 跟踪、导航与 GTA Mode…', flush=True)
        with BUILD_LOG_FILE.open('a') as log:
            objects = [compile_source_copy(build_src, item, text, command[0], log)
                       for item, text in zip(sources, patched_sources)]
            link_project_sumo_gui(build_src, sources, command, objects, log)
        temporary_stamp = SIGNATURE_FILE.with_suffix('.new')
        temporary_stamp.write_text(signature)
        temporary_stamp.replace(SIGNATURE_FILE)
        print(f'[SUMO] GUI 已就绪：{GUI_BINARY.relative_to(PROJECT_DIR.parent)}', flush=True)
        return GUI_BINARY


if __name__ == '__main__':
    try:
        print(ensure_sumo_gui())
    except (RuntimeError, OSError) as error:
        raise SystemExit(f'错误：{error}')
