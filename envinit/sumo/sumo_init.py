"""读取已有的 SUMO 配置文件并打开 SUMO GUI。"""

import argparse
from pathlib import Path
import subprocess


PROJECT_DIR = Path(__file__).resolve().parents[2]
MAPS_DIR = PROJECT_DIR / '.runtime' / 'maps'


def find_configuration():
    """默认使用 runtime/maps 中最近修改的配置文件。"""
    configurations = list(MAPS_DIR.glob('*/*.sumocfg'))
    if not configurations:
        raise RuntimeError('runtime/maps 中没有 .sumocfg，请先运行地图转换脚本。')
    return max(configurations, key=lambda path: path.stat().st_mtime) # Max = 修改时间最近的那个


def open_sumo(config_file):
    """加载已有配置，后台打开 SUMO，输出追加到同目录的 sumo.log。"""
    log_file = config_file.parent / 'sumo.log'
    with log_file.open('a') as log:
        process = subprocess.Popen(
            ['sumo-gui', '-c', str(config_file)],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    print(f'加载 SUMO 配置：{config_file.relative_to(PROJECT_DIR.parent)}')
    print(f'SUMO GUI 进程已启动（PID {process.pid}），日志：{log_file.relative_to(PROJECT_DIR.parent)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path,
                        help='.sumocfg 路径；不填则使用 runtime/maps 中最近修改的配置')
    args = parser.parse_args()

    # 指定文件时直接使用；否则从运行产物目录中选择最新配置。
    config_file = args.config if args.config is not None else find_configuration()
    config_file = config_file.resolve()
    if not config_file.is_file() or config_file.suffix != '.sumocfg':
        raise RuntimeError(f'找不到有效的 .sumocfg 配置文件：{config_file}')
    open_sumo(config_file)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError) as error:
        raise SystemExit(f'错误：{error}')
