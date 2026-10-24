"""添加一辆 Lincoln；车辆、传感器和驾驶在本模块，SUMO 同步交给 CoBridge。"""
from pathlib import Path
import sys

PROJECT_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_DIR))

from modules.lincoln.runtime.arguments import parse_lincoln_arguments


def main():
    args = parse_lincoln_arguments()
    from modules.lincoln.runtime.application import run_lincoln_application
    run_lincoln_application(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        import logging
        logging.exception('Lincoln 退出')
        raise SystemExit(f'Lincoln 错误：{error}')
