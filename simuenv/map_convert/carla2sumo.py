"""
读取 CARLA 当前地图，调用官方工具转换转换为 sumo 地图
Read the current CARLA map and convert it to a SUMO road network using the official tool
"""
# 本文件依赖 simuenv/carla/carla-official/ ，如果移动 carla/carla-official/ 需要修改本文件调用路径


import os
import sys
import argparse
from pathlib import Path 
import subprocess



SIMUENV_DIR = Path(__file__).resolve().parents[1]
# 当前文件的路径.的绝对路径.所在目录的上1层目录；.parents[0] = .parent
PROJECT_DIR = SIMUENV_DIR.parent
OFFICIAL_SUMO_DIR = SIMUENV_DIR / 'carla' / 'carla-official' / 'Co-Simulation' / 'Sumo'
MAPS_DIR = PROJECT_DIR / '.runtime' / 'maps'


def parse_arguments():
    """解析运行当前py文件的命令行参数"""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter) 
        # argparse.ArgumentParser() 创建自带解析器
        # description=__doc__：运行 --help 时，显示“__doc__“ (当前文件最开头的 """xxx""")
        # formatter_class=argparse.RawDescriptionHelpFormatter = __doc__ 换行，-h 输出也换行
    parser.add_argument('--host', default='127.0.0.1', help='CARLA address, defaut 127.0.0.1')
    parser.add_argument('--port', type=int, default=2000, help='CARLA RPC port, default 2000')
    parser.add_argument('--timeout', type=float, default=20, help='请求超时秒数，默认 20')
    args = parser.parse_args() #读取并解析输入的参数，把结果保存到 args 中
    if not 1 <= args.port <= 65535 or args.timeout <= 0:
        parser.error('port 必须为 1～65535，timeout 必须大于 0。') # 检查参数是否合法

    return args


def mapconvert_carla2sumo(carla_xodr_file, sumo_network_file, log_file):
    """调用官方脚本 netconvert_carla.py 转换地图, 并补充信号灯信息猜测。"""
    converter = OFFICIAL_SUMO_DIR / 'util' / 'netconvert_carla.py'  # 指定地图转换工具地址
    command = [sys.executable, str(converter), str(carla_xodr_file),
               '--output', str(sumo_network_file), '--guess-tls']
        # 这些参数都是 netconvert_carla.py 中定义的
        # sys.executable = sys.executable = 当前环境 python 解释器；str(converter) = netconvert_carla.py 的路径，
        # str(carla_xodr_file) = 输入的 xodr 路径地址; '--output', str(sumo_network_file) = 输出文件地址
        # --guess-tls 让转换工具推断猜测“这个信号灯应该控制哪些车道，写进生成的 .net.xml 中
    with log_file.open('w') as log:  # w = write，打开 log_file 指向的文件，清空后用于写入，通过标签 log 使用它，用完自动关闭。
        try:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            # subprocess.run = 启动另一个命令，等待程序执行结束，subprocess.Popen 不在终端等待
                # command,                    # 执行什么命令
                # stdout=log,                 # 普通输出写到日志文件，不打印到终端
                # stderr=subprocess.STDOUT,   # 错误输出也写到同一份日志，写到 log 中不不打印到终端
                # check=True                  # 命令退出码不是 0 时，抛出异常，由后面的 except 处理。

        except subprocess.CalledProcessError as error:  
            # 如果前面的命令成功，就跳过，
            # 如果执行失败，就捕获这个错误，并把它保存到变量 error
            raise RuntimeError(f'地图转换失败，请查看 {log_file}') from error
                # raise：抛出异常，当前正常流程停止。
                # RuntimeError(...)：创建一个运行时错误。
                # f = 拼接字符串和变量


def generate_sumocfg(config_file, sumo_network_file): # 这个函数生成 SUMO 的启动配置文件 .sumocfg
    """告诉 SUMO 加载哪个net.xml，以及使用什么显示配置。"""
    import xml.etree.ElementTree as ET
    root = ET.Element('configuration') # configuration 是 SUMO 配置文件要求的根标签名
        # 使用 py 自带的 xml 包(自命名为 ET)，调用 Element(...)：创建一个可控制对象，名字叫 configuration
    inputs = ET.SubElement(root, 'input')
        # 在 root 里面添加一个 <input> 子元素，并用变量 inputs 指向它。
    ET.SubElement(inputs, 'net-file', value=sumo_network_file.name)
        # 在 inputs 里面添加一个 <net-file> 类型的元素，并设置它的 value 属性。
        # sumo_network_file.name = 只取文件名字
    gui = ET.SubElement(root, 'gui_only') # 在 root 里面创建一个 <gui_only> 子元素，并用变量 gui 指向它。
        # gui_only 是 SUMO 配置文件中固定的配置分组名称，用于放图形界面相关设置，
    view_settings = OFFICIAL_SUMO_DIR / 'examples' / 'viewsettings.xml'
    ET.SubElement(gui, 'gui-settings-file', value=str(view_settings))
        # gui-settings-file 也是 SUMO 配置文件中固定的配置分组名称
    ET.indent(root)  # 给 XML 添加换行和缩进，让保存后的内容更好读。
    ET.ElementTree(root).write(config_file, encoding='utf-8', xml_declaration=True)
        # 把内存中的 XML 内容写入文件，生成 .sumocfg，
            # 本质是写 xml 文件，调用时通过 config_file 指定后缀 .sumoconfig
        # ET.ElementTree(root)：以 root 为根元素，组成完整的 XML 文档。
        # xml_declaration=True：在文件开头加上一行声明： <?xml version='1.0' encoding='utf-8'?>


def main():
    args = parse_arguments()
    if not os.environ.get('SUMO_HOME'):
        raise RuntimeError('请先设置 SUMO_HOME，并将 SUMO 的 bin 目录加入 PATH。')

    import carla

    # 1. 连接 CARLA，读取当前已经加载的地图。
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)  # 超过 timeout 时间无响应报错
    print(f'CARLA连接成功：{args.host}:{args.port}')

    current_map = client.get_world().get_map()
    map_name = Path(current_map.name).name  # .name 取 PATH 对象的最后一段
    if map_name in ('', '.', '..'): # 检查 map_name 是否等于这三个字符串中的任意一个
        raise RuntimeError(f'无效的地图名称：{current_map.name!r}') # py 自带报错，!r = 引号把输出括起来
    print(f'连接完成，当前地图：{map_name}')
    
    map_dir = MAPS_DIR / map_name
    map_dir.mkdir(parents=True, exist_ok=True)
        # 创建 创建 map_dir 指向的目录
        # parents=True：上层目录不存在时，也一起创建
        # exist_ok=True：目标目录已经存在时不报错，也不会清空里面的文件

    # 2. 导出 OpenDRIVE 道路数据。
    carla_xodr_file = map_dir / f'{map_name}.xodr'
    current_map.save_to_disk(str(carla_xodr_file))
    print(f'carla 地图导出成功：{carla_xodr_file.relative_to(PROJECT_DIR.parent)}', flush=True)
        # 打印后立即刷新输出缓冲区，让信息及时显示：

    # 3. 调用官方转换工具；失败时直接停止，不打开 SUMO。
    sumo_sumo_network_file = map_dir / f'{map_name}.net.xml'
    mapconvert_carla2sumo(carla_xodr_file, sumo_sumo_network_file, map_dir / 'conversion.log')
    print(f'转换 sumo 地图成功：{sumo_sumo_network_file.relative_to(PROJECT_DIR.parent)}\n',
          f'       日志存放在：{map_dir.relative_to(PROJECT_DIR.parent) / "conversion.log"}',  flush=True)

    # 4. 生成 SUMO 启动配置。
    sumo_config_file = map_dir / f'{map_name}.sumocfg'
    generate_sumocfg(sumo_config_file, sumo_sumo_network_file)
    print(f'生成 sumo 配置文件：{sumo_config_file.relative_to(PROJECT_DIR.parent)}')


if __name__ == '__main__':  # 如果直接运行这个python文件
    try:  # 执行一下代码
        main()
    except KeyboardInterrupt: # 如果用户按 Ctrl+C，
        raise SystemExit(130) # 记录130中断退出
    except (RuntimeError, OSError, ImportError) as error: # 捕获这些错误，并保存到变量 error
        raise SystemExit(f'错误：{error}') # 输出错误信息并退出
# raise SystemExit 是抛出一个“退出程序”的异常
