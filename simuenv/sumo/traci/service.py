"""持有唯一的 TraCI 连接，串行处理地图、POI 和统一时钟的步进请求。"""

import fcntl  # 导入 Linux 文件锁接口，防止同时启动多个 SUMO 管理端。
import hashlib  # 导入哈希计算工具，用 SHA-256 判断两份 OpenDRIVE 地图内容是否一致。
import json  # 导入 JSON 工具，将请求和回复在 Python 字典与 JSON 数据之间转换。
import os  # 导入操作系统接口，用于读取环境变量、获取 PID、设置文件权限等。
from pathlib import Path  # 导入 Path，用路径对象拼接文件位置、读取文件和创建目录。
import signal  # 导入信号接口，接收 Ctrl+C 或正常终止进程的通知。
import socket  # 导入 socket 模块，用 UNIX socket 接收本机其他模块的请求。
import subprocess  # 导入子进程工具，让启动入口另起一个后台 Python 服务进程。
import sys  # 导入解释器信息和模块搜索路径接口。
import time  # 导入计时与休眠工具，控制启动等待和定期检查的间隔。
import xml.etree.ElementTree as ET  # 导入 XML 解析工具并简称 ET，用来读取 SUMO 配置和路网文件。

PROJECT_DIR = Path(__file__).resolve().parents[3]  # 取得当前文件绝对路径，再向上取第四个父目录，即工程根目录。

sys.path.insert(0, str(PROJECT_DIR)) # 把工程根目录放到 Python 模块搜索路径的最前面，方便导入工程里的代码。；str 把 Path 转成字符串；insert(0, ...) 把工程根目录放到模块查找列表最前面。
    # sys.path：Python 查找模块时使用的目录列表。
    # .insert(0, ...)：在列表索引 0，也就是最前面插入一项。
from simuenv.sumo.traci.client import RUNTIME_DIR, SOCKET_FILE, STEP_LENGTH, owner_alive, request  # 复用客户端定义的运行目录、socket 地址、步长、进程存活检查及请求接口。


def start_sumo(config_file, headless=False):  # 定义后台启动入口：config_file 是配置路径，headless=True 表示 SUMO 无窗口运行。
    """启动后台管理端，等待就绪；同一配置已运行时直接复用。"""
    try:  # 尝试访问已有管理端；通信失败时由下面的 except 处理。
        status = request({'action': 'status'}, timeout=1)  # 发送 status 请求，用 1 秒 socket 超时检查已有服务是否可以响应。
    except RuntimeError:  # 捕获 request 包装后的运行错误，允许后续尝试启动新服务。
        status = None  # 没有得到有效状态，以 None 表示尚未发现可用服务。

    if status:  # 如果拿到了非空状态字典，就先检查能否复用已有服务。
        if status['config'] != str(config_file) or status.get('headless', False) != headless:  # 比较配置路径和显示模式；get 的 False 是旧状态里没有 headless 字段时的默认值。
            raise RuntimeError('SUMO 已使用其他配置或显示模式，请先关闭旧管理端。')  # 已有服务不符合本次要求时抛出异常，避免误操作另一个配置的 SUMO。
        print(f'复用 SUMO 管理端（PID {status["pid"]}）：{config_file}')  # 用 f 字符串打印已存在的服务进程 PID 和所加载的配置路径。
        return  # 复用成功，直接结束函数；没有指定返回值时返回 None。

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)  # 创建运行目录；parents=True 允许补建父目录，exist_ok=True 允许目录已存在。
    log_file = RUNTIME_DIR / 'service.log'  # 拼出管理服务自己的日志文件路径，不是 SUMO 仿真日志。
    command = [sys.executable, '-u', str(Path(__file__).resolve()), str(config_file)]  # 组装子进程参数：当前 Python、关闭输出缓冲的 -u、自身脚本路径、SUMO 配置路径。
    if headless:  # 检查调用者是否选择无窗口模式。
        command.append('--headless')  # 向参数列表末尾追加 --headless，传给后台 service.py 的命令行解析器。
    with log_file.open('a') as log:  # 以追加模式打开日志，with 结束后自动关闭当前进程持有的文件对象。
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,  # 启动子进程但不等它结束；标准输入接空设备，普通输出写日志，process 保存进程句柄。
                                   stderr=subprocess.STDOUT, start_new_session=True)  # 把错误输出合并到普通输出，并建立新会话，使后台服务与启动终端分离。
    deadline = time.monotonic() + 20  # 使用不受系统时间校准影响的单调时钟，设定约 20 秒的就绪等待截止点。
    while time.monotonic() < deadline:  # 截止点之前持续检查服务；单次请求自身的耗时也计入等待过程。
        try:  # 尝试查询正在启动的服务。
            status = request({'action': 'status'}, timeout=1)  # 再次发送 status；服务完成初始化并开始监听后才会正常回复。
        except RuntimeError:  # 服务尚未就绪时，捕获当前查询的通信错误。
            status = None  # 本轮未得到状态，继续后面的进程检查和重试逻辑。
        if status:  # 如果拿到了状态，检查响应服务是否加载了要求的配置。
            if status['config'] != str(config_file):  # 比较状态中的配置字符串和本次传入路径。
                raise RuntimeError('另一个 SUMO 管理端已加载其他配置。')  # 响应来自另一配置的服务时停止启动流程，并给出明确原因。
            print(f'加载 SUMO 配置：{os.path.relpath(config_file, PROJECT_DIR.parent)}')  # 把配置路径显示为相对于工程父目录的路径，保留工程目录名便于识别。
            print(f'TraCI 管理端已就绪（PID {status["pid"]}），终端可继续使用。')  # 提示后台服务已就绪；PID 来自服务回复，而不是 SUMO 窗口进程的 PID。
            print(f'管理日志：{log_file.relative_to(PROJECT_DIR.parent)}')  # 打印管理服务日志位置，便于查看启动或运行错误。
            return  # 就绪确认完成后结束启动函数，终端可以继续使用。
        if process.poll() is not None:  # poll 不等待进程；返回 None 表示子进程仍在运行，其他值表示退出码。
            break  # 子进程已经退出就结束等待，进入下面的启动失败处理。
        time.sleep(.1)  # 休眠 0.1 秒，避免启动等待期间连续发请求占满 CPU。
    if process.poll() is None:  # 等待结束后检查服务子进程是否仍然存活。
        process.terminate()  # 如果子进程一直未就绪，就发送终止信号；此处没有同步等待进程退出。
    raise RuntimeError(f'SUMO 管理端启动失败，请查看 {log_file}')  # 抛出启动失败异常，提示调用者去管理日志查看具体原因。


def read_network(config_file):  # 定义地图信息读取函数，参数是已有的 SUMO 配置文件路径。
    """从实际 SUMO 配置及路网读取地图身份和坐标偏移，不手填偏移。"""
    config = ET.parse(config_file).getroot()  # 解析 .sumocfg XML，并取出 configuration 根元素。
    value = config.find('input/net-file').get('value')  # 查找 input 下的 net-file 元素，读取其 value 属性作为路网文件名或路径。
    network_file = (config_file.parent / value).resolve()  # 相对路径以配置所在目录为基准；若 value 已是绝对路径，则使用该绝对路径。
    network = ET.parse(network_file).getroot()  # 解析 .net.xml，取得路网 XML 的根元素。
    offset = [float(number) for number in network.find('location').get('netOffset').split(',')]  # 读取 location 的 netOffset，按逗号拆分后将各项转成浮点数，得到坐标偏移列表。
    map_name = network_file.name.removesuffix('.net.xml')  # 只取路网文件名并移除 .net.xml 后缀，例如 Town04.net.xml 变为 Town04。
    xodr_file = network_file.with_name(map_name + '.xodr')  # 用同一目录和地图名构造对应的 .xodr 文件路径。
    fingerprint = hashlib.sha256(xodr_file.read_text().strip().encode()).hexdigest() if xodr_file.is_file() else None  # 存在 xodr 时读取文本、去掉首尾空白、编码并计算 SHA-256；不存在则记为 None。
    return dict(config=str(config_file), network=str(network_file), map=map_name,  # 构造并返回地图信息字典，路径转成字符串，以便后续用 JSON 传输。
                offset=offset, xodr_sha256=fingerprint)  # 继续填写返回字典：实际坐标偏移和 OpenDRIVE 文件内容指纹。


class SumoService:  # 定义服务状态类，将 TraCI 连接、RSU 会话和时钟所有权集中到一个对象中。
    def __init__(self, connection, network):  # 构造方法：创建 SumoService 对象时自动执行；self 表示本次创建的对象。
        self.connection = connection  # 保存已经建立的 TraCI 连接；此处不创建第二条连接。
        self.network = network  # 保存 read_network 返回的地图信息字典。
        self.sessions = {}  # 初始化 RSU 会话字典，以会话 ID 关联所属进程及其 POI ID 列表。
        self.stopped = False  # 初始化退出标志为 False，服务主循环据此决定是否继续运行。
        self.clock = None  # None 表示当前没有进程持有 SUMO 时钟控制权。

    def remove_session(self, session):  # 定义清理方法：删除指定会话的 SUMO 图标及管理记录，不处理 CARLA 实体。
        owned = self.sessions.get(session, {})  # get 查询会话；若不存在，返回空字典，使清理不存在会话也能正常结束。
        existing = set(self.connection.poi.getIDList())  # 查询 SUMO 中实际存在的全部 POI ID，转成集合以便快速判断成员是否存在。
        for name in owned.get('ids', []):  # 遍历这个会话记录的图标 ID；没有记录时使用空列表。
            if name in existing:  # 仅删除仍在 SUMO 中的图标，避免图标已经消失时重复删除报错。
                self.connection.poi.remove(name, layer=100)  # 通过 TraCI 删除指定 POI，layer=100 与创建时使用的图层一致。
        self.sessions.pop(session, None)  # 删除会话记录；默认值 None 让不存在的会话也不会引发 KeyError。

    def handle(self, command):  # 定义请求分发方法，接收客户端发来的 command 字典并返回业务执行结果。
        action = command['action']  # 用字典的 action 键取得操作名称；缺少该键会抛出 KeyError。
        if action == 'status':  # 处理查询状态请求，这个分支不推进仿真。
            pois = list(self.connection.poi.getIDList())  # 读取当前 POI ID，并转为列表，供返回和查询位置使用。
            return dict(self.network, pid=os.getpid(), pois=pois,  # 以地图字典为基础创建新状态字典，再加入当前服务进程 PID 和 POI ID 列表。
                        poi_positions={name: self.connection.poi.getPosition(name) for name in pois},  # 字典推导式：为每个 POI ID 查询坐标，构成 ID 到位置的对应关系。
                        sessions=list(self.sessions), time=self.connection.simulation.getTime(),  # 返回会话 ID 列表，以及 SUMO 当前累计仿真时间，单位为秒。
                        step_length=self.connection.simulation.getDeltaT(), clock=self.clock)  # 返回 SUMO 实际仿真步长和当前时钟记录；clock 为 None 表示无人持有时钟。
        if action == 'claim_clock':  # 处理 CoBridge 申请 SUMO 时钟控制权的请求。
            owner = command['owner']  # 取出请求进程的身份字典，包含 PID 和启动时刻。
            if not owner_alive(owner):  # 检查申请进程是否仍存活且 PID 未被其他进程复用。
                raise ValueError('时钟进程已经退出。')  # 申请者已退出时拒绝登记，交给外层转换成失败回复。
            if self.clock and owner_alive(self.clock['owner']):  # 已有时钟且其所属进程仍存活时，不允许另一个时钟再次接管。
                raise ValueError('SUMO 已由另一个时钟管理，不能重复接管。')  # 抛出控制权冲突异常，防止多个时钟同时驱动 SUMO。
            if abs(self.connection.simulation.getDeltaT() - STEP_LENGTH) > 1e-9:  # 比较 SUMO 实际步长和公共常量；1e-9 是允许的微小浮点误差。
                raise ValueError('SUMO 步长不是 0.05 秒，请重启 SUMO 管理端。')  # 实际步长不符时拒绝接管，提示重新启动采用正确步长的 SUMO。
            self.clock = dict(owner=owner, sequence=0)  # 记录时钟所属进程，并把当前会话的步进序号初始化为 0。
            return dict(time=self.connection.simulation.getTime())  # 返回接管时的 SUMO 仿真时间，CoBridge 用它作为本次时间增量的起点。
        if action in ('step', 'release_clock'):  # step 和 release_clock 都必须先经过下面的时钟所有权检查。
            if not self.clock or command['owner'] != self.clock['owner']:  # 未登记时钟，或请求身份与当前所有者不一致时，都不允许操作。
                raise ValueError('请求方没有持有 SUMO 时钟。')  # 抛出没有控制权的异常，不执行任何步进。
            if action == 'release_clock':  # 所有权检查通过后，判断本次是否只是释放时钟。
                self.clock = None  # 清空时钟记录，使之后的时钟进程可以重新申请。
                return 'SUMO 时钟已释放。'  # 返回释放结果字符串，并结束该请求的处理。
            if not owner_alive(self.clock['owner']):  # step 请求还要确认当前时钟所属进程没有退出。
                raise ValueError('时钟进程已经退出。')  # 进程已退出时拒绝推进，避免执行失效进程遗留的请求。
            if command['sequence'] != self.clock['sequence'] + 1:  # 要求新序号严格等于上次序号加 1；拒绝重复提交或跳过序号。
                raise ValueError('步进序号不连续，停止推进以免重复执行。')  # 序号不连续时抛出错误，CoBridge 收到失败后应停止推进。
            self.connection.simulationStep()  # 通过已持有的 TraCI 连接推进 SUMO 恰好一帧；本工程步长为 0.05 秒。
            self.clock['sequence'] = command['sequence']  # 步进调用成功后才登记新序号，表示这一帧已经执行。
            return dict(sequence=self.clock['sequence'], time=self.connection.simulation.getTime())  # 返回完成序号和更新后的仿真时间，供 CoBridge 校验。
        if action == 'stop':  # 处理要求关闭管理服务的请求，不是单纯释放时钟。
            self.stopped = True  # 设置退出标志；serve 主循环处理完回复后检查它并进入收尾。
            return 'SUMO 管理端正在关闭。'  # 先回复准备关闭，实际 socket 和 TraCI 清理由主循环退出后执行。
        if action == 'add_pois':  # 处理批量添加 POI 的请求，目前用于 RSU 图标。
            session, pois = command['session'], command['pois']  # 分别从请求字典取出会话标识和待添加的 POI 信息列表。
            if session in self.sessions:  # 检查这个会话 ID 是否已经登记。
                raise ValueError('该 RSU 会话已经注册。')  # 拒绝重复注册会话，避免覆盖已有会话的资源归属记录。
            if not owner_alive(command['owner']):  # 检查创建图标的 RSU 进程是否仍然存活。
                raise ValueError('RSU 请求进程已经退出。')  # 拒绝已经退出的 RSU 进程提交的创建请求。
            names = [poi['id'] for poi in pois]  # 列表推导式：从每个 POI 配置取出 id，得到所有待创建的名称。
            if len(names) != len(set(names)) or set(names) & set(self.connection.poi.getIDList()):  # 前半段检查本批 ID 是否重复；集合交集 & 检查是否与 SUMO 已有 ID 冲突。
                raise ValueError('RSU ID 重复或已被 SUMO 中的其他对象使用。')  # 检测到任意名称冲突就抛出异常，防止覆盖别人的图标。
            self.sessions[session] = dict(owner=command['owner'], ids=[])  # 先登记所属进程和空 ID 列表，创建过程中逐个记录已成功生成的图标。
            try:  # 尝试批量添加；任一操作失败时，下面的 except 回滚已经完成的部分。
                for poi in pois:  # 遍历本批 POI 配置，每个 poi 都是包含 ID、坐标、图片路径的字典。
                    self.connection.poi.add(  # 调用官方 TraCI POI 添加接口，下面几行是同一次调用的参数。
                        poi['id'], poi['x'], poi['y'], (255, 255, 255, 255),  # 传入 ID、SUMO x/y 坐标及 RGBA 白色不透明颜色，255 表示对应分量最大。
                        poiType='RSU', layer=100, imgFile=poi['image'], width=8, height=8)  # 指定类型、图层、图片路径和 8 米宽高，图标尺寸位于 SUMO 地图坐标中。
                    self.sessions[session]['ids'].append(poi['id'])  # 本次添加成功后记录该 ID，供正常退出、异常回滚和失效回收使用。
                    self.connection.poi.setParameter(poi['id'], 'name', poi['id'])  # 把 POI 的通用参数 name 设置为自身 ID；这是对象参数，不改变请求协议。
            except Exception:  # 捕获添加期间的普通异常，先清理，再通过下一条 raise 保留原始错误。
                self.remove_session(session)  # 删除本会话截至目前已经成功添加的图标，并删除会话记录。
                raise  # 不指定异常对象的 raise 会重新抛出刚捕获的异常，而不是吞掉错误。
            return names  # 全部创建成功后，返回本批图标 ID 列表。
        if action == 'remove_pois':  # 处理客户端主动要求删除某个 RSU 会话图标的请求。
            self.remove_session(command['session'])  # 按请求中的会话 ID 调用统一清理方法。
            return 'RSU POI 已清理。'  # 清理成功后返回提示字符串。
        raise ValueError(f'未知 SUMO 请求：{action}')  # 所有已知分支都未匹配，抛出未知操作错误并附上收到的 action。

    def poll(self):  # 定义周期维护方法；由主循环约每秒调用，而不是独立线程。
        # 只处理存活检查；simulationStep 只能响应时钟进程的显式请求。
        self.connection.getVersion()  # 查询 SUMO 版本以确认 TraCI 连接仍可用；断开会抛出异常。
        if self.clock and not owner_alive(self.clock['owner']):  # 如果持有时钟的进程已经退出，释放失效的所有权记录。
            self.clock = None  # 清空失效时钟记录，但不会自动调用 simulationStep。
        for session, state in list(self.sessions.items()):  # 把字典项转成列表快照再遍历，允许循环中安全删除原字典的会话。
            if not owner_alive(state['owner']):  # 逐个检查 RSU 会话所属进程是否仍然存活。
                self.remove_session(session)  # 所属进程已退出就清理这个会话的 SUMO 图标。


def serve(config_file, headless=False): # 主循环；定义常驻主服务：启动 SUMO、持有 TraCI、监听请求并负责退出清理。
    sumo_home = os.environ.get('SUMO_HOME')  # 从当前进程环境读取 SUMO_HOME；未设置时 get 返回 None。
    if not sumo_home:  # 检查环境变量是否为空或未定义。
        raise RuntimeError('请先设置 SUMO_HOME，使管理端能导入 tools/traci。')  # 没有 SUMO 安装位置就无法导入其工具，抛出明确的环境错误。
    sys.path.insert(0, str(Path(sumo_home) / 'tools'))  # 将 SUMO_HOME/tools 加入模块搜索路径，以加载官方 traci 包。
    import traci  # 导入官方 SUMO TraCI 库；与本工程 simuenv.sumo.traci 包不是同一个模块。

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)  # 确保运行目录存在，供文件锁和本地 socket 使用。
    with (RUNTIME_DIR / 'service.lock').open('a') as lock:  # 以追加模式打开锁文件；with 结束时关闭文件，系统也会释放文件锁。
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)  # LOCK_EX 请求独占锁；LOCK_NB 表示不等待，若别的服务已持锁则立即报错。
        SOCKET_FILE.unlink(missing_ok=True)  # 拿到独占锁后删除旧 socket 文件节点；missing_ok=True 允许文件不存在。
        network = read_network(config_file)  # 调用本文件函数读取配置、实际路网偏移及地图指纹。
        network['headless'] = headless  # 把本次是否无窗口运行也加入服务返回的地图状态字典。
        binary = 'sumo' if headless else 'sumo-gui'  # 条件表达式：headless 为真选 sumo，否则选带界面的 sumo-gui。
        # SUMO 日志仍放在地图目录；管理端自身的输出另存 service.log。
        command = [binary, '-c', str(config_file), '--quit-on-end', 'false',  # 开始组装 SUMO 命令：-c 指定配置；--quit-on-end false 不要求 GUI 在结束时自动退出。
                   '--step-length', str(STEP_LENGTH),  # 将公共步长转为字符串传给 SUMO，覆盖配置中的步长设置。
                   '--log', str(config_file.parent / 'sumo.log')]  # 把 SUMO 自己的日志写入配置所在目录下的 sumo.log。
        if not headless:  # 只有 GUI 模式需要添加下面的自动启动选项。
            command += ['--start']  # --start 让 GUI 加载完成后开始运行；TraCI 仍需收到明确步进请求才推进时间。
        connection = None  # 先标记尚无连接，便于 finally 判断是否需要调用 TraCI 关闭接口。
        try:  # 开始管理 SUMO 运行过程，后面的 finally 无论正常结束还是异常都执行。
            traci.start(command, label='managed', numRetries=5)  # 启动 SUMO 子进程并建立 TraCI 连接，使用 managed 标签，设置连接重试次数。
            connection = traci.getConnection('managed')  # 按标签取得官方 TraCI 连接对象，后续所有 SUMO 操作共用它。
            service = SumoService(connection, network)  # 创建服务状态对象，保存连接、地图信息、RSU 会话和时钟状态。
            for signum in (signal.SIGINT, signal.SIGTERM):  # 依次为 Ctrl+C 的 SIGINT 和正常终止的 SIGTERM 注册处理函数。
                signal.signal(signum, lambda *_: setattr(service, 'stopped', True))  # lambda *_ 接收并忽略信号回调参数；setattr 只设置退出标志，不立即强退进程。
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:  # 创建 UNIX 字节流监听 socket，用 server 操作；with 退出时关闭监听端点。
                server.bind(str(SOCKET_FILE))  # 将监听 socket 绑定到本机文件路径，由内核创建对应 socket 文件节点。
                os.chmod(SOCKET_FILE, 0o600)  # 0o600 是八进制权限：只有当前文件所有者可读写，其他用户无权限。
                server.listen(8)  # 启动监听，8 是待接受连接的队列容量，不是同时运行八个请求处理线程。
                server.settimeout(.2)  # accept 等待新连接最多约 0.2 秒，以便循环及时检查退出标志和维护任务。
                print(f'TraCI 管理端已就绪：{config_file}', flush=True)  # 打印就绪消息，flush=True 立即刷新输出缓冲，后台运行时通常写入管理日志。
                next_poll = 0  # 初始化下次维护时刻为 0，让主循环第一次运行时立即维护。
                while not service.stopped:  # 只要没有收到关闭请求或退出信号，就持续串行处理请求。
                    if time.monotonic() >= next_poll:  # 比较单调时钟，判断是否到达下一次周期维护时刻。
                        service.poll()  # 执行连接存活、时钟所有者和 RSU 所属进程的检查。
                        next_poll = time.monotonic() + 1  # 本次维护完成后，把下一次检查安排在约 1 秒后。
                    try:  # 尝试接受一个客户端连接；没有连接到达时可能超时。
                        client, _ = server.accept()  # accept 返回通信 socket 和对端地址；本代码不使用地址，因此用 _ 接收。
                    except socket.timeout:  # 捕获等待客户端连接的超时；没有新连接不是服务错误。
                        continue  # 回到 while 顶部，继续检查退出条件及维护任务。
                    with client:  # 用 with 管理本次客户端通信 socket，完成一个请求后自动关闭它。
                        client.settimeout(3)  # 为本次客户端 socket 的接收和发送等阻塞操作设置 3 秒超时，不是业务执行总时限。
                        try:  # 尝试接收、解析并执行请求，指定异常会被包装为失败回复。
                            with client.makefile('rb') as stream:  # 给 socket 创建二进制读取接口；rb 表示读字节，不会创建磁盘文件。
                                command = json.loads(stream.readline(262144))  # 最多读 256 KiB，遇换行或 EOF 可结束；json.loads 把 JSON 字节解析成 Python 对象。
                            reply = dict(ok=True, result=service.handle(command))  # 先调用 handle 执行业务，再包装成项目约定的成功回复字典。
                        except (ValueError, KeyError, OSError, traci.TraCIException) as error:  # 捕获格式、缺失键、系统通信及 TraCI 操作错误，并把异常对象命名为 error。
                            reply = dict(ok=False, error=str(error))  # 创建失败回复，将原始错误说明转为字符串放在 error 字段。
                        try:  # 尝试给当前请求方发送回复；即使对方断开，也不应影响后续客户端。
                            client.sendall((json.dumps(reply) + '\n').encode())  # 回复字典转 JSON，加换行作为消息边界，编码为字节后全部发送。
                        except OSError:  # 捕获发送时对方断开、超时等系统通信错误。
                            pass  # 请求方断开不影响其他模块。；忽略本次回复发送失败，继续服务其他请求；业务操作可能已经执行成功。
        finally:  # 服务主循环无论如何结束，都执行这里的资源清理。
            if connection is not None:  # 只有确实取得过 TraCI 连接时才尝试关闭，避免操作未初始化对象。
                try:  # 尝试关闭连接，关闭时发生的普通异常只记录，不阻碍 socket 路径清理。
                    connection.close(wait=False)  # 通知 SUMO 关闭 TraCI 连接；wait=False 不等待 SUMO 进程或 GUI 窗口退出。
                except Exception as error:  # 捕获关闭期间的普通异常，以 error 保存具体原因。
                    print(f'TraCI 关闭信息：{error}', flush=True)  # 记录 TraCI 关闭信息并立即刷新，避免退出时日志留在缓冲区。
            SOCKET_FILE.unlink(missing_ok=True)  # 删除当前服务的 socket 文件节点，防止后续客户端连到遗留地址。


def main():  # 定义命令行入口函数；函数定义本身不会启动服务。
    """管理命令留在服务模块，不放进 sumo_init.py。"""
    import argparse  # 导入 Python 自带的命令行参数解析库。
    parser = argparse.ArgumentParser(description=__doc__)  # 创建参数解析器；description=__doc__ 使用文件顶部的模块说明。
    parser.add_argument('config', nargs='?', type=Path, help='要加载的 .sumocfg')  # 定义可选的位置参数 config；nargs=? 允许不填，填写时转换成 Path 对象。
    operations = parser.add_mutually_exclusive_group()  # 创建互斥参数组，让 --status 和 --stop 不能同时使用。
    operations.add_argument('--status', action='store_true', help='查询管理端状态')  # 定义 --status 开关，出现时 args.status 为 True，否则为 False。
    operations.add_argument('--stop', action='store_true', help='关闭管理端及其 SUMO')  # 定义 --stop 开关，出现时向已有服务发送关闭请求。
    parser.add_argument('--headless', action='store_true', help='无窗口测试模式')  # 定义 --headless 开关，启动服务时选择不显示 GUI 的 SUMO。
    args = parser.parse_args()  # 读取当前命令行参数，校验后放入 args 对象供下面分支使用。
    if args.status:  # 如果用户选择查询模式，就只联系已有服务，不启动第二个管理端。
        print(json.dumps(request({'action': 'status'}), ensure_ascii=False, indent=2))  # 查询状态并打印 JSON；ensure_ascii=False 保留中文，indent=2 缩进便于阅读。
    elif args.stop:  # 如果用户选择停止模式，向已有服务请求正常退出。
        request({'action': 'stop'})  # 发送 stop 请求；收到回复表示服务接受请求，不表示所有退出清理都已完成。
        deadline = time.monotonic() + 10  # 设置约 10 秒的退出观察窗口，用单调时钟计时。
        while SOCKET_FILE.exists() and time.monotonic() < deadline:  # 只要 socket 文件仍在且尚未超时，就继续等待服务删除地址节点。
            time.sleep(.1)  # 每次等待 0.1 秒，避免反复检查文件造成忙循环。
        if SOCKET_FILE.exists():  # 超时后再检查 socket 节点是否仍然存在。
            raise RuntimeError('SUMO 管理端尚未退出，请查看 service.log。')  # 节点未被清理，报告服务可能尚未退出，提示查看管理日志。
        print('SUMO 管理端已关闭。')  # socket 节点已消失，打印管理端关闭提示；本检查并不直接等待 GUI 进程。
    else:  # 没有指定查询或停止时，默认按启动服务处理。
        if args.config is None:  # 启动服务必须提供配置文件路径，None 表示没有填写该位置参数。
            parser.error('启动服务时需要填写 .sumocfg 路径。')  # 通过 argparse 输出用法错误并退出，默认退出码是 2。
        serve(args.config.resolve(), args.headless)  # 将配置路径转为绝对路径，并进入常驻服务；该函数会持续运行到停止或异常。


if __name__ == '__main__':  # 仅直接运行此文件时进入；通过 import 导入时不执行下面的启动逻辑。
    try:  # 统一处理命令行入口中的退出和预期错误。
        main()  # 调用上面定义的 main，实际解析参数并执行对应操作。
    except KeyboardInterrupt:  # 捕获未被其他地方处理的 Ctrl+C 中断异常。
        raise SystemExit(130)  # 以 130 退出码结束，表示这次退出由键盘中断触发。
    except (RuntimeError, OSError, ImportError, ValueError) as error:  # 捕获已知运行、系统、导入和数值类错误，不掩盖其他未预料的编程错误。
        raise SystemExit(f'错误：{error}')  # 将异常原因转换成简短错误提示，以 SystemExit 字符串结束时通常退出码为 1。
