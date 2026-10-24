"""只管理本次车辆、线程、ROS 2 和窗口，不启动其他仿真进程。"""
import logging
import signal
import threading
import time

from foundation.module_status import ModuleReporter
from modules.lincoln.runtime.paths import RUNTIME_DIR

SENSOR_LOG_NAME = 'lincoln.log'
CONTROL_INTERVAL = 0.03


def run_sensor_executor(executor, node):
    try:
        executor.spin()
    except Exception:
        logging.exception('传感器发布线程异常')
        node.stop_requested = True


def run_vehicle_control_loop(node, stopped):
    """周期检查 TM 模式；控制算法只在收到新 CARLA 帧时执行一次。"""
    last_frame = None
    try:
        while not stopped.is_set():
            try:
                snapshot = node.world.wait_for_tick(.5)
            except RuntimeError:
                continue
            if snapshot.frame != last_frame:
                node.update_automatic_driving(snapshot)
                node.update_camera(snapshot)
                last_frame = snapshot.frame
            if not node.car.is_alive or node.client.get_world().id != node.world.id:
                node.stop_requested = True
                return
            node.drive()
            stopped.wait(CONTROL_INTERVAL)
    except Exception:
        logging.exception('驾驶/视角线程异常')
        node.stop_requested = True


def run_lincoln_application(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from modules.lincoln.runtime.lincoln_node import LincolnNode
    from modules.lincoln.integration.bridge_registration import LincolnBridgeRegistration
    from modules.lincoln.visualization.rviz_launcher import launch_vehicle_rviz

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=RUNTIME_DIR / SENSOR_LOG_NAME, level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.set())
    # ROS 不安装自己的 SIGINT 处理器，避免与入口的退出事件竞争。
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = executor = registration = reporter = None
    workers = []
    started = None
    try:
        node = LincolnNode(args)
        reporter = ModuleReporter(node.uid, args.bridge_socket)
        registration = LincolnBridgeRegistration(node, args.bridge_socket, reporter)
        if args.autopilot:
            print(f'[{node.uid}] 正在准备驾驶控制器：{args.driver}', flush=True)
            node.set_autopilot(True)
        if not args.no_rviz:
            node.rviz = launch_vehicle_rviz(node)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        workers = [threading.Thread(target=run_sensor_executor, args=(executor, node), name='lincoln-sensors'),
                   threading.Thread(target=run_vehicle_control_loop, args=(node, stopped), name='lincoln-control')]
        for worker in workers:
            worker.start()

        started = time.monotonic()

        def done():
            return (stopped.is_set() or node.stop_requested or not rclpy.ok()
                    or (args.duration > 0 and time.monotonic() - started >= args.duration))

        if args.headless:
            while not done():
                stopped.wait(CONTROL_INTERVAL)
        else:
            from modules.lincoln.ui.driving_panel import run_mouse_driving_panel
            run_mouse_driving_panel(node, done)
    finally:
        stopped.set()
        if executor is not None:
            executor.shutdown(timeout_sec=5)
        for worker in workers:
            if worker.ident is not None:
                worker.join()
        if registration is not None:
            registration.close()
        if reporter is not None:
            reporter.close()
        if node is not None:
            try:
                node.cleanup()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
