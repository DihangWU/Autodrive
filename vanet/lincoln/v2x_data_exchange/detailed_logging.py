"""Lincoln 会话详细日志：现实时间、仿真时间、帧号和任务上下文。"""
import logging
from vanet.project_paths import VANET_SESSION_LOG_DIRECTORY


class ControlContext(logging.Filter):
    def __init__(self,service):
        super().__init__()
        self.service=service

    def filter(self,record):
        record.sim_time=self.service.simulation_time
        record.frame=self.service.ack_frame
        record.task_id=self.service.task.task_id
        return True


def initialize_logging(service):
    VANET_SESSION_LOG_DIRECTORY.mkdir(parents=True,exist_ok=True)
    handler=logging.FileHandler(VANET_SESSION_LOG_DIRECTORY/'Lincoln.events.log')
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s sim_time=%(sim_time)s frame=%(frame)s task=%(task_id)s %(name)s %(message)s'))
    handler.addFilter(ControlContext(service))
    logger=logging.getLogger('vanet.lincoln')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    return handler
