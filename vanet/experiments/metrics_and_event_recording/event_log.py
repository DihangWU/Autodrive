"""逐事件 JSONL 和可读日志；每次会话独立，不覆盖历史记录。"""
import csv
import json
import logging
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from vanet.project_paths import (EXPERIMENT_RESULTS_DIRECTORY,EXPERIMENT_OMNET_RELATIVE_DIRECTORY,
    EXPERIMENT_LOGS_RELATIVE_DIRECTORY,EXPERIMENT_EVENTS_RELATIVE_FILE,
    EXPERIMENT_CONFIGURATION_RELATIVE_FILE,EXPERIMENT_METRICS_RELATIVE_FILE,
    EXPERIMENT_NETWORK_BOUNDARIES_RELATIVE_FILE,EXPERIMENT_ARRIVAL_METRICS_RELATIVE_FILE)


def new_result_directory(prefix='run'):
    identifier = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:6]
    folder = EXPERIMENT_RESULTS_DIRECTORY / f'{prefix}_{identifier}'
    folder.mkdir(parents=True, exist_ok=False)
    (folder / EXPERIMENT_LOGS_RELATIVE_DIRECTORY).mkdir()
    (folder / EXPERIMENT_OMNET_RELATIVE_DIRECTORY).mkdir()
    return folder


class EventLog:
    def __init__(self, folder, component='experiment'):
        self.folder = Path(folder)
        self.component = component
        self.lock = threading.Lock()
        self.stream = (self.folder / EXPERIMENT_EVENTS_RELATIVE_FILE).open('a', buffering=1)
        self.network_stream = None
        self.logger = logging.getLogger(f'vanet.{component}.{self.folder.name}')
        self.logger.setLevel(logging.DEBUG)
        handler = logging.FileHandler(self.folder / EXPERIMENT_LOGS_RELATIVE_DIRECTORY / f'{component}.log')
        handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
        self.logger.addHandler(handler)
        self.logger.propagate = False
        self.handler = handler

    def emit(self, event, sim_time=None, level='info', **fields):
        record = dict(wall_time=datetime.now(timezone.utc).isoformat(), sim_time=sim_time,
                      component=self.component, event=event, **fields)
        line = json.dumps(record, ensure_ascii=False, allow_nan=False, default=str)
        with self.lock:
            self.stream.write(line + '\n')
            getattr(self.logger, level)(line)
        if level != 'debug':
            print(f'[{self.component}] t={sim_time} {event} {json.dumps(fields, ensure_ascii=False, default=str)}', flush=True)
        return record

    def save_config(self, config):
        target = self.folder / EXPERIMENT_CONFIGURATION_RELATIVE_FILE
        target.write_text(json.dumps(config, indent=2, ensure_ascii=False, allow_nan=False) + '\n')

    def save_metrics(self, metrics):
        targets=[self.folder/EXPERIMENT_METRICS_RELATIVE_FILE]
        arrival=self.folder/EXPERIMENT_ARRIVAL_METRICS_RELATIVE_FILE
        if metrics.get('arrived') and not arrival.exists():targets.append(arrival)
        for target in targets:
            with target.open('w', newline='') as output:
                writer = csv.DictWriter(output, fieldnames=list(metrics))
                writer.writeheader()
                writer.writerow(metrics)

    def network_boundary(self, result):
        if self.network_stream is None:
            self.network_stream = (self.folder / EXPERIMENT_NETWORK_BOUNDARIES_RELATIVE_FILE).open('a',buffering=1)
        self.network_stream.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')

    def close(self):
        if self.network_stream:self.network_stream.close()
        self.stream.close()
        self.logger.removeHandler(self.handler)
        self.handler.close()
