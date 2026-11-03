"""RSU 终端只读状态轮询；不接收无线、不推进时钟、不阻塞实体显示线程。"""
from collections import Counter
import json
import threading
import time
from foundation.cobridge.control import ControlClient

STATUS_LABELS = {'normal':'正常','received':'已收到','yielding':'避让中','cleared':'已清空',
                 'unable':'无法避让','recovering':'恢复中'}


def summarize_status(status):
    rsu = status.get('rsu') or {}
    counters = rsu.get('counters') or {}
    npc_states = status.get('npc_states') or {}
    if isinstance(npc_states,list):
        npc_states = {item['id']:item for item in npc_states if isinstance(item,dict) and 'id' in item}
    states = {name:(value if isinstance(value,str) else value.get('status','unknown'))
              for name,value in npc_states.items()}
    feedback = {name:(value if isinstance(value,str) else value.get('status','unknown'))
                for name,value in (rsu.get('feedback') or {}).items()}
    return dict(phase=status.get('phase','unknown'),group=status.get('group','unknown'),
                rsu_id=rsu.get('id','rsu0'),task_id=rsu.get('task_id') or '',
                beacon_count=rsu.get('beacon_count',0),selected=sorted(rsu.get('selected') or []),
                received=counters.get('received',0),notice_sent=counters.get('notice_sent',0),
                feedback_received=counters.get('feedback_received',0),
                clearance_sent=counters.get('clearance_sent',0),backfill_sent=counters.get('backfill_sent',0),
                npc_states=states,npc_counts=dict(Counter(states.values())),feedback=feedback,
                detail=status.get('detail',''))


def render_status(summary, simulation_time=None):
    count_text=', '.join(f'{STATUS_LABELS.get(name,name)}={count}' for name,count in sorted(summary['npc_counts'].items())) or '无 NPC 状态'
    feedback_text=', '.join(f'{name}:{STATUS_LABELS.get(value,value)}' for name,value in sorted(summary['feedback'].items())) or '暂无'
    stamp='未建立' if simulation_time is None else f'{float(simulation_time):.2f}s'
    return (f"VANET t={stamp} 阶段={summary['phase']} 组={summary['group']} RSU={summary['rsu_id']} "
            f"任务={summary['task_id'] or '无'} 已知信标车辆={summary['beacon_count']} "
            f"收包={summary['received']} 定向通知={summary['notice_sent']} 回补={summary['backfill_sent']} "
            f"收到反馈={summary['feedback_received']} 清空汇总={summary['clearance_sent']} "
            f"选中={','.join(summary['selected']) or '无'} NPC[{count_text}] 反馈[{feedback_text}] "
            f"{summary['detail']}")


class RsuTerminalStatus:
    def __init__(self, console, path, interval=1., timeout=.5, client_factory=ControlClient):
        self.console,self.path = console,path
        self.interval,self.timeout = interval,timeout
        self.client_factory = client_factory
        self.client = None
        self.previous = None
        self.last_error = None
        self.stopped = threading.Event()
        self.thread = None

    def poll_once(self):
        try:
            if self.client is None:
                self.client=self.client_factory(self.path,self.timeout)
            status=self.client.command('vanet.status')
            if not isinstance(status,dict):
                raise ValueError('CoBridge VANET 状态不是对象')
            summary=summarize_status(status)
            key=json.dumps(summary,sort_keys=True,ensure_ascii=False)
            if self.last_error is not None:
                self.console.info('VANET 状态连接已恢复。')
                self.previous=None
            self.last_error=None
            if key!=self.previous:
                self.console.info(render_status(summary,status.get('simulation_time')))
                self.previous=key
            return status
        except Exception as error:
            message=str(error)
            if message!=self.last_error:
                self.console.info(f'VANET 状态暂不可用：{message}')
                self.last_error=message
            if self.client is not None:
                try:self.client.close()
                except Exception:pass
                self.client=None
            return None

    def start(self):
        if self.thread is None:
            self.thread=threading.Thread(target=self.run,name='rsu-vanet-status',daemon=True)
            self.thread.start()
        return self

    def run(self):
        try:
            while not self.stopped.is_set():
                started=time.monotonic()
                self.poll_once()
                self.stopped.wait(max(0.,self.interval-(time.monotonic()-started)))
        finally:
            if self.client is not None:
                self.client.close()
                self.client=None

    def close(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(self.timeout+1.)
        elif self.client is not None:
            self.client.close()
            self.client=None
