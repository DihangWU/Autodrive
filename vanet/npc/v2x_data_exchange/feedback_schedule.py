"""状态变化立即反馈；周期反馈保持固定相位，避免与紧急广播持续同刻碰撞。"""
import math
from .beacon_schedule import beacon_phase_seconds


def next_feedback_time(now,node_id,config):
    interval=config.get('feedback_interval',1.)
    spread=config.get('beacon_phase_spread_seconds',0.)
    if not spread:return now+interval
    phase=beacon_phase_seconds(node_id+':feedback',config.get('seed',0),spread)%interval
    return phase+(math.floor((now-phase+1e-8)/interval)+1)*interval
