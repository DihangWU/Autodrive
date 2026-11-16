"""收到紧急任务只反馈接收，不修改 SUMO 驾驶状态。"""
import logging
from vanet.npc.v2x_data_exchange.feedback_schedule import next_feedback_time

LOG = logging.getLogger(__name__)


def communication_only_step(component, now, own_state, messages):
    if component.task is None:
        return messages
    # No driving action needs a 30-second out-of-range hold in this mode.
    current = component.range_leases.active(now) if component.range_leases.enabled else None
    expired = current is None if component.range_leases.enabled else now >= component.lease_until
    if component.recovering or expired:
        component.task = None
        component.status, component.reason = 'normal', 'yielding_disabled_for_scenario'
        component.recovering = False
        component.conflict_anchor = None
        component.range_leases.clear()
        return messages
    if current is not None:
        component.task, component.lease_until = current['message'], current['until']
    if (component.status,component.reason)!=('received','yielding_disabled_for_scenario'):
        LOG.info('NPC communication only node=%s task=%s sim_time=%.3f; normal SUMO driving retained',
                 component.node_id,component.task['task_id'],now)
    component.status, component.reason = 'received', 'yielding_disabled_for_scenario'
    if now + 1e-8 >= component.next_feedback:
        component.append_feedback(messages,now,own_state,'received',component.reason)
        component.next_feedback = next_feedback_time(now,component.node_id,component.config)
    return messages
