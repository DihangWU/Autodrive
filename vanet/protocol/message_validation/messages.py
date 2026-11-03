"""程序边界的输入校验；拒绝非有限数、过期版本与未知实验组。"""
import math

GROUPS = ('normal', 'emergency', 'cooperative')
WIRELESS_TYPES = {'beacon', 'emergency', 'notice', 'feedback', 'clearance', 'cancel', 'identity'}


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{name} 必须为有限数值')
    return float(value)


def position(value):
    return {key: finite_number(value[key], key) for key in ('x', 'y', 'z')}


def experiment_group(value):
    if value not in GROUPS:
        raise ValueError(f'实验组必须为 {GROUPS}')
    return value


def wireless_message(value):
    if not isinstance(value, dict) or value.get('type') not in WIRELESS_TYPES:
        raise ValueError('无线消息类型无效')
    if not isinstance(value.get('source'), str) or not value['source']:
        raise ValueError('无线消息缺少来源')
    finite_number(value.get('sent_at'), 'sent_at')
    return value
