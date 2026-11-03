"""由实验种子和节点身份确定信标首次发送相位，不改变后续发送周期。"""
import hashlib
import math


def beacon_phase_seconds(node_id, seed=0, spread_seconds=0.0):
    """使用稳定 SHA-256 映射，避免 Python hash 的进程间随机化。"""
    spread_seconds = float(spread_seconds)
    if not math.isfinite(spread_seconds) or spread_seconds < 0:
        raise ValueError('beacon_phase_spread_seconds must be finite and nonnegative')
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('Beacon phase seed must be a nonnegative integer')
    if not isinstance(node_id, str) or not node_id:
        raise ValueError('Beacon phase node_id must be a nonempty string')
    identity = f'vanet.beacon.phase.v1\0{seed}\0{node_id}'.encode('utf-8')
    # 53 bits are exactly representable as a float; phase stays below the upper bound.
    fraction = (int.from_bytes(hashlib.sha256(identity).digest()[:8], 'big') >> 11) / (1 << 53)
    return spread_seconds * fraction
