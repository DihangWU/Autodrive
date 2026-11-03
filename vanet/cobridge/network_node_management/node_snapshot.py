"""Validate externally owned radio node identities and CARLA-world positions."""
import math


def network_node(state, role='npc'):
    node = {key: state[key] for key in ('id', 'x', 'y', 'z')}
    node['role'] = state.get('role', role)
    node['speed'] = float(state.get('speed', 0))
    node['heading'] = float(state.get('heading', 0))
    if not isinstance(node['id'], str) or not node['id'] or len(node['id']) > 128:
        raise ValueError('Radio node id must contain 1..128 characters')
    if node['role'] not in ('lincoln', 'npc', 'rsu'):
        raise ValueError('Unknown network node role')
    for name in ('x', 'y', 'z', 'speed', 'heading'):
        node[name] = float(node[name])
        if not math.isfinite(node[name]):
            raise ValueError(f'Non-finite node {name}')
    return node


def network_snapshot(states):
    nodes = [network_node(state) for state in states]
    ids = [node['id'] for node in nodes]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate network node identity')
    if sum(node['role'] == 'lincoln' for node in nodes) > 1 or sum(node['role'] == 'rsu' for node in nodes) > 1:
        raise ValueError('This experiment supports one Lincoln and one RSU')
    return nodes
