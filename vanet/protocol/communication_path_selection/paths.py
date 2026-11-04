"""几何信息仅选择直连或 RSU 路径，避让仍必须由实际无线收包触发。"""
import math


def select_communication_paths(states, rsu, ranges):
    lincoln = next((state for state in states if state['role'] == 'lincoln'), None)
    paths = {state['id']:None for state in states if state['role'] == 'npc'}
    if lincoln is None:
        return paths, False
    distance = lambda a,b: math.hypot(a['x']-b['x'], a['y']-b['y'])
    connected = distance(lincoln,rsu) <= ranges['rsu_metres']+1e-9
    for state in states:
        if state['role'] != 'npc':continue
        if distance(lincoln,state) <= ranges['v2v_metres']+1e-9:
            paths[state['id']] = 'direct'
        elif connected and distance(state,rsu) <= ranges['rsu_metres']+1e-9:
            paths[state['id']] = 'rsu'
    return paths, connected
