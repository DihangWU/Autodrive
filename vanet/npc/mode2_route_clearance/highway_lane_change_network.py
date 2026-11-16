"""统一平行四车道内部连接的逻辑长度，解除 SUMO 的跨线结构限制。"""


def normalize_parallel_internal_lengths(root):
    entries={connection.get('via'):connection for connection in root.findall('connection')
             if connection.get('via') and not connection.get('from','').startswith(':')}
    changed=[]
    for edge in root.findall("edge[@function='internal']"):
        lanes=edge.findall('lane')
        if len(lanes)<4:continue
        connections=[entries.get(lane.get('id')) for lane in lanes]
        if any(connection is None for connection in connections):continue
        # Only parallel through lanes between the same pair of road edges;
        # never join diverging turns or sibling lanes sharing an origin.
        if len({(c.get('from'),c.get('to')) for c in connections})!=1:continue
        if len({c.get('fromLane') for c in connections})!=len(lanes):continue
        if len({c.get('toLane') for c in connections})!=len(lanes):continue
        lengths={lane.get('id'):lane.get('length') for lane in lanes}
        length=max(lengths.values(),key=float)
        if all(value==length for value in lengths.values()):continue
        for lane in lanes:lane.set('length',length)
        changed.append(dict(edge=edge.get('id'),original_lengths=lengths,length=length))
    return changed
