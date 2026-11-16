"""将已收到的 CARLA 规划路线匹配到 SUMO 信号连接；不修改路网。"""
import math
import xml.etree.ElementTree as ET

from vanet.npc.route_conflict_assessment.route_geometry import project


class RouteSignalIndex:
    def __init__(self, network_file):
        root=ET.parse(network_file).getroot()
        self.offset=tuple(float(v) for v in root.find('location').get('netOffset').split(','))
        self.shapes={lane.get('id'):[self.world_point(value) for value in lane.get('shape').split()]
                     for edge in root.findall('edge') for lane in edge.findall('lane')}
        self.connections=root.findall('connection')
        self.successors={}
        for link in self.connections:
            self.successors.setdefault(link.get('from')+'_'+link.get('fromLane'),[]).append(link)
        self.groups={}
        self.signal_counts={}
        for logic in root.findall('tlLogic'):
            name=logic.get('id')
            self.signal_counts[name]=len(logic.find('phase').get('state'))
            groups={}
            for entry in logic.findall('param'):
                if entry.get('key','').startswith('linkSignalID:'):
                    groups.setdefault(entry.get('value'),set()).add(int(entry.get('key').split(':')[1]))
            self.groups[name]=list(groups.values())
        self.movements=[]
        for link in self.connections:
            if not link.get('tl'):continue
            incoming=link.get('from')+'_'+link.get('fromLane')
            outgoing=link.get('to')+'_'+link.get('toLane')
            via=link.get('via');path=[];seen=set()
            while via and via not in seen:
                seen.add(via);path.extend(self.shapes[via])
                continuation=next((item for item in self.successors.get(via,[])
                                   if item.get('to')==link.get('to') and item.get('toLane')==link.get('toLane')),None)
                via=continuation.get('via') if continuation is not None else None
            if not path:continue
            approach=self.shapes[incoming];exit_shape=self.shapes[outgoing]
            # Approach and exit direction distinguish turning/straight and
            # opposing movements, even when they meet at the same junction.
            samples=[self.near_endpoint(approach[-1],approach[-2]),approach[-1]]
            samples += [path[round(i*(len(path)-1)/4)] for i in range(5)]
            samples += [exit_shape[0],self.near_endpoint(exit_shape[0],exit_shape[1])]
            self.movements.append(dict(controller=link.get('tl'),link_index=int(link.get('linkIndex')),
                                       incoming=incoming,outgoing=outgoing,samples=samples))

    def world_point(self, value):
        coordinates=[float(v) for v in value.split(',')]
        return dict(x=coordinates[0]-self.offset[0],y=self.offset[1]-coordinates[1],
                    z=coordinates[2] if len(coordinates)>2 else 0.)

    @staticmethod
    def near_endpoint(origin, toward):
        distance=math.hypot(toward['x']-origin['x'],toward['y']-origin['y'])
        fraction=min(1.,3./max(distance,.001))
        return {key:origin[key]+fraction*(toward[key]-origin[key]) for key in ('x','y','z')}

    def match(self, route, tolerance=2.5):
        matches={}
        for movement in self.movements:
            projections=[]
            for point in movement['samples']:
                projection=project(point,route)
                if projection is None or projection['distance']>tolerance or projection['height']>2.:break
                projections.append(projection)
            if len(projections)!=len(movement['samples']):continue
            stations=[p['station'] for p in projections]
            if stations[-1]-stations[0]<3. or any(b<a-.5 for a,b in zip(stations,stations[1:])):continue
            # Positive entry/exit progress rules out perpendicular paths that
            # only touch a stop line or the destination endpoint.
            if stations[1]-stations[0]<.5 or stations[-1]-stations[-2]<.5:continue
            name=movement['controller'];score=max(p['distance'] for p in projections)
            if name in matches and matches[name]['score']<=score:continue
            indices={movement['link_index']}
            for group in self.groups.get(name,[]):
                if indices & group:indices.update(group)
            green=''.join('G' if index in indices else 'r' for index in range(self.signal_counts[name]))
            matches[name]=dict(controller=name,link_index=movement['link_index'],green_indices=sorted(indices),
                incoming=movement['incoming'],outgoing=movement['outgoing'],green_state=green,
                entry_station=stations[1],exit_station=stations[-2],score=score)
        return sorted(matches.values(),key=lambda item:item['entry_station'])
