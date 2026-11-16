"""普通交通分流：按第 4 车道实际到达顺序应用配置的直行/右转周期。"""
import logging

LOG = logging.getLogger(__name__)


def value(state, variable):
    return state.get(str(variable),state.get(variable))


class JunctionRouteSplit:
    def __init__(self, connection):
        self.connection = connection
        self.decisions = []
        self.assigned = set()

    def reset(self):
        self.decisions.clear()
        self.assigned.clear()

    def update(self, created, states, constants, sim_time):
        candidates = []
        for name, record in created.items():
            policy = record['scenario'].get('junction_departure')
            state = states.get(name)
            if not policy or name in self.assigned or not isinstance(state,dict):
                continue
            road = value(state,constants.VAR_ROAD_ID)
            approaches = policy['approach_edges']
            if road not in approaches:
                continue
            position = value(state,constants.VAR_LANEPOSITION)
            if position is None:
                continue
            # Same-frame arrivals are sorted physically front-to-back, not by id.
            candidates.append((-approaches.index(road),-float(position),name,policy,road))
        for _,_,name,policy,road in sorted(candidates):
            sequence = len(self.decisions)+1
            pattern=policy.get('departure_pattern',['straight','city'])
            branch = pattern[(sequence-1)%len(pattern)]
            remaining = policy['approach_edges'][policy['approach_edges'].index(road):]
            route = remaining+policy[branch+'_suffix']
            self.connection.vehicle.setRoute(name,route)
            self.connection.vehicle.setParameter(name,'vanet.scenario.departure',branch)
            self.assigned.add(name)
            record = dict(vehicle=name,sequence=sequence,branch=branch,road=road,route=route)
            self.decisions.append(record)
            LOG.info('HIGHWAY_SPLIT sim_time=%.3f sequence=%s vehicle=%s branch=%s route=%s',
                     sim_time,sequence,name,branch,route)

    def snapshot(self):
        return dict(assigned=len(self.decisions),straight=sum(v['branch']=='straight' for v in self.decisions),
                    city=sum(v['branch']=='city' for v in self.decisions),decisions=list(self.decisions))
