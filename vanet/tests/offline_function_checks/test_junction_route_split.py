"""按实际到达顺序分流；同帧车辆按道路进度排序，复位后从直行重新开始。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.npc.highway_background_traffic.junction_route_split import JunctionRouteSplit


class JunctionRouteSplitTests(unittest.TestCase):
    def test_front_to_back_assignment_is_alternating_and_not_repeated(self):
        c=Mock();split=JunctionRouteSplit(c)
        tc=SimpleNamespace(VAR_ROAD_ID=80,VAR_LANEPOSITION=86)
        policy=dict(approach_edges=['-47','-48'],straight_suffix=['-49','-50'],city_suffix=['-7','-8'])
        created={name:dict(scenario=dict(junction_departure=policy)) for name in ('rear','front','middle')}
        states={'rear':{80:'-47',86:90.},'front':{'80':'-48','86':10.},'middle':{80:'-48',86:5.}}
        split.update(created,states,tc,10.)
        self.assertEqual([(v['vehicle'],v['branch']) for v in split.decisions],
                         [('front','straight'),('middle','city'),('rear','straight')])
        self.assertEqual(split.decisions[0]['route'],['-48','-49','-50'])
        self.assertEqual(split.decisions[2]['route'],['-47','-48','-49','-50'])
        split.update(created,states,tc,10.05)
        self.assertEqual(c.vehicle.setRoute.call_count,3)
        c.vehicle.changeLane.assert_not_called()
        c.vehicle.setSpeed.assert_not_called()
        split.reset();split.update(created,{'rear':{80:'-47',86:20.}},tc,0.)
        self.assertEqual(split.decisions[0]['branch'],'straight')

    def test_no_route_changes_before_the_approach_or_for_other_vehicles(self):
        c=Mock();split=JunctionRouteSplit(c)
        tc=SimpleNamespace(VAR_ROAD_ID=80,VAR_LANEPOSITION=86)
        created={'ordinary':dict(scenario={}), 'pending':dict(scenario=dict(junction_departure={
            'approach_edges':['-47','-48']}))}
        split.update(created,{'ordinary':{80:'-48',86:50.},'pending':{80:'-41',86:500.}},tc,1.)
        c.vehicle.setRoute.assert_not_called()


if __name__=='__main__':unittest.main()
