"""滚动合流只过滤来得及让出安全窗口的后车，保留近车并恢复原参数。"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.highway_ramp_merging.rolling_merge_priority import RollingMergePriority

TC=SimpleNamespace(VAR_LANE_ID=81,VAR_LANEPOSITION=86,VAR_SPEED=64)


class RollingMergePriorityTests(unittest.TestCase):
    def fixture(self):
        vehicles,policy=load_merging_vehicles();c=Mock();c.vehicle.getParameter.return_value='previous_id'
        controller=RollingMergePriority(c,policy['zipper_meter'])
        name=vehicles[0]['id']
        created={name:dict(scenario=vehicles[0],spec=dict(length='4.')),
                 'far':dict(scenario=dict(yielding_enabled=False)),
                 'near':dict(scenario=dict(yielding_enabled=False))}
        states={name:{81:':1061_0_1',86:39.,64:0.},
                'far':{81:'-46_1',86:0.,64:10.},
                'near':{81:':1061_2_1',86:34.,64:10.}}
        return c,controller,name,created,states

    def test_only_safe_trailing_vehicles_are_filtered_and_original_setting_is_restored(self):
        c,controller,name,created,states=self.fixture()
        controller.update(created,states,TC,{name})
        c.vehicle.setParameter.assert_called_once_with(name,controller.PARAMETER,'far previous_id')
        controller.update(created,states,TC,{name})
        self.assertEqual(c.vehicle.setParameter.call_count,1)
        states[name][86]=49.
        controller.update(created,states,TC,{name})
        c.vehicle.setParameter.assert_called_with(name,controller.PARAMETER,'previous_id')
        self.assertFalse(controller.applied)
        c.vehicle.setSpeed.assert_not_called();c.vehicle.moveTo.assert_not_called()

    def test_external_lincoln_uses_current_pose_and_conservative_acceleration_bound(self):
        c,controller,name,created,states=self.fixture()
        controller.external=[dict(role='lincoln',sumo_id='lincoln0',x=16.2,y=1000.,length=5.,speed=14.,heading=-90.)]
        controller.update(created,states,TC,{name})
        self.assertIn('lincoln0',controller.applied[name].split())
        controller.external[0]['y']=130.
        controller.update(created,states,TC,{name})
        self.assertNotIn('lincoln0',controller.applied[name].split())
        self.assertNotIn('near',controller.applied[name].split())

    def test_next_main_car_respects_the_preceding_ramp_early_without_a_late_priority_switch(self):
        vehicles,policy=load_merging_vehicles();c=Mock();c.vehicle.getParameter.return_value=''
        controller=RollingMergePriority(c,policy['zipper_meter'])
        first,second,third=vehicles[:3]
        created={v['id']:dict(scenario=v,spec=dict(length='3.7')) for v in (first,second,third)}
        states={first['id']:{81:'31_1',86:140.,64:0.},second['id']:{81:'31_1',86:129.2,64:0.},
                third['id']:{81:'31_1',86:118.4,64:0.},
                'main1':{81:'-46_1',86:100.,64:10.},'main2':{81:'-46_1',86:60.,64:10.}}
        controller.reservations={first['id']:dict(main='main1'),second['id']:dict(main='main2')}
        controller.update(created,states,TC,set(created))
        self.assertEqual(set(controller.applied['main1'].split()),set(created))
        self.assertEqual(set(controller.applied['main2'].split()),{second['id'],third['id']})
        states['main1']={81:':1061_2_1',86:46.,64:10.}
        controller.main_crossings['main1']=1.
        controller.update(created,states,TC,set(created))
        self.assertEqual(set(controller.applied['main2'].split()),{second['id'],third['id']})


if __name__=='__main__':unittest.main()
