"""拉链合流规则副本与按一辆主路、一辆匝道车预约的流量计。"""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from foundation.cobridge.network import read_network
from vanet.project_paths import SUMO_TOWN04_CONFIGURATION_FILE,SUMO_TOWN04_NETWORK_FILE
from vanet.npc.highway_ramp_merging.zipper_network import prepare_zipper_configuration
from vanet.npc.highway_ramp_merging.load_merging_vehicles import load_merging_vehicles
from vanet.npc.highway_ramp_merging.zipper_release_meter import ZipperReleaseMeter

TC=SimpleNamespace(VAR_LANE_ID=81,VAR_LANEPOSITION=86)


class ZipperMergingTests(unittest.TestCase):
    def test_map_geometry_fingerprint_and_other_junctions_are_unchanged(self):
        original=SUMO_TOWN04_NETWORK_FILE.read_bytes()
        with tempfile.TemporaryDirectory(prefix='zipper_rules_') as folder:
            config=prepare_zipper_configuration(Path(folder))
            base=ET.fromstring(original);patched=ET.parse(read_network(config)['network_file']).getroot()
            self.assertEqual(read_network(config)['xodr_sha256'],read_network(SUMO_TOWN04_CONFIGURATION_FILE)['xodr_sha256'])
            changed_lengths=set()
            for before,after in zip(base.findall('edge'),patched.findall('edge')):
                self.assertEqual(before.attrib,after.attrib)
                before_lanes=before.findall('lane');after_lanes=after.findall('lane')
                self.assertEqual(len(before_lanes),len(after_lanes))
                for old,new in zip(before_lanes,after_lanes):
                    old_attributes=dict(old.attrib);new_attributes=dict(new.attrib)
                    old_length=old_attributes.pop('length');new_length=new_attributes.pop('length')
                    self.assertEqual(old_attributes,new_attributes)
                    if old_length!=new_length:
                        self.assertEqual(before.get('function'),'internal')
                        self.assertGreaterEqual(len(before_lanes),4)
                        self.assertEqual(float(new_length),max(float(l.get('length')) for l in before_lanes))
                        changed_lengths.add(before.get('id'))
            self.assertIn(':134_10',changed_lengths)
            self.assertEqual(len({l.get('length') for l in patched.find("edge[@id=':134_10']").findall('lane')}),1)
            self.assertEqual([ET.tostring(v) for v in base.findall('tlLogic')],[ET.tostring(v) for v in patched.findall('tlLogic')])
            for before,after in zip(base.findall('junction'),patched.findall('junction')):
                if before.get('id')!='1061':self.assertEqual(ET.tostring(before),ET.tostring(after))
                else:
                    self.assertEqual(after.get('type'),'zipper')
                    for attr in ('shape','x','y','z','incLanes','intLanes'):self.assertEqual(before.get(attr),after.get(attr))
                    requests={int(q.get('index')):q for q in after.findall('request')}
                    self.assertEqual(int(requests[1].get('foes'),2),1<<3)
                    self.assertEqual(int(requests[3].get('response'),2),1<<1)
                    self.assertTrue(all(not int(requests[i].get('foes'),2)&(1<<1) for i in (2,4,5,6)))
            changes=[(a,b) for a,b in zip(base.findall('connection'),patched.findall('connection'))
                     if ET.tostring(a)!=ET.tostring(b)]
            self.assertEqual({b.get('via') for _,b in changes},{':1061_0_1',':1061_2_1'})
            self.assertTrue(all(b.get('state')=='Z' for _,b in changes))
        self.assertEqual(SUMO_TOWN04_NETWORK_FILE.read_bytes(),original)

    def fixture(self):
        vehicles,policy=load_merging_vehicles();c=Mock()
        meter=ZipperReleaseMeter(c,policy['zipper_meter'])
        created={v['id']:dict(scenario=v,spec=dict(length='3.7')) for v in vehicles[:2]}
        for name in ('main1','main2'):
            created[name]=dict(scenario=dict(role='same_direction',yielding_enabled=False,maximum_speed_kmh=80.))
        states={vehicles[0]['id']:{81:'31_1',86:140.,64:0.},
                vehicles[1]['id']:{81:'31_1',86:129.2,64:0.},
                'main1':{81:'-46_1',86:100.,64:12.},'main2':{81:'-46_1',86:60.,64:12.}}
        return c,meter,created,states

    def test_each_main_vehicle_reserves_one_ramp_slot_and_ramps_can_accelerate_before_it_passes(self):
        c,meter,created,states=self.fixture();released=set();completed=set()
        tc=SimpleNamespace(VAR_LANE_ID=81,VAR_LANEPOSITION=86,VAR_SPEED=64)
        meter.start(0.)
        meter.update_slots(created,states,tc,released,completed,0.)
        self.assertEqual([e['main_vehicle'] for e in meter.events],['main1','main2'])
        self.assertEqual([e['required_main_vehicles'] for e in meter.events],[1,1])
        self.assertEqual(len(released),2)  # pipelined approaches, not one stationary release at a time
        self.assertEqual(meter.main_passed,0)
        self.assertTrue(any(call.args[1]>0. for call in c.vehicle.setSpeed.call_args_list))
        self.assertTrue(all(call.args[0].startswith('vanet_npc_ramp_merge_')
                            for call in c.vehicle.setSpeed.call_args_list))
        c.vehicle.slowDown.assert_not_called();c.vehicle.openGap.assert_not_called()
        c.vehicle.setMaxSpeed.assert_not_called();c.vehicle.moveTo.assert_not_called()
        calls=c.vehicle.setSpeed.call_count
        meter.update_slots(created,states,tc,released,completed,.05)
        self.assertEqual(c.vehicle.setSpeed.call_count,calls)
        meter.update_slots(created,states,tc,released,completed,.2)
        self.assertEqual(len(meter.events),2)  # reservation is stable; no double booking

    def test_empty_main_stream_is_used_immediately_without_a_fixed_timeout(self):
        c,meter,created,states=self.fixture();released=set();completed=set()
        states={name:state for name,state in states.items() if name.startswith('vanet_npc_ramp_merge_')}
        tc=SimpleNamespace(VAR_LANE_ID=81,VAR_LANEPOSITION=86,VAR_SPEED=64)
        meter.start(0.)
        meter.update_slots(created,states,tc,released,completed,0.)
        self.assertEqual(len(released),2)
        self.assertTrue(all(e['reason']=='main_empty' for e in meter.events))
        self.assertTrue(all(call.args[1]>0. for call in c.vehicle.setSpeed.call_args_list))
        c.vehicle.setSpeedMode.assert_not_called()

    def test_more_ramp_cars_than_main_slots_can_still_drive_into_the_ramp(self):
        c,meter,created,states=self.fixture();released=set();completed=set()
        tc=SimpleNamespace(VAR_LANE_ID=81,VAR_LANEPOSITION=86,VAR_SPEED=64)
        states.pop('main2')
        waiting=next(name for name,record in created.items()
                     if record['scenario'].get('merge_queue_index')==1)
        meter.start(0.)
        meter.update_slots(created,states,tc,released,completed,0.)
        self.assertEqual(len(meter.reservations),1)
        self.assertIn(waiting,released)
        c.vehicle.setSpeed.assert_any_call(waiting,-1.)
        self.assertNotIn(waiting,meter.reservations)  # approach is independent of the final merge slot
        states['main1']={81:':1061_2_1',86:46.,64:12.}
        meter.update_slots(created,states,tc,released,completed,1.)
        self.assertEqual(meter.reservations[waiting]['main'],None)
        self.assertEqual(len(meter.events),2)

    def test_crossing_is_counted_once_at_the_actual_main_conflict_point(self):
        c,meter,created,states=self.fixture()
        meter.observe(created,{'main1':{81:':1061_2_1',86:35.}},TC,0.)
        self.assertEqual(meter.main_passed,0)
        meter.observe(created,{'main1':{81:':1061_2_1',86:46.}},TC,1.)
        meter.observe(created,{'main1':{81:':1061_2_1',86:46.}},TC,1.05)
        self.assertEqual(meter.main_passed,1)
        self.assertEqual(meter.main_crossings,{'main1':1.})
        meter.reset();self.assertFalse(meter.reservations);self.assertFalse(meter.commanded)


if __name__=='__main__':unittest.main()
