"""净空反馈只能对有效前方冲突增加限速，不能提供越过障碍的许可。"""
import math
from types import SimpleNamespace
import unittest

from vanet.lincoln.v2x_data_exchange.clearance_advice import ClearanceAdvice


class ClearanceAdviceTests(unittest.TestCase):
    def setUp(self):
        self.advice=ClearanceAdvice()
        self.samples=[SimpleNamespace(center=(float(x),0.,0.)) for x in range(0,201,10)]
        self.distances=[float(x) for x in range(0,201,10)]

    def message(self,**changes):
        result=dict(type='clearance',source='rsu0',task_id='trip',route_version=2,
                    sequence=1,sent_at=10.,vehicles={'npc-a':'yielding'},
                    conflict_points={'npc-a':dict(x=40.,y=0.,z=0.)})
        result.update(changes)
        return result

    def evaluate(self,now=11.,task='trip',version=2,position=(0.,0.,0.)):
        return self.advice.evaluate(now,task,version,position,self.samples,self.distances,position[0])

    def receive(self,message=None):
        return self.advice.receive(message or self.message(),11.,'trip',2)

    def test_no_actual_delivery_produces_no_speed_limit(self):
        self.assertIsNone(self.evaluate()['speed_limit_kmh'])

    def test_pending_feedback_limits_speed_to_stop_before_closest_conflict(self):
        self.assertTrue(self.receive(self.message(vehicles={'npc-a':'yielding','npc-b':'unknown'},
            conflict_points={'npc-a':dict(x=40,y=0,z=0),'npc-b':dict(x=25,y=0,z=0)})))
        result=self.evaluate()
        self.assertTrue(result['active'])
        self.assertAlmostEqual(result['speed_limit_kmh'],math.sqrt(2.*2.5*(25.-8.))*3.6)
        self.assertEqual(result['vehicles'],['npc-a','npc-b'])
        self.assertEqual(result['nearest_distance_metres'],25.)

    def test_conflict_inside_stopping_margin_requests_full_stop(self):
        self.receive(self.message(conflict_points={'npc-a':dict(x=5,y=0,z=0)}))
        self.assertEqual(self.evaluate()['speed_limit_kmh'],0.)

    def test_cleared_and_normal_feedback_remove_only_the_communication_limit(self):
        self.receive()
        self.assertTrue(self.evaluate()['active'])
        self.receive(self.message(sequence=2,vehicles={'npc-a':'cleared','npc-b':'normal'},all_cleared=True))
        result=self.evaluate()
        self.assertFalse(result['active'])
        self.assertIsNone(result['speed_limit_kmh'])
        self.assertFalse(self.receive())  # 较旧的等待消息不能覆盖较新的净空消息。
        self.assertIsNone(self.evaluate()['speed_limit_kmh'])

    def test_future_expired_wrong_task_version_or_source_messages_are_rejected(self):
        for change in (dict(sent_at=11.1),dict(sent_at=7.9),dict(sent_at=float('nan')),
                       dict(task_id='old-trip'),dict(route_version=1),dict(source='npc-a')):
            with self.subTest(change=change):
                self.assertFalse(self.receive(self.message(**change)))
                self.assertFalse(self.evaluate()['active'])

    def test_valid_feedback_expires_and_is_invalid_after_route_change(self):
        self.receive()
        self.assertTrue(self.evaluate(now=13.)['active'])
        self.assertFalse(self.evaluate(now=13.001)['active'])
        self.assertFalse(self.evaluate(now=9.)['active'])
        self.assertFalse(self.evaluate(version=3)['active'])
        self.assertFalse(self.evaluate(task='another-trip')['active'])

    def test_behind_different_height_off_route_and_beyond_horizon_are_ignored(self):
        for point in (dict(x=-3,y=0,z=0),dict(x=25,y=0,z=6),dict(x=25,y=5,z=0),dict(x=121,y=0,z=0)):
            with self.subTest(point=point):
                self.advice.reset()
                self.receive(self.message(conflict_points={'npc-a':point}))
                self.assertFalse(self.evaluate()['active'])

    def test_current_borrowing_route_is_used_instead_of_original_lane(self):
        self.samples=[SimpleNamespace(center=(float(x),-3.5,0.)) for x in range(0,201,10)]
        self.receive()
        self.assertFalse(self.evaluate(position=(0.,-3.5,0.))['active'])

    def test_curved_route_uses_travel_distance_instead_of_straight_line_distance(self):
        self.samples=[SimpleNamespace(center=p) for p in ((0.,0.,0.),(20.,0.,0.),(20.,20.,0.))]
        self.distances=[0.,20.,40.]
        self.receive(self.message(conflict_points={'npc-a':dict(x=20,y=10,z=0)}))
        result=self.evaluate()
        self.assertEqual(result['nearest_distance_metres'],30.)
        self.assertAlmostEqual(result['speed_limit_kmh'],math.sqrt(110.)*3.6)


if __name__=='__main__':unittest.main()
