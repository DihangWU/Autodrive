"""不启动仿真，检查紧急身份、逆向轨迹和预测冲突的关键约束。"""
import math
import unittest
from dataclasses import dataclass
from vanet.lincoln.emergency_state_management.task_state import EmergencyTaskState
from vanet.lincoln.emergency_route_planning.borrowing_geometry import blend_borrowing_segment, trajectory_clear


@dataclass(frozen=True)
class Sample:
    center: tuple
    forward: tuple=(1.,0.,0.)
    right: tuple=(0.,1.,0.)
    width: float=3.5


class EmergencyIdentityTests(unittest.TestCase):
    def prepare(self):
        task=EmergencyTaskState()
        task.prepare(dict(id='prepare-1',group='cooperative',task_id='trip-1',destination=dict(x=100,y=0,z=0)))
        task.prepared();task.start()
        return task

    def test_manual_and_arrival_preserve_emergency_until_explicit_cancel(self):
        task=self.prepare();task.takeover()
        self.assertTrue(task.emergency)
        self.assertEqual(task.destination,dict(x=100,y=0,z=0))
        task.arrive()
        self.assertTrue(task.emergency)
        with self.assertRaisesRegex(ValueError,'Cancel emergency'):
            task.reset()
        task.cancel_from_sumo();task.reset()
        self.assertFalse(task.emergency)
        self.assertEqual(task.state,'reset')

    def test_normal_group_cannot_silently_clear_existing_emergency(self):
        task=self.prepare()
        with self.assertRaisesRegex(ValueError,'Cancel emergency'):
            task.prepare(dict(id='prepare-2',group='normal',destination=dict(x=0,y=0,z=0)))
        self.assertTrue(task.emergency)

    def test_start_requires_successfully_prepared_route(self):
        task=EmergencyTaskState()
        with self.assertRaisesRegex(ValueError,'not ready'):
            task.start()


class BorrowingGeometryTests(unittest.TestCase):
    def test_borrow_returns_to_own_lane_and_keeps_forward_travel(self):
        samples=[Sample((float(x),0.,0.)) for x in range(61)]
        targets=[(float(x),-3.5,0.) for x in range(5,56)]
        changed=blend_borrowing_segment(samples,targets,5,55,12.)
        self.assertEqual(changed[5].center,samples[5].center)
        self.assertEqual(changed[55].center,samples[55].center)
        self.assertAlmostEqual(changed[30].center[1],-3.5)
        self.assertTrue(all(p.forward[0]>.8 for p in changed))
        self.assertEqual(samples[30].center[1],0.)

    def test_predicted_oncoming_vehicle_rejects_currently_clear_path(self):
        samples=[Sample((float(x),-3.5,0.)) for x in range(41)]
        obstacle=dict(center=(60.,-3.5,0.),velocity=(-10.,0.,0.),yaw=math.pi,length=2.5,width=1.,height=1.)
        self.assertFalse(trajectory_clear(samples,[obstacle],10.,1.,2.5))
        obstacle['center']=(60.,-3.5,10.)
        self.assertTrue(trajectory_clear(samples,[obstacle],10.,1.,2.5))

    def test_side_lane_vehicle_does_not_block_valid_borrowing_corridor(self):
        samples=[Sample((float(x),-3.5,0.)) for x in range(41)]
        obstacle=dict(center=(20.,0.,0.),velocity=(0.,0.,0.),yaw=0.,length=2.5,width=1.,height=1.)
        self.assertTrue(trajectory_clear(samples,[obstacle],10.,1.,2.5))
        obstacle['center']=(20.,-3.5,0.)
        self.assertFalse(trajectory_clear(samples,[obstacle],10.,1.,2.5))


if __name__=='__main__':unittest.main()
