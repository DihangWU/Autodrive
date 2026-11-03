"""离线验证同一 CARLA 帧只运行一次自动控制，且控制完成后才确认帧。"""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from modules.lincoln.runtime.application import run_vehicle_control_loop


class ControlLoopTests(unittest.TestCase):
    def make_node(self,frames):
        stopped=threading.Event()
        events=[]
        pending=iter(frames)
        current=SimpleNamespace(frame=-1)

        def wait_for_tick(timeout):
            frame,timed_out=next(pending)
            current.frame=frame
            if timed_out:raise RuntimeError('No new simulation frame')
            return SimpleNamespace(frame=frame)

        world=SimpleNamespace(id=3,wait_for_tick=wait_for_tick,
                              get_snapshot=lambda:SimpleNamespace(frame=current.frame))
        service=SimpleNamespace(
            process_commands=Mock(side_effect=lambda:events.append(('commands',current.frame))),
            after_frame=Mock(side_effect=lambda snapshot:events.append(('ack',snapshot.frame))))
        node=SimpleNamespace(world=world,emergency_service=service,
            car=SimpleNamespace(is_alive=True),client=SimpleNamespace(get_world=lambda:world),
            publish_vehicle_transform=Mock(),update_camera=Mock(),stop_requested=False)
        node.update_automatic_driving=Mock(side_effect=lambda snapshot:events.append(('automatic',snapshot.frame)))

        def drive():
            events.append(('manual',current.frame))
            if sum(kind=='manual' for kind,_ in events)==len(frames):stopped.set()

        node.drive=Mock(side_effect=drive)
        return node,stopped,events

    def test_same_frame_timeout_does_not_repeat_pid_but_keeps_manual_and_commands_responsive(self):
        node,stopped,events=self.make_node([(100,False),(100,True),(101,False),(101,True)])
        run_vehicle_control_loop(node,stopped)
        self.assertFalse(node.stop_requested)
        self.assertEqual([frame for kind,frame in events if kind=='automatic'],[100,101])
        self.assertEqual(node.drive.call_count,4)
        self.assertEqual(node.emergency_service.process_commands.call_count,6)
        self.assertEqual(node.publish_vehicle_transform.call_count,2)
        self.assertEqual(node.update_camera.call_count,2)
        for index,(kind,frame) in enumerate(events):
            if kind=='ack':
                self.assertIn(('automatic',frame),events[:index])
                self.assertEqual(events[index-1],('manual',frame))

    def test_failed_automatic_control_never_acknowledges_uncontrolled_frame(self):
        node,stopped,_=self.make_node([(100,False)])
        node.update_automatic_driving.side_effect=RuntimeError('Control write failed')
        with patch('modules.lincoln.runtime.application.logging.exception'):
            run_vehicle_control_loop(node,stopped)
        self.assertTrue(node.stop_requested)
        node.emergency_service.after_frame.assert_not_called()
        node.drive.assert_not_called()

    def test_failed_manual_control_never_acknowledges_frame(self):
        node,stopped,_=self.make_node([(100,False)])
        node.drive.side_effect=RuntimeError('Manual control write failed')
        with patch('modules.lincoln.runtime.application.logging.exception'):
            run_vehicle_control_loop(node,stopped)
        self.assertTrue(node.stop_requested)
        node.emergency_service.after_frame.assert_not_called()


if __name__=='__main__':unittest.main()
