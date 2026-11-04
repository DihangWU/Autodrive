"""使用真实 CARLA 值类型和模拟车辆验证暂停时命令与控制权交接。"""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import carla
from vanet.lincoln.emergency_state_management.emergency_service import LincolnEmergencyService
from vanet.lincoln.emergency_driving_control.trajectory_controller import EmergencyTrajectoryController
from modules.lincoln.driving.navigation import LincolnNavigation


class ServiceCommandTests(unittest.TestCase):
    def test_completed_control_frame_immediately_notifies_exchange_once(self):
        node,service=self.make_service()
        service.network=Mock()
        snapshot=SimpleNamespace(frame=100,find=lambda actor_id:None)
        try:
            service.after_frame(snapshot)
            self.assertEqual(service.ack_frame,100)
            service.network.notify_frame.assert_called_once_with()
            service.after_frame(snapshot)
            service.network.notify_frame.assert_called_once_with()
            snapshot.frame=101
            service.after_frame(snapshot)
            self.assertEqual(service.network.notify_frame.call_count,2)
        finally:service.close();node.navigation.close()

    def make_service(self):
        node=SimpleNamespace(control_lock=threading.RLock(),car=Mock(id=7),world=Mock(),driver=Mock(),
                             active=None,autopilot=False,cruise_speed=None,reverse=False,last_manual_control=0.,navigation_view=Mock())
        node.car.get_location.return_value=carla.Location()
        node.driver.supports_junction_local_control=False
        node.navigation=LincolnNavigation(node)
        service=LincolnEmergencyService(node)
        return node,service

    def test_prepare_finishes_without_world_tick_and_start_uses_existing_plan(self):
        node,service=self.make_service()
        points=[SimpleNamespace(transform=carla.Transform(carla.Location(x=x)),is_junction=False,
                                road_id=1,lane_id=-1,section_id=0,s=x) for x in (0.,10.,20.)]
        goal=points[-1]
        try:
            with patch('vanet.lincoln.emergency_state_management.emergency_service.plan_route',return_value=(points,goal)) as planner, \
                 patch('vanet.lincoln.emergency_state_management.emergency_service.EmergencyTrajectoryController') as factory:
                service.receive_command(dict(id='prepare',action='prepare',group='cooperative',task_id='trip',destination=dict(x=20,y=0,z=0)))
                service.process_commands()
                deadline=time.monotonic()+1
                while service.task.state=='preparing' and time.monotonic()<deadline:
                    service.process_commands();time.sleep(.005)
                self.assertEqual(service.task.state,'prepared')
                self.assertTrue(service.task.emergency)
                node.world.tick.assert_not_called()
                node.world.wait_for_tick.assert_not_called()
                node.navigation_view.prepare_navigation.assert_called_once()
                self.assertFalse(node.autopilot)
                factory.return_value.run_step.return_value=carla.VehicleControl(throttle=.5)
                service.receive_command(dict(id='start',action='start'))
                service.process_commands()
                self.assertEqual(service.task.state,'running')
                self.assertTrue(node.autopilot)
                self.assertEqual(planner.call_count,1)
                node.driver.enable.assert_not_called()
                self.assertAlmostEqual(node.car.apply_control.call_args.args[0].throttle,.5)
        finally:service.close();node.navigation.close()

    def test_duplicate_cancel_cannot_clear_a_later_emergency(self):
        node,service=self.make_service()
        try:
            service.task.emergency=True
            command=dict(id='cancel-1',action='cancel')
            service.receive_command(command);service.process_commands()
            self.assertFalse(service.task.emergency)
            service.task.emergency=True
            service.receive_command(command);service.process_commands()
            self.assertTrue(service.task.emergency)
        finally:service.close();node.navigation.close()

    def test_reset_rejected_while_emergency_identity_is_active(self):
        node,service=self.make_service()
        try:
            service.task.emergency=True
            service.receive_command(dict(id='reset',action='reset',pose=dict(x=0,y=0,z=.5,yaw=0)))
            with self.assertLogs('vanet.lincoln',level='ERROR'):
                service.process_commands()
            self.assertTrue(service.task.emergency)
            self.assertEqual(service.task.state,'error')
            node.car.set_transform.assert_not_called()
        finally:service.close();node.navigation.close()

    def test_received_clearance_limits_controller_and_predicted_collision_still_forces_stop(self):
        node,service=self.make_service()
        try:
            service.task.state='running';service.task.group='cooperative';service.task.task_id='trip'
            service.task.route_version=1;service.task.emergency=True;service.simulation_time=10.
            controller=Mock(progress=0.,distances=[0.,200.])
            controller.samples=[SimpleNamespace(center=(x,0.,0.)) for x in (0.,200.)]
            controller.closest_sample.return_value=(0,0.)
            controller.run_step.return_value=carla.VehicleControl()
            service.controller=controller
            service.goal=SimpleNamespace(transform=carla.Transform(carla.Location(x=200.)))
            service.borrowing_planner=Mock(predicted_conflict=False)
            service.borrowing_planner.evaluate.return_value=False
            ego=Mock()
            ego.get_transform.return_value=carla.Transform()
            ego.get_velocity.return_value=carla.Vector3D(x=5.)
            snapshot=Mock(timestamp=SimpleNamespace(elapsed_seconds=10.))
            snapshot.find.return_value=ego
            service.receive_deliveries([dict(payload=dict(type='clearance',source='rsu0',task_id='trip',route_version=1,
                sequence=1,sent_at=10.,vehicles={'npc-a':'unknown'},conflict_points={'npc-a':dict(x=40,y=0,z=0)}))])
            service.control(snapshot)
            self.assertGreater(controller.run_step.call_args.args[1],0.)
            self.assertLess(controller.run_step.call_args.args[1],50.)
            self.assertTrue(service.snapshot(False)['clearance_advice']['active'])
            service.borrowing_planner.predicted_conflict=True
            service.control(snapshot)
            self.assertEqual(controller.run_step.call_args.args[1],0.)
        finally:service.close();node.navigation.close()

    def test_clearance_speed_advice_never_bypasses_physical_obstacle_check(self):
        controller=EmergencyTrajectoryController.__new__(EmergencyTrajectoryController)
        controller.vehicle=SimpleNamespace(id=7)
        controller.maximum_speed=120.;controller.last_time=None
        controller.progress=0.;controller.distances=[0.,200.]
        controller.lateral_control=Mock(return_value=0.)
        controller.curve_speed_limit=Mock(side_effect=lambda target,speed:target)
        controller.obstacle_speed_limit=Mock(return_value=0.)
        controller.longitudinal_control=Mock(return_value=(0.,1.))
        ego=Mock();ego.get_velocity.return_value=carla.Vector3D(x=10.)
        ego.get_transform.return_value=carla.Transform()
        snapshot=Mock(timestamp=SimpleNamespace(elapsed_seconds=10.))
        snapshot.find.return_value=ego
        control=controller.run_step(snapshot,40.)
        controller.obstacle_speed_limit.assert_called_once_with(snapshot,40./3.6,10.,10.)
        self.assertEqual(controller.target_speed,0.)
        self.assertEqual(control.brake,1.)


if __name__=='__main__':unittest.main()
