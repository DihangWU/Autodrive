"""时钟限速覆盖跨循环等待，阻止已耗尽预算后再次限速。"""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from vanet.cobridge.simulation_time_coordination.frame_budget import FrameBudget


class FrameBudgetTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.budget = FrameBudget(.05, clock=lambda:self.now)

    def test_barrier_wait_is_part_of_current_frame_budget(self):
        self.budget.begin()
        self.now += .04
        self.budget.begin()  # Next poll loop must not restart the frame clock.
        self.now += .02
        self.assertEqual(self.budget.remaining(), 0.)

    def test_only_unused_budget_is_paced(self):
        self.budget.begin()
        self.now += .02
        self.assertAlmostEqual(self.budget.remaining(), .03)
        self.now += .03
        self.budget.finish()
        self.budget.begin()
        self.assertAlmostEqual(self.budget.remaining(), .05)

    def test_user_pause_does_not_pollute_next_frame_timing(self):
        self.budget.begin()
        self.budget.add('coordinator', .01)
        self.now += 10.
        self.budget.suspend()
        self.budget.begin()
        self.assertAlmostEqual(self.budget.remaining(), .05)
        self.assertEqual(self.budget.stages, {})

    def test_timing_report_retains_per_frame_stage_totals(self):
        for _ in range(20):
            self.budget.begin()
            self.budget.add('traffic_step', .01)
            self.budget.add('barrier_poll_wait', .04)
            self.now += .05
            report = self.budget.finish()
        self.assertEqual(report['frames'], 20)
        self.assertEqual(report['mean_ms']['frame_wall'], 50.)
        self.assertEqual(report['mean_ms']['traffic_step'], 10.)
        self.assertEqual(report['mean_ms']['barrier_poll_wait'], 40.)
        self.assertEqual(self.budget.samples, [])

    def test_cobridge_loop_counts_previous_poll_iterations_in_pacing(self):
        from foundation.cobridge import cobridge
        from vanet.cobridge.simulation_time_coordination import frame_budget
        from vanet.protocol.control_and_time_messages import control_server
        clock=SimpleNamespace(now=0.,polls=0)
        stopped=Mock(is_set=Mock(return_value=False))
        stopped.wait.side_effect=lambda duration:setattr(clock,'now',clock.now+duration)
        runtime=Mock(frames=0)
        runtime.vanet=Mock(phase='running',network_time=0.)

        def before_step():
            clock.polls+=1
            if clock.polls%3:return False
            clock.now+=.025
            return True

        def traffic_step():
            runtime.frames+=1
            clock.now+=.010

        runtime.vanet.before_step.side_effect=before_step
        runtime.vanet.after_step.side_effect=lambda:setattr(clock,'now',clock.now+.001)
        runtime.step.side_effect=traffic_step
        args=SimpleNamespace(vanet=True,steps=3,step_length=.05,fast=False,control_socket='unused-test-socket')
        with patch.object(cobridge,'BridgeRuntime',return_value=runtime), \
             patch.object(control_server,'VanetControlServer'), \
             patch.object(cobridge.time,'monotonic',side_effect=lambda:clock.now), \
             patch.object(frame_budget,'FrameBudget',side_effect=lambda step:FrameBudget(step,clock=lambda:clock.now)):
            cobridge.run_clock(args,stopped)
        self.assertEqual(runtime.frames,3)
        self.assertAlmostEqual(clock.now,.15)
        self.assertEqual(runtime.vanet.after_step.call_count,3)
