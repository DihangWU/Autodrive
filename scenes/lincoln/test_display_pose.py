"""Display interpolation checks; no CARLA, ROS or sensor processes required."""

import math
import unittest

from display_pose import DisplayPoseBuffer


def heading(degrees):
    angle = math.radians(degrees) / 2
    return (0., 0., math.sin(angle), math.cos(angle))


class DisplayPoseTests(unittest.TestCase):
    def test_position_is_interpolated_at_delayed_simulation_time(self):
        poses = DisplayPoseBuffer(.05)
        self.assertIsNone(poses.sample(0.))
        for t in (0., .05, .1):
            poses.push(t, (10 * t, 20 * t, 3.), heading(0), wall_time=t)
        timestamp, position, rotation = poses.sample(.125)
        self.assertAlmostEqual(timestamp, .075)
        self.assertAlmostEqual(position[0], .75)
        self.assertAlmostEqual(position[1], 1.5)
        self.assertEqual(position[2], 3.)
        self.assertEqual(rotation, heading(0))
        self.assertIsNone(poses.sample(.125))

    def test_heading_crosses_180_degrees_by_the_shortest_path(self):
        poses = DisplayPoseBuffer(.05)
        poses.push(0., (0., 0., 0.), heading(179), wall_time=0.)
        poses.push(.1, (1., 0., 0.), heading(-179), wall_time=.1)
        _, _, rotation = poses.sample(.1)
        self.assertAlmostEqual(sum(x * x for x in rotation), 1.)
        angle = math.degrees(2 * math.atan2(rotation[2], rotation[3]))
        self.assertAlmostEqual(abs(angle), 180.)

    def test_pause_stops_at_the_last_real_pose(self):
        poses = DisplayPoseBuffer(.05)
        poses.push(0., (0., 0., 0.), heading(0), wall_time=0.)
        poses.push(.05, (1., 0., 0.), heading(30), wall_time=.05)
        timestamp, position, rotation = poses.sample(10.)
        self.assertEqual(timestamp, .05)
        self.assertEqual(position, (1., 0., 0.))
        for a, b in zip(rotation, heading(30)):
            self.assertAlmostEqual(a, b)
        self.assertIsNone(poses.sample(20.))

    def test_new_simulation_timeline_resets_display_state(self):
        poses = DisplayPoseBuffer(.05)
        poses.push(50., (10., 0., 0.), heading(0), wall_time=0.)
        poses.sample(1.)
        poses.push(0., (0., 0., 0.), heading(90), wall_time=2.)
        timestamp, position, _ = poses.sample(2.)
        self.assertEqual(timestamp, 0.)
        self.assertEqual(position, (0., 0., 0.))

    def test_jittery_20_hz_input_produces_more_display_poses_without_prediction(self):
        poses = DisplayPoseBuffer(.05)
        events = [(i * .05 + (.004 if i % 2 else 0.), i * .05)
                  for i in range(41)]
        next_event = 0
        output = []
        for frame in range(133):
            now = frame / 60
            while next_event < len(events) and events[next_event][0] <= now:
                receipt, simulation = events[next_event]
                poses.push(simulation, (24 * simulation, 0., 0.), heading(0), receipt)
                next_event += 1
            display = poses.sample(now)
            if display is not None:
                timestamp, position, _ = display
                self.assertLessEqual(timestamp, events[next_event - 1][1])
                self.assertAlmostEqual(position[0], 24 * timestamp)
                output.append(timestamp)
        self.assertGreater(len(output), 100)
        self.assertTrue(all(b > a for a, b in zip(output, output[1:])))
        self.assertAlmostEqual(output[-1], 2.)


if __name__ == '__main__':
    unittest.main()
