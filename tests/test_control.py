import math
import time
import unittest
from dataclasses import replace

from dobot_control.control import Config, Controller, Observation, Pose, zone
from dobot_control.robot import SimRobot


class ZoneTests(unittest.TestCase):
    def test_center_and_four_directions(self):
        for point, expected in [((.5,.5),'stop'), ((.35,.5),'stop'),
                                ((.1,.5),'left'), ((.9,.5),'right'),
                                ((.5,.1),'forward'), ((.5,.9),'back'),
                                ((.1,.1),'stop'), ((float('nan'),.5),'stop')]:
            with self.subTest(point=point):
                self.assertEqual(zone(*point), expected)


class ConfigTests(unittest.TestCase):
    def test_validation_rejects_unsafe_values(self):
        for changes in [dict(speed=0), dict(speed=math.nan), dict(step_mm=2),
                        dict(x_min=300, x_max=200), dict(pick_z=70),
                        dict(right_axis='z'), dict(right_axis='x',forward_axis='x')]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(Config.demo(), **changes).validate()


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.robot = SimRobot(Pose(200, 0, 50, 0))
        self.config = Config.demo()
        self.c = Controller(self.robot, self.config)
        self.t = 0.0

    def observe(self, gesture='point', x=.5, y=.5, hands=1, elapsed=.05):
        self.t += elapsed
        self.c.tick(Observation(self.t, hands, gesture, x, y), self.t)

    def start(self):
        self.observe()
        self.c.start(self.t)
        self.assertEqual(self.c.state, 'ready')

    def stable(self, gesture):
        for _ in range(12):
            self.observe(gesture)

    def finish(self, gesture):
        for _ in range(300):
            self.observe(gesture)
            if not self.c.busy:
                return
        self.fail('sequence did not complete')

    def test_startup_paused_and_requires_calibration(self):
        self.observe(x=.9)
        self.assertEqual(self.robot.pose.y, 0)
        self.c.config = replace(self.config, calibrated=False)
        with self.assertRaises(ValueError):
            self.c.start(self.t)

    def test_movement_is_one_axis_and_bounded(self):
        self.start()
        for _ in range(100):
            self.observe(x=.9)
        self.assertEqual(self.robot.pose.x, 200)
        self.assertLessEqual(self.robot.pose.y, self.config.y_max)
        self.assertEqual(self.robot.pose.z, 50)
        self.assertTrue(all(a.distance(b) <= 1.0001 for a,b in self.robot.moves))

    def test_pick_then_place_and_no_repeated_action(self):
        self.start()
        self.stable('fist')
        self.finish('fist')
        self.assertTrue(self.c.holding)
        self.assertTrue(self.robot.suction)
        self.assertEqual(self.robot.pose.z, self.config.travel_z)
        self.assertEqual(self.robot.pump_events, [True])
        self.stable('fist')
        self.assertFalse(self.c.busy)
        # A fresh pointing gesture is needed after an operation.
        self.stable('point')
        self.stable('open')
        self.finish('open')
        self.assertFalse(self.c.holding)
        self.assertEqual(self.robot.pump_events, [True, False])
        self.assertEqual(self.robot.pose.z, self.config.travel_z)

    def test_lost_hand_latches_pause_and_preserves_load(self):
        self.start()
        self.stable('fist')
        self.finish('fist')
        self.observe(hands=0)
        self.assertEqual(self.c.state, 'paused')
        self.assertTrue(self.robot.suction)
        previous = self.robot.pose
        self.observe(x=.9)
        self.assertEqual(self.robot.pose, previous)

    def test_two_hands_stop(self):
        self.start()
        self.observe(hands=2)
        self.assertEqual(self.c.state, 'paused')

    def test_stale_frame_stops_even_during_pick(self):
        self.start()
        self.stable('fist')
        self.assertTrue(self.c.busy)
        obs = Observation(self.t, 1, 'fist', .5, .5)
        self.t += .31
        self.c.tick(obs, self.t)
        self.assertEqual(self.c.state, 'recovery')
        self.assertFalse(self.c.busy)
        with self.assertRaises(ValueError):
            self.c.start(self.t)

    def test_stop_mid_sequence_cannot_resume_automatically(self):
        self.start()
        self.stable('fist')
        self.c.pause('operator stop')
        self.assertEqual(self.c.state, 'recovery')
        self.assertEqual(self.robot.stop_count, 1)
        self.assertFalse(self.c.busy)

    def test_slow_pose_read_cannot_act_on_an_expired_frame(self):
        self.start()
        read_pose = self.robot.read_pose
        def delayed_pose():
            time.sleep(.31)
            return read_pose()
        self.robot.read_pose = delayed_pose
        before = len(self.robot.moves)
        self.observe(x=.9)
        self.assertEqual(len(self.robot.moves), before)
        self.assertEqual(self.c.state, 'paused')

    def test_recovery_rejects_sim_calibration_on_real_robot(self):
        self.robot.real = True
        with self.assertRaises(ValueError):
            self.c.recover(False)
        self.assertEqual(self.robot.pump_events, [])
        self.assertEqual(self.robot.moves, [])

    def test_motion_timeout_requires_recovery(self):
        self.start()
        self.robot.is_idle = lambda: False
        self.observe(x=.9)
        self.assertIsNotNone(self.c.pending)
        obs = Observation(self.t + 11,1,'point',.9,.5)
        self.t += 11
        self.c.tick(obs,self.t)
        self.assertEqual(self.c.state,'recovery')
        self.assertIsNone(self.c.pending)

    def test_stop_after_pump_on_preserves_pump_during_recovery(self):
        self.start()
        self.stable('fist')
        for _ in range(200):
            self.observe('fist')
            if self.c.phase == 'pump':
                break
        self.assertTrue(self.robot.suction)
        self.c.pause('Operator stop')
        self.assertEqual(self.c.state,'recovery')
        self.assertTrue(self.robot.suction)
        self.c.recover(True)
        for _ in range(200):
            self.observe()
            if self.c.state == 'paused':
                break
        self.assertEqual(self.c.state,'paused')
        self.assertTrue(self.robot.suction)
        self.assertEqual(self.robot.pose.z,self.config.travel_z)
        self.c.start(self.t)
        self.assertEqual(self.c.state,'ready')

    def test_unknown_gesture_stops_without_disarming(self):
        self.start()
        previous = self.robot.pose
        self.observe('unknown', x=.9)
        self.assertEqual(self.robot.pose, previous)
        self.assertEqual(self.c.state, 'ready')

    def test_empty_open_and_holding_fist_are_ignored(self):
        self.start()
        self.stable('open')
        self.assertEqual(self.robot.pump_events, [])
        self.assertFalse(self.c.busy)


if __name__ == '__main__':
    unittest.main()
