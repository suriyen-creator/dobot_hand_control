from pathlib import Path
import unittest
import time
from dobot_control.control import Pose
from dobot_control.sdk import SdkRobot

SDK = str(Path(__file__).parent / 'fixtures' / 'sdk')


class SdkTests(unittest.TestCase):
    def test_connect_move_and_pump_readback_through_subprocess(self):
        robot = SdkRobot(SDK, 'TEST')
        try:
            self.assertEqual(robot.read_pose(), Pose(200,0,50,0))
            robot.move(Pose(201,0,50,0), 5)
            self.assertTrue(robot.is_idle())
            self.assertEqual(robot.read_pose().x, 201)
            robot.set_suction(True)
            robot.stop()
            self.assertTrue(robot.suction_state())
            robot.set_suction(False)
            self.assertFalse(robot.suction_state())
        finally:
            robot.close()
        self.assertFalse(robot.process.is_alive())

    def test_expired_frame_cannot_send_motion_or_pump(self):
        robot = SdkRobot(SDK, 'TEST')
        try:
            deadline = time.monotonic() - 1
            with self.assertRaises(ValueError):
                robot.move(Pose(201,0,50,0),5,deadline=deadline)
            with self.assertRaises(ValueError):
                robot.set_suction(True,deadline=deadline)
            self.assertEqual(robot.read_pose(),Pose(200,0,50,0))
            self.assertFalse(robot.suction_state())
        finally:
            robot.close()

    def test_failed_connect_is_reported(self):
        with self.assertRaisesRegex(RuntimeError, 'ConnectDobot'):
            SdkRobot(SDK, 'MISSING')

    def test_hung_vendor_call_is_bounded_and_process_is_killed(self):
        robot = SdkRobot(SDK, 'HANG')
        robot.timeout = .1
        try:
            with self.assertRaisesRegex(RuntimeError, 'STOP NOT CONFIRMED'):
                robot.move(Pose(201,0,50,0), 5)
            self.assertFalse(robot.process.is_alive())
            with self.assertRaises(RuntimeError):
                robot.stop()
        finally:
            robot.close()


if __name__ == '__main__':
    unittest.main()
