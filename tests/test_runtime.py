import tempfile
import time
import unittest
from pathlib import Path
from dobot_control.control import Config, Controller, Observation, Pose
from dobot_control.robot import SimRobot
from dobot_control.runtime import Runtime


class RuntimeTests(unittest.TestCase):
    def make_runtime(self, root):
        return Runtime(SimRobot(), Config.demo(), Path(root)/'config.json')

    def test_camera_stall_is_detected_without_main_thread_ticks(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.make_runtime(root)
            r.start_thread()
            try:
                r.observation = Observation(time.monotonic(),1,'point',.5,.5)
                r.command('s')
                deadline = time.monotonic()+1
                while r.status.get('state') != 'ready' and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertEqual(r.status['state'],'ready')
                time.sleep(.4)
                self.assertEqual(r.status['state'],'paused')
            finally:
                r.close()

    def test_teaching_disables_start_until_saved(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.make_runtime(root)
            r.handle('c',0)
            self.assertFalse(r.controller.config.calibrated)
            with self.assertRaises(ValueError):
                r.handle('s',0)

    def test_capture_rejected_while_motion_pending(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.make_runtime(root)
            r.handle('c',0)
            r.handle('left',0)
            with self.assertRaises(ValueError):
                r.handle('t',0)

    def test_sim_calibration_cannot_arm_real_robot(self):
        robot = SimRobot()
        robot.real = True
        c = Controller(robot, Config.demo())
        c.holding = False
        c.observation = Observation(0,1,'point',.5,.5)
        with self.assertRaises(ValueError):
            c.start(0)

    def test_nontravel_height_while_active_requires_recovery(self):
        c = Controller(SimRobot(), Config.demo())
        o = Observation(0,1,'point',.5,.5)
        c.tick(o,0)
        c.start(0)
        c.robot.pose = Pose(200,0,40,0)
        c.tick(Observation(.05,1,'point',.9,.5),.05)
        self.assertEqual(c.state,'recovery')


if __name__ == '__main__':
    unittest.main()
