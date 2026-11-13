import tempfile
import unittest
from pathlib import Path
from dobot_control.control import Config, Pose
from dobot_control.teaching import Teaching


class TeachingTests(unittest.TestCase):
    def test_requires_all_captures_and_explicit_workspace_confirmation(self):
        teaching = Teaching(Config())
        with self.assertRaises(ValueError):
            teaching.finish(False)
        for key,pose in [('1',Pose(180,-20,50)), ('2',Pose(220,20,50)),
                         ('t',Pose(200,0,50)),('p',Pose(200,0,20)),('l',Pose(200,0,20))]:
            teaching.capture(key,pose)
        with self.assertRaises(ValueError):
            teaching.finish(False)
        cfg = teaching.finish(True)
        self.assertTrue(cfg.calibrated)
        self.assertEqual((cfg.x_min,cfg.x_max), (180,220))
        self.assertEqual((cfg.pick_z,cfg.place_z,cfg.travel_z),(20,20,50))

    def test_axis_swap_and_reverse(self):
        teaching = Teaching(Config())
        teaching.swap_axes()
        teaching.reverse('right')
        self.assertEqual(teaching.config.right_axis, 'x')
        self.assertEqual(teaching.config.forward_axis, 'y')
        self.assertEqual(teaching.config.right_sign, -1)

    def test_config_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'config.json'
            Config.demo().save(path)
            self.assertEqual(Config.load(path), Config.demo())


if __name__ == '__main__':
    unittest.main()
