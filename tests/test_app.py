import json
import subprocess
import sys
import unittest
from dobot_control.app import parse_args, key_name


class AppTests(unittest.TestCase):
    def test_default_does_not_connect_real_robot(self):
        args = parse_args([])
        self.assertFalse(args.real)
        self.assertEqual(args.config,'config.json')

    def test_headless_demo_moves_and_completes_pick_place(self):
        result = subprocess.run([sys.executable,'-m','dobot_control','--headless-demo'],
                                text=True,capture_output=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report['pump_events'],[True,False])
        self.assertEqual(report['state'],'paused')
        self.assertEqual(report['final_z'],50)
        self.assertGreater(report['moves'],10)

    def test_arrow_keys_on_windows_and_linux(self):
        for code,name in [(2424832,'left'),(2490368,'up'),(2555904,'right'),
                          (2621440,'down'),(2162688,'pgup'),(2228224,'pgdown'),
                          (65361,'left'),(65365,'pgup'),(32,'space'),(ord('S'),'s')]:
            self.assertEqual(key_name(code),name)
        self.assertIsNone(key_name(-1))


if __name__ == '__main__':
    unittest.main()
