import math
import unittest
from dobot_control.vision import classify, palm_center


def hand(extended, thumb=False):
    points = [(0., 0., 0.)] * 21
    points[0] = (0., 2., 0.)
    for number, base in enumerate((5,9,13,17)):
        x = float(number) - 1.5
        points[base] = (x, 0., 0.)
        points[base+1] = (x, -1., 0.)
        if number in extended:
            points[base+2] = (x, -2., 0.)
            points[base+3] = (x, -3., 0.)
        else:
            points[base+2] = (x, -.5, .3)
            points[base+3] = (x, .3, .3)
    points[1] = (-1.,1.5,0.)
    points[2] = (-2.,1.,0.)
    points[3] = (-3.,.5,0.) if thumb else (-2.,0.,0.)
    points[4] = (-4.,0.,0.) if thumb else (-1.,.5,0.)
    return points


class VisionTests(unittest.TestCase):
    def test_three_commands_and_unknown(self):
        for points, result in [(hand([],False),'fist'), (hand([0],False),'point'),
                               (hand(range(4),True),'open'), (hand([0,1],False),'unknown'),
                               (hand([],True),'unknown')]:
            self.assertEqual(classify(points), result)

    def test_rotation_and_mirroring_do_not_change_gesture(self):
        for fingers, thumb in [([],False),([0],False),(range(4),True)]:
            points = hand(fingers,thumb)
            expected = classify(points)
            theta = .8
            rotated = [(x*math.cos(theta)-y*math.sin(theta),
                        x*math.sin(theta)+y*math.cos(theta),z) for x,y,z in points]
            self.assertEqual(classify(rotated), expected)
            self.assertEqual(classify([(-x,y,z) for x,y,z in points]), expected)

    def test_bad_landmarks_are_unknown(self):
        self.assertEqual(classify([(0,0,0)]*21), 'unknown')
        self.assertEqual(classify([]), 'unknown')
        points = hand([0]); points[8] = (math.nan,0,0)
        self.assertEqual(classify(points), 'unknown')

    def test_palm_center_uses_palm_not_finger_tip(self):
        points = hand([0])
        before = palm_center(points)
        points[8] = (100,100,0)
        self.assertEqual(palm_center(points), before)


if __name__ == '__main__':
    unittest.main()
