#!/usr/bin/env python3

import unittest
import math

from basestate import BaseState

class TestBaseState(unittest.TestCase):

    def test_clampDegrees(self):
        state = BaseState(None, None, None)
        self.assertEqual(state.clampDegrees(10), 10)
        self.assertEqual(state.clampDegrees(-10), 350)
        self.assertEqual(state.clampDegrees(370), 10)

    def test_toDegrees(self):
        state = BaseState(None, None, None)
        self.assertEqual(state.toDegrees(math.pi), 180)

    def test_distanceToDegrees(self):
        state = BaseState(None, None, None)
        self.assertEqual(state.distanceToDegrees(0, 0), 0)
        self.assertEqual(state.distanceToDegrees(1, 1), 45)
        self.assertEqual(state.distanceToDegrees(0, 1), 90)
        self.assertEqual(state.distanceToDegrees(-1, 1), 135)
        self.assertEqual(state.distanceToDegrees(-1, 0), 180)
        self.assertEqual(state.distanceToDegrees(-1, -1), 225)
        self.assertEqual(state.distanceToDegrees(0, -1), 270)
        self.assertEqual(state.distanceToDegrees(1, -1), 315)

    def test_shortestAngle(self):
        state = BaseState(None, None, None)
        self.assertEqual(state.shortestAngle(10, 45), 35)
        self.assertEqual(state.shortestAngle(10, 350), -20)
        self.assertEqual(state.shortestAngle(0, 179), 179)
        self.assertEqual(state.shortestAngle(0, 180), -180)
        self.assertEqual(state.shortestAngle(0, 270), -90)


if __name__ == '__main__':
    unittest.main()