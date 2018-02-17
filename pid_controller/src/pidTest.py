#!/usr/bin/python3
import unittest
from pid import PIDControl

class TestPid(unittest.TestCase):

    def test_getErrorAngle1(self):
        testPid = PIDControl(0, 0, 0, True)
        errAngle = testPid.getErrorAngle(120, 180)
        self.assertEqual(errAngle, -60)

    def test_getErrorAngle2(self):
        testPid = PIDControl(0, 0, 0, True)
        errAngle = testPid.getErrorAngle(720, 240)
        self.assertEqual(errAngle, 120)

    def test_getErrorAngle3(self):
        testPid = PIDControl(0, 0, 0, True)
        errAngle = testPid.getErrorAngle(-30, 45)
        self.assertEqual(errAngle, -75)


if __name__ == '__main__':
    unittest.main()
