#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx import *

class TestHironxRtcOnOff(TestHiro):

    def test_rtc_on_off(self):
        self.assertTrue(['co', "CollisionDetector"] in self.robot.getRTCList())
        self.assertFalse(['sc', "ServoController"] in self.robot.getRTCList())
        self.assertTrue(['log', "DataLogger"] in self.robot.getRTCList())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hironx_rtc_on_off', TestHironxRtcOnOff)
