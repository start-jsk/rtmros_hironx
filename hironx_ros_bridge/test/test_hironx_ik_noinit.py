#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'hironx_ros_bridge'
# rosbuild needs load_manifest
try:
    import roslib
    import hironx_ros_bridge
except:
    import roslib; roslib.load_manifest(PKG)
    import hironx_ros_bridge

from hironx_ros_bridge import hironx_client as hironx

import unittest
class TestHiroIK(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot = hironx.HIRONX()
        self.robot.init()

    # DO NOT CALL goInital anyware, this is what I want to test
    def test_set_target_pose_relative_333(self): # https://github.com/start-jsk/rtmros_hironx/issues/333
        ret = self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.01, tm=0.5)
        self.assertTrue(ret)
        ret = self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0, tm=0.5)
        self.assertTrue(ret)

# for debug
# $ python -m unittest test_hironx_ik.TestHiroIK.test_set_target_pose
#
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ik_no_init', TestHiroIK) 
