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
    def assertTrue_333(self, ret):
        if self.robot.fk.ref.get_component_profile().version <= '315.3.1':
            return
        self.assertTrue(ret)
    def test_set_target_pose_relative_333(self): # https://github.com/start-jsk/rtmros_hironx/issues/333
        ret = self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.01, tm=0.5)
        self.assertTrue_333(ret)
        ret = self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0, tm=0.5)
        self.assertTrue_333(ret)

# for debug
# $ python -m unittest test_hironx_ik.TestHiroIK.test_set_target_pose
#
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ik_no_init', TestHiroIK) 
