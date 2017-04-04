#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, JSK Lab, University of Tokyo
# Copyright (c) 2014, TORK
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Lab, University of Tokyo, TORK, nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy
import rospy
from tf.transformations import quaternion_matrix, euler_from_matrix
import time

from hironx_ros_bridge.testutil.test_rosbridge import TestHiroROSBridge

PKG = 'hironx_ros_bridge'


class TestHiroROSBridgeController(TestHiroROSBridge):

    def test_tf_and_controller(self):
        goal = self.goal_RArm()
        for av in [[-0.6, 0, -100, 15.2, 9.4, 3.2]]: #[  25,-139,-157,  45,   0,   0]]:
            goal = self.setup_Positions(goal, [av])
            self.rarm.send_goal_and_wait(goal)
            # check if tf and current link is same
            rospy.sleep(1)
            now = rospy.Time.now()
            self.listener.waitForTransform("WAIST", "RARM_JOINT5_Link", now, rospy.Duration(1.0))
            (pos_tf, rot_tf) = self.listener.lookupTransform("WAIST", "RARM_JOINT5_Link", now)
            rot_tf = quaternion_matrix(rot_tf)[0:3,0:3]
            pos_c = self.robot.getCurrentPosition('RARM_JOINT5','WAIST')
            rot_c = self.robot.getCurrentRotation('RARM_JOINT5','WAIST')
            numpy.testing.assert_array_almost_equal(pos_tf, pos_c, decimal=3)
            numpy.testing.assert_array_almost_equal(rot_tf, rot_c, decimal=2)

        goal = self.goal_LArm()
        for av in [[0.6, 0, -100, -15.2, 9.4, -3.2]]: #[  25,-139,-157,  45,   0,   0]]:
            goal = self.setup_Positions(goal, [av])
            self.larm.send_goal_and_wait(goal)
            # check if tf and current link is same
            rospy.sleep(1)
            now = rospy.Time.now()
            self.listener.waitForTransform("WAIST", "LARM_JOINT5_Link", now, rospy.Duration(1.0))
            (pos_tf, rot_tf) = self.listener.lookupTransform("WAIST", "LARM_JOINT5_Link", now)
            rot_tf = quaternion_matrix(rot_tf)[0:3,0:3]
            pos_c = self.robot.getCurrentPosition('LARM_JOINT5','WAIST')
            rot_c = self.robot.getCurrentRotation('LARM_JOINT5','WAIST')
            numpy.testing.assert_array_almost_equal(pos_tf, pos_c, decimal=3)
            numpy.testing.assert_array_almost_equal(rot_tf, rot_c, decimal=2)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge_controller', TestHiroROSBridgeController)



