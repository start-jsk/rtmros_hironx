#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, JSK lab at University of Tokyo All rights reserved.
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
#  * Neither the name of University of Tokyo. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

# Author: Kenji Miyake, Isaac Isao Saito

# This test script needs improved so that it becomes call-able from ROS test
# structure.

import time
import unittest

from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander, conversions
import rospy

_PKG = 'hironx_ros_bridge'
_SEC_WAIT_BETWEEN_TESTCASES = 3

class TestHironxMoveit(unittest.TestCase):

    @classmethod
    def setUpClass(self):

        rospy.init_node("test_hironx_moveit")

        self.rarm = MoveGroupCommander("right_arm")
        self.larm = MoveGroupCommander("left_arm")

        self.rarm_current_pose = self.rarm.get_current_pose().pose
        self.larm_current_pose = self.larm.get_current_pose().pose

    def _set_target_random(self):
        '''
        @type self: moveit_commander.MoveGroupCommander
        @param self: In this particular test script, the argument "self" is either
                     'rarm' or 'larm'.
        '''
        global current, current2, target
        current = self.get_current_pose()
        print "*current*", current
        target = self.get_random_pose()
        print "*target*", target
        self.set_pose_target(target)
        self.go()
        current2 = self.get_current_pose()
        print "*current2*", current2

    # Associate a method to MoveGroupCommander class. This enables users to use
    # the method on interpreter.
    # Although not sure if this is the smartest Python code, this works fine from
    # Python interpreter.
    ##MoveGroupCommander._set_target_random = _set_target_random
    
    # ****** Usage ******
    #
    # See wiki tutorial
    #  https://code.google.com/p/rtm-ros-robotics/wiki/hironx_ros_bridge_en#Manipulate_Hiro_with_Moveit_via_Python
    #

    def test_set_pose_target_rpy(self):    
        # #rpy ver
        target=[0.2035, -0.5399, 0.0709, 0,-1.6,0]
        self.rarm.set_pose_target(target)
        self.rarm.go()
        time.sleep(_SEC_WAIT_BETWEEN_TESTCASES)
    
    def test_set_pose_target_quarternion(self):    
        # #Quaternion ver
        target2=[0.2035, -0.5399, 0.0709, 0.000427, 0.000317, -0.000384, 0.999999]
        self.rarm.set_pose_target(target2)
        self.rarm.go()
        time.sleep(_SEC_WAIT_BETWEEN_TESTCASES)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_hironx_moveit_run', TestHironxMoveit)
