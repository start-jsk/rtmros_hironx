#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association 
# (TORK) All rights reserved.
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
#  * Neither the name of TORK. nor the names of its
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

# Author: Isaac I.Y. Saito

# This test script needs improved so that it becomes call-able from ROS test
# structure.

import numpy
import time
import unittest

from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander, conversions
import rospy

_PKG = 'hironx_ros_bridge'
_SEC_WAIT_BETWEEN_TESTCASES = 3

class TestHironxMoveit(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestHironxMoveit, self).__init__(*args, **kwargs)

    @classmethod
    def setUpClass(self):

        rospy.init_node("test_hironx_moveit")

        self.rarm = MoveGroupCommander("right_arm")
        self.larm = MoveGroupCommander("left_arm")
        self.botharms = MoveGroupCommander("both_arm")

        # Tentative workaround for https://github.com/tork-a/rtmros_nextage/issues/170
        safe_kinematicsolver = "RRTConnectkConfigDefault"
        self.larm.set_planner_id(safe_kinematicsolver)
        self.rarm.set_planner_id(safe_kinematicsolver)
        self.botharms.set_planner_id(safe_kinematicsolver)

        self._botharms_joints = ['LARM_JOINT0', 'LARM_JOINT1',
                                 'LARM_JOINT2', 'LARM_JOINT3',
                                 'LARM_JOINT4', 'LARM_JOINT5',
                                 'RARM_JOINT0', 'RARM_JOINT1',
                                 'RARM_JOINT2', 'RARM_JOINT3',
                                 'RARM_JOINT4', 'RARM_JOINT5']

        self.rarm_current_pose = self.rarm.get_current_pose().pose
        self.larm_current_pose = self.larm.get_current_pose().pose
        # For botharms get_current_pose ends with no eef error.

        self.init_rtm_jointvals = [0.010471975511965976, 0.0, -1.7453292519943295, -0.26529004630313807, 0.16406094968746698, -0.05585053606381855, -0.010471975511965976, 0.0, -1.7453292519943295, 0.26529004630313807, 0.16406094968746698, 0.05585053606381855]

        # These represent a pose as in the image https://goo.gl/hYa15h
        self.banzai_pose_larm_goal = [-0.0280391167993, 0.558512828409, 0.584801820449, 0.468552399035, -0.546097642377, -0.449475560632, 0.529346516701]
        self.banzai_pose_rarm_goal = [0.0765219167208, -0.527210000725, 0.638387081642, -0.185009037721, -0.683111796219, 0.184872589841, 0.681873929223]
        self.banzai_jointvals_goal = [1.3591412928962834, -1.5269810342586994, -1.5263864987632225, -0.212938998306429, -0.19093239258017988, -1.5171864827145887,
                                      -0.7066724299606867, -1.9314110634425135, -1.4268663042616962, 1.0613942164863952, 0.9037643195141568, 1.835342100423069]

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

    def test_set_pose_target_rpy(self):
        # #rpy ver
        target = [0.2035, -0.5399, 0.0709, 0,-1.6,0]
        self.rarm.set_pose_target(target)
        self.rarm.plan()
        self.assertTrue(self.rarm.go())

    def test_set_pose_target_quarternion(self):
        target = [0.2035, -0.5399, 0.0709, 0.000427, 0.000317, -0.000384, 0.999999]
        self.rarm.set_pose_target(target)
        self.rarm.plan()
        self.assertTrue(self.rarm.go())

    def test_botharms_compare_joints(self):
        '''Comparing joint names.'''
        self.assertItemsEqual(self.botharms.get_joints(), self._botharms_joints)

    def test_botharms_get_joint_values(self):
        '''
        "both_arm" move group can't set pose target for now (July, 2015) due to 
        missing eef in the moveit config. Here checking if MoveGroup.get_joint_values is working.
        '''

        self.larm.set_pose_target(self.banzai_pose_larm_goal)
        self.larm.plan()
        self.larm.go()
        self.rarm.set_pose_target(self.banzai_pose_rarm_goal)
        self.rarm.plan()
        self.rarm.go()
        # Comparing to the 3rd degree seems too aggressive; example output:
        #   x: array([ 1.35906843, -1.52695742, -1.52658066, -0.21309358, -0.19092542,
        #             -1.51707957, -0.70651268, -1.93170852, -1.42660669,  1.0629058 ,
        #              0.90412021,  1.83650476])
        #   y: array([ 1.35914129, -1.52698103, -1.5263865 , -0.212939  , -0.19093239,
        #             -1.51718648, -0.70667243, -1.93141106, -1.4268663 ,  1.06139422,
        #              0.90376432,  1.8353421 ])
        numpy.testing.assert_almost_equal(self.botharms.get_current_joint_values(), self.banzai_jointvals_goal, 2)

    def test_botharms_set_named_target(self):
        '''
        Test if moveit_commander.set_named_target brings the arms to the init_rtm pose defined in HiroNx.srdf.
        '''
        # Move the arms to non init pose.
        self.larm.set_pose_target(self.banzai_pose_larm_goal)
        self.larm.plan()
        self.larm.go()
        self.rarm.set_pose_target(self.banzai_pose_rarm_goal)
        self.rarm.plan()
        self.rarm.go()

        self.botharms.set_named_target('init_rtm')
        self.botharms.plan()
        self.botharms.go()

        # Raises AssertException when the assertion fails, which automatically be flagged false by unittest framework.
        # http://stackoverflow.com/a/4319836/577001
        numpy.testing.assert_almost_equal(self.botharms.get_current_joint_values(), self.init_rtm_jointvals, 3)

#    def test_simple_unittest(self):
#        self.assertEqual(1, 1)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_hironx_moveit_run', TestHironxMoveit)
