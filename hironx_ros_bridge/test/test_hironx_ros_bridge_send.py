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

import rospy
from tf.transformations import quaternion_matrix, euler_from_matrix
import time

from hironx_ros_bridge.testutil.test_rosbridge import TestHiroROSBridge

PKG = 'hironx_ros_bridge'


class TestHiroROSBridgeSend(TestHiroROSBridge):

    def test_send_goal_and_wait(self):
        #self.rarm.send_goal(self.setup_Positions(self.goal_LArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]])) this should returns error
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.joint_states = []
        time.sleep(1.0);
        tm0 = rospy.Time.now()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        tm1 = rospy.Time.now()
        data_time = (tm1 - tm0).to_sec()
        filename = self.filename_base + "-wait"
        data = self.check_q_data(filename)
        min_data = min([d[1] for d in data])
        max_data = max([d[1] for d in data])
        print "check setJointAnglesOfGroup(wait=True),  tm = ", data_time, ", ok?", abs(data_time - 15.0) < 0.1
        self.assertTrue(abs(data_time - 15.0) < 1.0)
        print "                                        min = ", min_data, ", ok?", abs(min_data - -140) < 5
        self.assertTrue(abs(min_data - -140) < 5)
        print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
        self.assertTrue(abs(max_data - -100) < 5)

    def test_send_goal_and_nowait(self):
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        clear_time = [4.5, 3.0, 1.0]
        for i in range(len(clear_time)):
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.joint_states = []
            rospy.sleep(1.0)
            tm0 = rospy.Time.now()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
            rospy.sleep(clear_time[i]);
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            tm1 = rospy.Time.now()
            rospy.sleep(1.0)
            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            data = self.check_q_data(filename)
            data_time = (tm1 - tm0).to_sec()
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(wait=False), tm = ", data_time, ", ok?", abs(data_time - (10.0 - (5 - clear_time[i]))) < 1.5, " ", (10.0 - (5 - clear_time[i]))
            self.assertTrue(abs(data_time - (10.0 - (5 - clear_time[i]))) < 1.5)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+i*40/len(clear_time))) < 20, " ", -140+i*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+i*40/len(clear_time))) < 20)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

    def test_send_goal_and_clear(self):
        clear_time = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5]
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        clear_time = [4.5, 3.0, 1.0]
        for i in range(len(clear_time)):
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.joint_states = []
            rospy.sleep(1.0)
            tm0 = rospy.Time.now()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
            rospy.sleep(clear_time[i])
            self.rarm.cancel_goal()
            self.rarm.wait_for_result()
            tm1 = rospy.Time.now()
            rospy.sleep(1.0)
            filename = self.filename_base + "-clear-"+str(clear_time[i])
            data = self.check_q_data(filename)
            data_time = (tm1 - tm0).to_sec()
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(clear "+str(clear_time[i])+"),  tm = ", data_time, ", ok?", abs(data_time - (10 - (5 - clear_time[i]))) < 0.5, " ", (10 - (5 - clear_time[i]))
            self.assertTrue(abs(data_time - (10 - (5 - clear_time[i]))) < 0.5)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+(i+1)*40/len(clear_time))) < 35, " ", -140+(i+1)*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+(i+1)*40/len(clear_time))) < 35)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge_send', TestHiroROSBridgeSend)



