#!/usr/bin/env python

import rospy, actionlib
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

import unittest

class TestHeadAction(unittest.TestCase):

    client = None

    @classmethod
    def setUpClass(self):
        rospy.init_node('head_action_client')
        self.client = actionlib.SimpleActionClient('head_controller/point_head_action', PointHeadAction)
        rospy.loginfo("wait_for_server")
        self.client.wait_for_server()

    def test_head_goal(self):
        goal =  PointHeadGoal()
        point = PointStamped()
        point.header.frame_id = '/WAIST'
        point.point.x = 2
        point.point.y = 0
        point.point.z = 1
        goal.target = point
        goal.pointing_frame = "HEAD_JOINT1_Link"
        goal.pointing_axis.x = 1;
        goal.pointing_axis.y = 0;
        goal.pointing_axis.z = 0;
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 1.0
        rospy.loginfo("send_goal")
        self.client.send_goal(goal)
        rospy.loginfo("wait_for_result")
        self.client.wait_for_result()
        self.assertTrue(self.client.get_result())

# for debug
# $ python -m unittest test_head_action.TestHeadAction.test_head_goal
#
if __name__ == '__main__':
    import rostest
    rostest.rosrun('hironx_head_action', 'test_head_action', TestHeadAction) 

