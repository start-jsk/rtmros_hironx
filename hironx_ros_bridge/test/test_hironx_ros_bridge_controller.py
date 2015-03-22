#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx_ros_bridge import *

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



