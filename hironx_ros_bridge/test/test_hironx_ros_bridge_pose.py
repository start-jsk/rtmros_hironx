#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx_ros_bridge import *

class TestHiroROSBridgePose(TestHiroROSBridge):

    def test_LArmIK(self):
        #   /WAIST /LARM_JOINT5_Link
        # - Translation: [0.325, 0.182, 0.074]
        # - Rotation: in Quaternion [-0.000, -0.707, 0.000, 0.707]
        #             in RPY [-1.694, -1.571, 1.693]
        for torso_angle in ([0, -10, 10]):
            torso_goal = self.goal_Torso()
            torso_goal = self.setup_Positions(torso_goal, [[torso_angle]])
            self.torso.send_goal_and_wait(torso_goal)
            for p in [[ 0.325, 0.182, 0.074], # initial
                      [ 0.3, -0.2, 0.074], [ 0.3, -0.1, 0.074], 
                      #[ 0.3,  0.0, 0.074], [ 0.3,  0.1, 0.074], 
                      [ 0.3,  0.2, 0.074], [ 0.3,  0.3, 0.074],
                      [ 0.4, -0.1, 0.074], [ 0.4, -0.0, 0.074],
                      #[ 0.4,  0.1, 0.074], [ 0.4,  0.2, 0.074], 
                      [ 0.4,  0.3, 0.074], [ 0.4,  0.4, 0.074],
                      ] :

                print "solve ik at p = ", p
                ret = self.set_target_pose('LARM', p, [-1.694,-1.571, 1.693], 1.0)
                self.assertTrue(ret.operation_return, "ik failed")
                ret = self.wait_interpolation_of_group('LARM')
                self.assertTrue(ret.operation_return, "wait interpolation failed")

                rospy.sleep(1)
                now = rospy.Time.now()
                self.listener.waitForTransform("WAIST", "LARM_JOINT5_Link", now, rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform("WAIST", "LARM_JOINT5_Link", now)
                numpy.testing.assert_array_almost_equal(trans, p, decimal=1)
                print "current position   p = ", trans
                print "                 rot = ", rot
                print "                     = ", quaternion_matrix(rot)[0:3,0:3]
                # this fails?
                #numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                #                                        numpy.array([[ 0,  0, -1],
                #                                                     [ 0,  1,  0],
                #                                                     [ 1,  0,  0]]),
                #                                        decimal=3)


    def test_LArm(self):
        goal = self.goal_LArm()
        goal = self.setup_Positions(goal, [[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           [  25,-139,-157,  45,   0,   0],
                                           [ 0.618, -0.157,-100.0,-15.212, 9.501, -3.188]])
        self.larm.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "LARM_JOINT5_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "LARM_JOINT5_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.325493, 0.18236, 0.0744674], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[ 0, 0,-1],
                                                             [ 0, 1, 0],
                                                             [ 1, 0, 0]]), decimal=2)

    def test_RArm(self):
        goal = self.goal_RArm()
        goal = self.setup_Positions(goal, [[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           [  25,-139,-157,  45,   0,   0],
                                           [-0.618, -0.157,-100.0,15.212, 9.501, 3.188]])
        self.rarm.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "RARM_JOINT5_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "RARM_JOINT5_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.325493,-0.18236, 0.0744674], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[ 0, 0,-1],
                                                             [ 0, 1, 0],
                                                             [ 1, 0, 0]]), decimal=2)

    def test_Torso(self):
        goal = self.goal_Torso()
        goal = self.setup_Positions(goal, [[   0.0],
                                           [-162.949],
                                           [ 162.949]])
        self.torso.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "CHEST_JOINT0_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "CHEST_JOINT0_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.0, 0.0, 0.0], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[-0.956044, -0.293223, 0.0],
                                                             [ 0.293223, -0.956044, 0.0],
                                                             [ 0.0,       0.0,      1.0]]), decimal=3)

    def test_Head(self):
        goal = self.goal_Head()
        goal = self.setup_Positions(goal, [[  0.0,  0.0],
                                           [ 70.0, 70.0],
                                           [-70.0,-20.0]])
        self.head.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "HEAD_JOINT1_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "HEAD_JOINT1_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.0, 0.0, 0.5695], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[0.321394, 0.939693, -0.116978],
                                                             [-0.883022, 0.34202, 0.321394],
                                                             [0.34202, 0.0, 0.939693]]), decimal=3)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge_pose', TestHiroROSBridgePose)



