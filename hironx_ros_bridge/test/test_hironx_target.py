#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy
import re

from tf.transformations import quaternion_from_euler

from test_hironx import euler_from_matrix, TestHiro

PKG = 'hironx_ros_bridge'
RTM_JOINTGRP_LEFT_ARM = 'larm'
RTM_JOINTGRP_RIGHT_ARM = 'rarm'


class TestHiroTarget(TestHiro):

    def testSetTargetPoseBothArm(self):
        tm = 10
        self.robot.goInitial()
        posl1 = self.robot.getCurrentPosition('LARM_JOINT5')
        posl2 = self.robot.getCurrentPosition('LARM_JOINT5')
        posr1 = self.robot.getCurrentPosition('RARM_JOINT5')
        posr2 = self.robot.getCurrentPosition('RARM_JOINT5')
        rpyl1 = self.robot.getCurrentRPY('LARM_JOINT5')
        rpyr1 = self.robot.getCurrentRPY('RARM_JOINT5')
        posr1[0] += 0.05
        posr2[2] += 0.08
        posl1[0] -= 0.09
        posl2[2] -= 0.07
        # Til here initializing.

        if not self.robot.setTargetPose(RTM_JOINTGRP_LEFT_ARM, posl1, rpyl1, tm):
            assert(False)
        if not self.robot.setTargetPose(RTM_JOINTGRP_RIGHT_ARM, posr1, rpyr1, tm):
            assert(False)
        print('Before waitInterpolationOfGroup(larm) begins.')
        self.robot.waitInterpolationOfGroup(RTM_JOINTGRP_LEFT_ARM)
        print('waitInterpolationOfGroup(larm) returned.')
        self.robot.waitInterpolationOfGroup(RTM_JOINTGRP_RIGHT_ARM)  # just to make sure
        print('waitInterpolationOfGroup(rarm) returned.')
        if not self.robot.setTargetPose(RTM_JOINTGRP_LEFT_ARM, posl2, rpyl1, tm):
            assert(False)
        if not self.robot.setTargetPose(RTM_JOINTGRP_RIGHT_ARM, posr2, rpyr1, tm):
            assert(False)

        # Making sure if reached here. If any error occurred. If not reached
        # assert false should be returned earlier.
        assert(True)

    def testGetReferencePose(self):
        def print_pose(msg, pose):
            print msg, (pose[3], pose[7], pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')
        self.robot.goInitial()
        posel1 = self.robot.getReferencePose('LARM_JOINT5')
        poser1 = self.robot.getReferencePose('RARM_JOINT5')
        try:
            posel2 = self.robot.getReferencePose('LARM_JOINT5:WAIST')
            poser2 = self.robot.getReferencePose('RARM_JOINT5:WAIST')
        except RuntimeError as e:
            if re.match(r'frame_name \(.+\) is not supported', e.message):
                print(e.message + "...this is expected so pass the test")
                return True
            elif self.robot.fk.ref.get_component_profile().version <= '315.2.4':
                print("target version is " + self.robot.fk.ref.get_component_profile().version)
                print(e.message + "...this is expected so pass the test")
                return True
            else:
                raise RuntimeError(e.message)
        print_pose("robot.getReferencePose('LARM_JOINT5')", posel1);
        print_pose("robot.getReferencePose('RARM_JOINT5')", poser1);
        print_pose("robot.getReferencePose('LARM_JOINT5:WAIST')", posel2);
        print_pose("robot.getReferencePose('RARM_JOINT5:WAIST')", poser2);
        numpy.testing.assert_array_almost_equal(numpy.array(posel1), numpy.array(posel2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1), numpy.array(poser2), decimal=2)

        posel1 = self.robot.getReferencePose('LARM_JOINT5:CHEST_JOINT0')
        poser1 = self.robot.getReferencePose('RARM_JOINT5:CHEST_JOINT0')
        print_pose("robot.getReferencePose('LARM_JOINT5:CHEST_JOINT0')", posel1);
        print_pose("robot.getReferencePose('RARM_JOINT5:CHEST_JOINT0')", poser1);
        self.robot.setJointAnglesOfGroup('torso', [45], 1)
        self.robot.waitInterpolationOfGroup('torso')
        posel2 = self.robot.getReferencePose('LARM_JOINT5:CHEST_JOINT0')
        poser2 = self.robot.getReferencePose('RARM_JOINT5:CHEST_JOINT0')
        print_pose("robot.getReferencePose('LARM_JOINT5:CHEST_JOINT0')", posel2);
        print_pose("robot.getReferencePose('RARM_JOINT5:CHEST_JOINT0')", poser2);
        numpy.testing.assert_array_almost_equal(numpy.array(posel1), numpy.array(posel2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1), numpy.array(poser2), decimal=2)

        self.robot.setJointAnglesOfGroup('torso', [0], 1)
        self.robot.waitInterpolationOfGroup('torso')
        pos1 = self.robot.getReferencePosition('LARM_JOINT5')
        rot1 = self.robot.getReferenceRotation('LARM_JOINT5')
        rpy1 = self.robot.getReferenceRPY('LARM_JOINT5')

        self.robot.setJointAnglesOfGroup('torso', [0], 1)
        self.robot.waitInterpolationOfGroup('torso')
        pos2 = self.robot.getReferencePosition('LARM_JOINT5', 'CHEST_JOINT0')
        rot2 = self.robot.getReferenceRotation('LARM_JOINT5', 'CHEST_JOINT0')
        rpy2 = self.robot.getReferenceRPY('LARM_JOINT5', 'CHEST_JOINT0')
        numpy.testing.assert_array_almost_equal(numpy.array(pos1), numpy.array(pos2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(rot1), numpy.array(rot2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(rpy1), numpy.array(rpy2), decimal=2)

    def testGetCurrentPose(self):
        def print_pose(msg, pose):
            print msg, (pose[3], pose[7], pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')
        self.robot.goInitial()
        posel1 = self.robot.getCurrentPose('LARM_JOINT5')
        poser1 = self.robot.getCurrentPose('RARM_JOINT5')
        try:
            posel2 = self.robot.getCurrentPose('LARM_JOINT5:WAIST')
            poser2 = self.robot.getCurrentPose('RARM_JOINT5:WAIST')
        except RuntimeError as e:
            if re.match(r'frame_name \(.+\) is not supported', e.message):
                print(e.message + "...this is expected so pass the test")
                return True
            elif self.robot.fk.ref.get_component_profile().version <= '315.2.4':
                print("target version is " + self.robot.fk.ref.get_component_profile().version)
                print(e.message + "...this is expected so pass the test")
                return True
            else:
                raise RuntimeError(e.message)
        print_pose("robot.getCurrentPose('LARM_JOINT5')", posel1);
        print_pose("robot.getCurrentPose('RARM_JOINT5')", poser1);
        print_pose("robot.getCurrentPose('LARM_JOINT5:WAIST')", posel2);
        print_pose("robot.getCurrentPose('RARM_JOINT5:WAIST')", poser2);
        numpy.testing.assert_array_almost_equal(numpy.array(posel1), numpy.array(posel2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1), numpy.array(poser2), decimal=2)

        posel1 = self.robot.getCurrentPose('LARM_JOINT5:CHEST_JOINT0')
        poser1 = self.robot.getCurrentPose('RARM_JOINT5:CHEST_JOINT0')
        print_pose("robot.getCurrentPose('LARM_JOINT5:CHEST_JOINT0')", posel1);
        print_pose("robot.getCurrentPose('RARM_JOINT5:CHEST_JOINT0')", poser1);
        self.robot.setJointAnglesOfGroup('torso', [45], 1)
        self.robot.waitInterpolationOfGroup('torso')
        posel2 = self.robot.getCurrentPose('LARM_JOINT5:CHEST_JOINT0')
        poser2 = self.robot.getCurrentPose('RARM_JOINT5:CHEST_JOINT0')
        print_pose("robot.getCurrentPose('LARM_JOINT5:CHEST_JOINT0')", posel2);
        print_pose("robot.getCurrentPose('RARM_JOINT5:CHEST_JOINT0')", poser2);
        numpy.testing.assert_array_almost_equal(numpy.array(posel1), numpy.array(posel2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1), numpy.array(poser2), decimal=2)

        self.robot.setJointAnglesOfGroup('torso', [0], 1)
        self.robot.waitInterpolationOfGroup('torso')
        pos1 = self.robot.getCurrentPosition('LARM_JOINT5')
        rot1 = self.robot.getCurrentRotation('LARM_JOINT5')
        rpy1 = self.robot.getCurrentRPY('LARM_JOINT5')

        self.robot.setJointAnglesOfGroup('torso', [0], 1)
        self.robot.waitInterpolationOfGroup('torso')
        pos2 = self.robot.getCurrentPosition('LARM_JOINT5', 'CHEST_JOINT0')
        rot2 = self.robot.getCurrentRotation('LARM_JOINT5', 'CHEST_JOINT0')
        try:
            rpy2 = self.robot.getCurrentRPY('LARM_JOINT5', 'CHEST_JOINT0')
        except RuntimeError as e:
            if re.match(r'frame_name \(.+\) is not supported', e.message):
                print(e.message + "...this is expected so pass the test")
            elif self.robot.fk.ref.get_component_profile().version <= '315.2.4':
                print("target hrpsys version is " + self.robot.fk.ref.get_component_profile().version)
                print(e.message + "...this is expected so pass the test")
                return True
            else:
                raise RuntimeError(e.message)
        numpy.testing.assert_array_almost_equal(numpy.array(pos1), numpy.array(pos2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(rot1), numpy.array(rot2), decimal=2)
        numpy.testing.assert_array_almost_equal(numpy.array(rpy1), numpy.array(rpy2), decimal=2)

    def testGetterByFrame(self):
        def print_pose(msg, pose):
            print msg, (pose[3], pose[7], pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')

        self.robot.goInitial()

        posel1 = self.robot.getCurrentPose('LARM_JOINT5')
        try:
            posel2 = self.robot.getCurrentPose('LARM_JOINT5', 'WAIST')
        except RuntimeError as e:
            if re.match(r'frame_name \(.+\) is not supported', e.message):
                print(e.message + "...this is expected so pass the test")
            elif self.robot.fk.ref.get_component_profile().version <= '315.2.4':
                print("target hrpsys version is " + self.robot.fk.ref.get_component_profile().version)
                print(e.message + "...this is expected so pass the test")
                return True
            else:
                raise RuntimeError(e.message)
        numpy.testing.assert_array_almost_equal(numpy.array(posel1), numpy.array(posel2), decimal=2)

        print_pose("robot.getCurrentPose(LARM_JOINT5:DEFAULT)", posel1)
        print_pose("robot.getCurrentPose(LARM_JOINT5:WAIST)", posel2)

        posl1 = self.robot.getCurrentPosition('LARM_JOINT5')
        posl2 = self.robot.getCurrentPosition('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(posl1), numpy.array(posl2), decimal=2)

        print "robot.getCurrentPosition(LARM_JOINT5:DEFAULT)", posl1
        print "robot.getCurrentPosition(LARM_JOINT5:WAIST)", posl2

        rotl1 = self.robot.getCurrentRotation('LARM_JOINT5')
        rotl2 = self.robot.getCurrentRotation('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(rotl1), numpy.array(rotl2), decimal=2)

        print "robot.getCurrentRotation(LARM_JOINT5:DEFAULT)", rotl1
        print "robot.getCurrentRotation(LARM_JOINT5:WAIST)", rotl2

        rpyl1 = self.robot.getCurrentRPY('LARM_JOINT5')
        try:
            rpyl2 = self.robot.getCurrentRPY('LARM_JOINT5', 'WAIST')
        except RuntimeError as e:
            if re.match(r'frame_name \(.+\) is not supported', e.message):
                print(e.message + "...this is expected so pass the test")
            elif self.robot.fk.ref.get_component_profile().version <= '315.2.4':
                print("target hrpsys version is " + self.robot.fk.ref.get_component_profile().version)
                print(e.message + "...this is expected so pass the test")
                return True
            else:
                raise RuntimeError(e.message)
        numpy.testing.assert_array_almost_equal(numpy.array(rpyl1), numpy.array(rpyl2), decimal=2)

        print "robot.getCurrentRPY(LARM_JOINT5:DEFAULT)", rpyl1
        print "robot.getCurrentRPY(LARM_JOINT5:WAIST)", rpyl2

        ref_posel1 = self.robot.getReferencePose('LARM_JOINT5')
        ref_posel2 = self.robot.getReferencePose('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_posel1), numpy.array(ref_posel2), decimal=2)

        print "robot.getReferencePose(LARM_JOINT5:DEFAULT)", ref_posel1
        print "robot.getReferencePose(LARM_JOINT5:WAIST)", ref_posel2

        ref_posl1 = self.robot.getReferencePosition('LARM_JOINT5')
        ref_posl2 = self.robot.getReferencePosition('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_posl1), numpy.array(ref_posl2), decimal=2)

        print "robot.getReferencePosition(LARM_JOINT5:DEFAULT)", ref_posl1
        print "robot.getReferencePosition(LARM_JOINT5:WAIST)", ref_posl2

        ref_rotl1 = self.robot.getReferenceRotation('LARM_JOINT5')
        ref_rotl2 = self.robot.getReferenceRotation('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_rotl1), numpy.array(ref_rotl2), decimal=2)

        print "robot.getReferenceRotation(LARM_JOINT5:DEFAULT)", ref_rotl1
        print "robot.getReferenceRotation(LARM_JOINT5:WAIST)", ref_rotl2

        ref_rpyl1 = self.robot.getReferenceRPY('LARM_JOINT5')
        ref_rpyl2 = self.robot.getReferenceRPY('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_rpyl1), numpy.array(ref_rpyl2), decimal=2)

        print "robot.getReferenceRPY(LARM_JOINT5:DEFAULT)", ref_rpyl1
        print "robot.getReferenceRPY(LARM_JOINT5:WAIST)", ref_rpyl2

    def test_setTargetPoseRelative_rpy(self):
        '''
        Test if with setTargetPoseRelative with RPY values the arm pose becomes as intended.
        Contributed by Naoki Fuse (Daido Steel).
        '''

        print "goInitial", self.robot.getCurrentRPY('RARM_JOINT5'), self.robot.getCurrentRPY('LARM_JOINT5')
        l_eef = 'LARM_JOINT5'
        r_eef = 'RARM_JOINT5'

        init_l = quaternion_from_euler(-3.073, -1.57002, 3.073)  # goInit (not factory setting) pose
        init_r = quaternion_from_euler(3.073, -1.57002, -3.073)  # goInit (not factory setting) pose
        roll_l_post = quaternion_from_euler(-1.502, -1.5700, 3.073)  # dr=math.pi / 2 from init pose
        roll_r_post = quaternion_from_euler(-1.639, -1.5700, -3.073)  # dr=math.pi / 2 from init pose
        pitch_l_post = quaternion_from_euler(-0.000, -0.787, -4.924e-05)  # dp=math.pi / 4 from init pose
        pitch_r_post = quaternion_from_euler(0.000, -0.787, 4.827e-05)  # dp=math.pi / 4 from init pose
        yaw_l_post = quaternion_from_euler(-1.573, -4.205e-05, 1.571)  # dw=math.pi / 2 from init pose
        yaw_r_post = quaternion_from_euler(-1.573, -0.000, 1.571)  # dw=math.pi / 2 from init pose

        # roll motion
        self.robot.goInitial(2)
        for i in range(0, 5):  # Repeat the same movement 5 times
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dr=math.pi / 2, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dr=math.pi / 2, tm=0.5, wait=True)
            roll_l_post_now_rpy = self.robot.getCurrentRPY(l_eef)
            roll_l_post_now = quaternion_from_euler(roll_l_post_now_rpy[0], roll_l_post_now_rpy[1], roll_l_post_now_rpy[2])
            roll_r_post_now_rpy = self.robot.getCurrentRPY(r_eef)
            roll_r_post_now = quaternion_from_euler(roll_r_post_now_rpy[0], roll_r_post_now_rpy[1], roll_r_post_now_rpy[2])
            numpy.testing.assert_array_almost_equal(roll_l_post, roll_l_post_now, decimal=2)
            numpy.testing.assert_array_almost_equal(roll_r_post, roll_r_post_now, decimal=2)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dr=-math.pi / 2, dw=0, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dr=-math.pi / 2, dw=0, tm=0.5, wait=True)
            init_l_now_rpy = self.robot.getCurrentRPY(l_eef)
            init_l_now = quaternion_from_euler(init_l_now_rpy[0], init_l_now_rpy[1], init_l_now_rpy[2])
            init_r_now_rpy = self.robot.getCurrentRPY(r_eef)
            init_r_now = quaternion_from_euler(init_r_now_rpy[0], init_r_now_rpy[1], init_r_now_rpy[2])
            numpy.testing.assert_array_almost_equal(init_l, init_l_now, decimal=2)
            numpy.testing.assert_array_almost_equal(init_r, init_r_now, decimal=2)

        # pitch motion
        self.robot.goInitial(2)
        for i in range(0, 5):
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dp=math.pi / 4, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dp=math.pi / 4, tm=0.5, wait=True)
            pitch_l_post_now_rpy = self.robot.getCurrentRPY(l_eef)
            pitch_l_post_now = quaternion_from_euler(pitch_l_post_now_rpy[0], pitch_l_post_now_rpy[1], pitch_l_post_now_rpy[2])
            pitch_r_post_now_rpy = self.robot.getCurrentRPY(r_eef)
            pitch_r_post_now = quaternion_from_euler(pitch_r_post_now_rpy[0], pitch_r_post_now_rpy[1], pitch_r_post_now_rpy[2])
            numpy.testing.assert_array_almost_equal(pitch_l_post, pitch_l_post_now, decimal=2)
            numpy.testing.assert_array_almost_equal(pitch_r_post, pitch_r_post_now, decimal=2)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dp=-math.pi / 4, dw=0, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dp=-math.pi / 4, dw=0, tm=0.5, wait=True)
            init_l_now_rpy = self.robot.getCurrentRPY(l_eef)
            init_l_now = quaternion_from_euler(init_l_now_rpy[0], init_l_now_rpy[1], init_l_now_rpy[2])
            init_r_now_rpy = self.robot.getCurrentRPY(r_eef)
            init_r_now = quaternion_from_euler(init_r_now_rpy[0], init_r_now_rpy[1], init_r_now_rpy[2])
            numpy.testing.assert_array_almost_equal(init_l, init_l_now, decimal=2)
            numpy.testing.assert_array_almost_equal(init_r, init_r_now, decimal=2)

        # yaw motion
        self.robot.goInitial(2)
        for i in range(0, 5):
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dw=math.pi / 2, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dw=math.pi / 2, tm=0.5, wait=True)
            yaw_l_post_now_rpy = self.robot.getCurrentRPY(l_eef)
            yaw_l_post_now = quaternion_from_euler(yaw_l_post_now_rpy[0], yaw_l_post_now_rpy[1], yaw_l_post_now_rpy[2])
            yaw_r_post_now_rpy = self.robot.getCurrentRPY(r_eef)
            yaw_r_post_now = quaternion_from_euler(yaw_r_post_now_rpy[0], yaw_r_post_now_rpy[1], yaw_r_post_now_rpy[2])
            numpy.testing.assert_array_almost_equal(yaw_l_post, yaw_l_post_now, decimal=2)
            numpy.testing.assert_array_almost_equal(yaw_r_post, yaw_r_post_now, decimal=2)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_LEFT_ARM, l_eef, dw=-math.pi / 2, tm=0.5, wait=False)
            self.robot.setTargetPoseRelative(RTM_JOINTGRP_RIGHT_ARM, r_eef, dw=-math.pi / 2, tm=0.5, wait=True)
            init_l_now_rpy = self.robot.getCurrentRPY(l_eef)
            init_l_now = quaternion_from_euler(init_l_now_rpy[0], init_l_now_rpy[1], init_l_now_rpy[2])
            init_r_now_rpy = self.robot.getCurrentRPY(r_eef)
            init_r_now = quaternion_from_euler(init_r_now_rpy[0], init_r_now_rpy[1], init_r_now_rpy[2])
            numpy.testing.assert_array_almost_equal(init_l, init_l_now, decimal=2)
            numpy.testing.assert_array_almost_equal(init_r, init_r_now, decimal=2)

    def test_get_geometry_methods_noarg(self):
        '''
        @summary: What we call "geometry_methods" are supposed to raise
                  RuntimeError in a normal condition.

        geometry_methods are [
                     'getCurrentPose', 'getCurrentPosition',
                     'getCurrentRPY', 'getCurrentRPYRotation',
                     'getReferencePose', 'getReferencePosition',
                     'getReferenceRotation', 'getReferenceRPY']
        '''
        self.assertRaises(RuntimeError, lambda: self.robot.getCurrentPose())
        self.assertRaises(RuntimeError, lambda: self.robot.getReferencePose())

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_target', TestHiroTarget)
