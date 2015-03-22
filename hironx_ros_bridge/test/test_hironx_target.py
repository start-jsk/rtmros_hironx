#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx import *

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

        if not self.robot.setTargetPose('larm', posl1, rpyl1, tm):
            assert(False)
        if not self.robot.setTargetPose('rarm', posr1, rpyr1, tm):
            assert(False)
        print('Before waitInterpolationOfGroup(larm) begins.')
        self.robot.waitInterpolationOfGroup('larm')
        print('waitInterpolationOfGroup(larm) returned.')
        self.robot.waitInterpolationOfGroup('rarm') # just to make sure
        print('waitInterpolationOfGroup(rarm) returned.')
        if not self.robot.setTargetPose('larm', posl2, rpyl1, tm):
            assert(False)
        if not self.robot.setTargetPose('rarm', posr2, rpyr1, tm):
            assert(False)

        # Making sure if reached here. If any error occurred. If not reached
        # assert false should be returned earlier.
        assert(True)  

    def testGetReferencePose(self):
        def print_pose(msg, pose):
            print msg, (pose[3],pose[7],pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')
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
        numpy.testing.assert_array_almost_equal(numpy.array(posel1),numpy.array(posel2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1),numpy.array(poser2), decimal=3)

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
        numpy.testing.assert_array_almost_equal(numpy.array(posel1),numpy.array(posel2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1),numpy.array(poser2), decimal=3)

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
        numpy.testing.assert_array_almost_equal(numpy.array(pos1),numpy.array(pos2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rot1),numpy.array(rot2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rpy1),numpy.array(rpy2), decimal=3)


    def testGetCurrentPose(self):
        def print_pose(msg, pose):
            print msg, (pose[3],pose[7],pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')
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
        numpy.testing.assert_array_almost_equal(numpy.array(posel1),numpy.array(posel2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1),numpy.array(poser2), decimal=3)

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
        numpy.testing.assert_array_almost_equal(numpy.array(posel1),numpy.array(posel2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(poser1),numpy.array(poser2), decimal=3)

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
        numpy.testing.assert_array_almost_equal(numpy.array(pos1),numpy.array(pos2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rot1),numpy.array(rot2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rpy1),numpy.array(rpy2), decimal=3)
    
    def testGetterByFrame(self):
        def print_pose(msg, pose):
            print msg, (pose[3],pose[7],pose[11]), euler_from_matrix([pose[0:3], pose[4:7], pose[8:11]], 'sxyz')

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
        numpy.testing.assert_array_almost_equal(numpy.array(posel1),numpy.array(posel2), decimal=3)
        
        print_pose("robot.getCurrentPose(LARM_JOINT5:DEFAULT)", posel1)
        print_pose("robot.getCurrentPose(LARM_JOINT5:WAIST)", posel2)

        posl1 = self.robot.getCurrentPosition('LARM_JOINT5')
        posl2 = self.robot.getCurrentPosition('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(posl1),numpy.array(posl2), decimal=3)
        
        print "robot.getCurrentPosition(LARM_JOINT5:DEFAULT)", posl1
        print "robot.getCurrentPosition(LARM_JOINT5:WAIST)", posl2

        rotl1 = self.robot.getCurrentRotation('LARM_JOINT5')
        rotl2 = self.robot.getCurrentRotation('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(rotl1),numpy.array(rotl2), decimal=3)
        
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
        numpy.testing.assert_array_almost_equal(numpy.array(rpyl1),numpy.array(rpyl2), decimal=3)
        
        print "robot.getCurrentRPY(LARM_JOINT5:DEFAULT)", rpyl1
        print "robot.getCurrentRPY(LARM_JOINT5:WAIST)", rpyl2

        ref_posel1 = self.robot.getReferencePose('LARM_JOINT5')
        ref_posel2 = self.robot.getReferencePose('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_posel1),numpy.array(ref_posel2), decimal=3)
        
        print "robot.getReferencePose(LARM_JOINT5:DEFAULT)", ref_posel1
        print "robot.getReferencePose(LARM_JOINT5:WAIST)", ref_posel2

        ref_posl1 = self.robot.getReferencePosition('LARM_JOINT5')
        ref_posl2 = self.robot.getReferencePosition('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_posl1),numpy.array(ref_posl2), decimal=3)
        
        print "robot.getReferencePosition(LARM_JOINT5:DEFAULT)", ref_posl1
        print "robot.getReferencePosition(LARM_JOINT5:WAIST)", ref_posl2

        ref_rotl1 = self.robot.getReferenceRotation('LARM_JOINT5')
        ref_rotl2 = self.robot.getReferenceRotation('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_rotl1),numpy.array(ref_rotl2), decimal=3)
        
        print "robot.getReferenceRotation(LARM_JOINT5:DEFAULT)", ref_rotl1
        print "robot.getReferenceRotation(LARM_JOINT5:WAIST)", ref_rotl2

        ref_rpyl1 = self.robot.getReferenceRPY('LARM_JOINT5')
        ref_rpyl2 = self.robot.getReferenceRPY('LARM_JOINT5', 'WAIST')
        numpy.testing.assert_array_almost_equal(numpy.array(ref_rpyl1),numpy.array(ref_rpyl2), decimal=3)
        
        print "robot.getReferenceRPY(LARM_JOINT5:DEFAULT)", ref_rpyl1
        print "robot.getReferenceRPY(LARM_JOINT5:WAIST)", ref_rpyl2

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_target', TestHiroTarget) 
