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
from hrpsys import rtm
from hrpsys.hrpsys_config import euler_from_matrix

import numpy
import unittest
import time
import tempfile

import random
import numpy as np

from rtm import connectPorts, disconnectPorts

class TestHiroIK(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        #modelfile = rospy.get_param("hironx/collada_model_filepath")
        #rtm.nshost = 'hiro014'
        #robotname = "RobotHardware0"

        self.robot = hironx.HIRONX()
        #self.robot.init(robotname=robotname, url=modelfile)
        self.robot.init()

    def angle_vector_generator(self):
        for i0 in range(-80,80,80):
            for i1 in range(-130,50,90):
                for i2 in range(-150,10,80):
                    for i3 in range(-160,100,130):
                        for i4 in range(-80,80,80):
                            for i5 in range(-160,160,160):
                                yield [i0, i1, i2, i3, i4, i5]

# 3/9/2014 Isaac comments out the entire test_ik_joint_angle that can cause left arm to hit its shoulder.
#    def test_ik_joint_angle(self):
#        lav = self.angle_vector_generator().next()
#        for av in self.angle_vector_generator():
#            print "av", av
#            self.robot.setJointAnglesOfGroup("LARM", av, 2)
#            self.robot.waitInterpolationOfGroup("LARM")
#            pos1 = self.robot.getReferencePosition("LARM_JOINT5")
#            rpy1 = self.robot.getReferenceRPY("LARM_JOINT5")
#            if numpy.linalg.norm(numpy.array(lav) - numpy.array(av)) > 10:
#                lav = av
#            self.robot.setJointAnglesOfGroup("LARM", lav, 2)
#            self.robot.waitInterpolationOfGroup("LARM")
#            self.assertTrue(self.robot.setTargetPose("LARM", pos1, rpy1, 5))
#            self.robot.waitInterpolationOfGroup("LARM")
#            pos2 = self.robot.getReferencePosition("LARM_JOINT5")
#            rpy2 = self.robot.getReferenceRPY("LARM_JOINT5")
#            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
#            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
#            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<1.0e-4) # 0.1 mm
#            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<1.0e-3) # 0.001 rad = 0.057296 deg
#            lav = av

    def target_point_generator(self,xmin,xmax,ymin,ymax,zmin,zmax,step):
        for x in range(xmin,xmax,step):
            for y in range(ymin,ymax,step):
                for z in range (zmin,zmax,step):
                    yield [x/1000.0, y/1000.0, z/1000.0]

    def test_ik_left(self):
        self.pos_ik_test("LARM", 220,400, -5,320, 5,300, 100) # zmax=390

    def test_ik_right(self):
        self.pos_ik_test("RARM", 220,400, -320,5,  5,300, 100) # zmax=390

    def pos_ik_test(self, arm, xmin,xmax,ymin,ymax,zmin,zmax,step):
        arm_target = arm+"_JOINT5"
        rot1 = [[0,0,-1],[0,1,0],[1,0,0]]
        for pos1 in self.target_point_generator(xmin,xmax,ymin,ymax,zmin,zmax,step):
            self.robot.goInitial(tm=2)
            print self.robot.getReferenceRPY(arm_target)
            self.assertTrue(self.robot.setTargetPose(arm, pos1, euler_from_matrix(rot1), 1))
            self.robot.waitInterpolationOfGroup(arm)
            pos2 = self.robot.getReferencePosition(arm_target)
            rot2 = self.robot.getReferenceRotation(arm_target)
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rot1, rot2, numpy.linalg.norm(numpy.array(rot1)-numpy.array(rot2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<1.0e-4) # 0.1 mm
            self.assertTrue(numpy.linalg.norm(numpy.array(rot1)-numpy.array(rot2))<1.0e-3) # 0.001 rad = 0.057296 deg

    def test_set_target_pose(self):
        self.robot.setJointAnglesOfGroup("TORSO",[45],3)
        self.robot.waitInterpolationOfGroup("TORSO")
        ret = self.robot.setTargetPose("larm", [0.3255627368715471, 0.1823638733778268, 0.07462449717662004+0.2], [-3.0732189053889805, -1.5690225912054285, 3.0730289207320203], 5, "CHEST_JOINT0")
        self.assertTrue(ret)


# for debug
# $ python -m unittest test_hironx_ik.TestHiroIK.test_set_target_pose
#
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ik', TestHiroIK) 
