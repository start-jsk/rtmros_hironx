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
from hrpsys.hrpsys_config import euler_from_matrix
from hrpsys import rtm

import os
import unittest
import time
import tempfile

import math
import random
import numpy

from rtm import connectPorts, disconnectPorts

class TestHiro(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        modelfile = rospy.get_param("hironx/collada_model_filepath")
        rtm.nshost = 'hiro014'
        robotname = "RobotHardware0"

        cls.robot = hironx.HIRONX()
        cls.robot.init(robotname=robotname, url=modelfile)

    @classmethod
    def tearDownClass(cls):
        #self.write_output_to_pdf("test-hironx.pdf") # don't know how to call this mehtod
        True

    def test_log(self):
        av_r = [-0.6, -90, -100, 15.2, 9.4, 3.2]
        av_l = [0.6, -90, -100, -15.2, 9.4,-3.2]
        self.robot.servoOn()
        self.robot.goInitial()
        self.robot.log_svc.maxLength(200*15*3)
        self.robot.clearLog()
        for j in range(3):
            self.robot.setJointAnglesOfGroup( "rarm" , av_r,  5)
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.setJointAnglesOfGroup( "larm" , av_l,  5)
            self.robot.waitInterpolationOfGroup("larm")
            self.robot.goInitial(tm=5)
        self.robot.saveLog("/tmp/test_hironx_log")
        self.robot.goOffPose()
        

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_log', TestHiro) 
    print("===================================================")
    print("# Please consult test result with following process")
    print("sh /tmp/check-test_hironx_log.sh")
    command = "scp hiro@hiro014:/tmp/test_hironx_log.* /tmp/;"
    command += "gnuplot -e 'set style line 1 pointsize 1; plot "
    for i in [5,6,7,8,9,10, 11,12,13,14,15,16]:
        command += '"/tmp/test_hironx_log.RobotHardware0_q" using 1:%d with point pointtype %d, '%(i, i-4)
        command += '"/tmp/test_hironx_log.sh_qOut" using 1:%d with linespoints pointtype %d, '%(i, i-4)
    command += "0; pause -1'"
    with open("/tmp/check-test_hironx_log.sh", "w") as f:
        f.write(command)

