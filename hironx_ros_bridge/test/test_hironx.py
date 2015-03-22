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
import re

from rtm import connectPorts, disconnectPorts

class TestHiro(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        #modelfile = rospy.get_param("hironx/collada_model_filepath")
        #rtm.nshost = 'hiro024'
        #robotname = "RobotHardware0"

        cls.robot = hironx.HIRONX()
        #cls.robot.init(robotname=robotname, url=modelfile)
        cls.robot.init()

    @classmethod
    def tearDownClass(cls):
        #self.write_output_to_pdf("test-hironx.pdf") # don't know how to call this mehtod
        True

    ###
    def set_joint_angles_no_wait_test (self):
        # send initial pose
        av0 = [-0.6, 0, -100, 15.2, 9.4, 3.2]
        av1 = [-0.6, -90, -100, 15.2, 9.4, 3.2]
        self.robot.setJointAnglesOfGroup( "rarm" , av0,  5)
        pose0 = robot.getReferencePose("RARM_JOINT5")
        print pose0

        # make sure setJointAnglesOfGroup with wait
        tm0 = time.time()
        self.robot.setJointAnglesOfGroup( "rarm" , av1,  5)
        self.robot.setJointAnglesOfGroup( "rarm" , av0,  5)
        tm1 = time.time()
        pose1 = self.robot.getReferencePose("RARM_JOINT5")
        print "setJointAnglesOfGroup : reach goal?  ", numpy.linalg.norm(numpy.array(pose1)-numpy.array(pose0)) < 5.0e-3
        print "                      : spend 10 sec?", abs((tm1 - tm0) - 10.0) < 0.1, " " , tm1 - tm0

        # make sure setJointAnglesOfGorup with wita=False returns immediately
        self.robot.setJointAnglesOfGroup( "rarm" , av1,  5)
        tm0 = time.time()
        self.robot.setJointAnglesOfGroup( "rarm" , av0,  5, wait=False)
        tm1 = time.time()
        pose1 = self.robot.getReferencePose("RARM_JOINT5")
        print "setJointAnglesOfGroup(wait=False)", numpy.linalg.norm(numpy.array(pose1)-numpy.array(pose0)) > 5.0e-3
        print "                      : spend 10 sec?", abs((tm1 - tm0) - 0.0) < 0.1, " " , tm1 - tm0

        # make sure that waitInterpolationOfGroup wait until target reaches the goal
        self.robot.waitInterpolationOfGroup("rarm")
        tm1 = time.time()
        pose1 = self.robot.getReferencePose("RARM_JOINT5")
        print "waitInterpolationOfGroup", numpy.linalg.norm(numpy.array(pose1)-numpy.array(pose0)) < 5.0e-3
        print "                      : spend 10 sec?", abs((tm1 - tm0) - 5.0) < 0.1, " " , tm1 - tm0

    # load from log data, [[p0,p1,....p25],[p0,p1,....p25],...,[p0,p1,....p25]]
    def check_q_data(self,name):
        import math
        data = []
        name = name+".q"
        f = open(name)
        start_time = None
        for line in f:
            current_time = float(line.split(' ')[0])
            current_value = float(line.split(' ')[7])
            if not start_time:
                start_time = current_time
            data.append([current_time, current_value*180/math.pi])
        # print "%f %f" % (current_time - start_time, current_value*180/math.pi)
        f.close()
        self.filenames.append(name)
        return data

    # load from log data, [[p0,p1,....p25],[p0,p1,....p25],...,[p0,p1,....p25]] # degree
    def load_log_data(self,name):
        import math
        data = []
        f = open(name)
        start_time = None
        for line in f:
            data.append([float(i)*180/math.pi for i in line.split(' ')[1:-1]])
        # print "%f %f" % (current_time - start_time, current_value*180/math.pi)
        f.close()
        self.filenames.append(name)
        return data

    def check_log_data(self, data, idx, tm_data, min_data, max_data, acc_thre=0.06, tm_thre=0.1): # expected, time, min, max of index
        _tm_data = len(data)/200.0
        _min_data = min([d[idx] for d in data])
        _max_data = max([d[idx] for d in data])
        _tm_thre = tm_thre
        # min_data = [min_data, min_thre]
        if isinstance(min_data, (int, float)):
            min_data = [min_data, 5]
        if isinstance(max_data, (int, float)):
            max_data = [max_data, 5]

        print "time (= ", _tm_data, ") == ", tm_data, " -> ", abs(_tm_data - tm_data) < tm_data*_tm_thre
        print " min (= ", _min_data, ") == ", min_data, " -> ", abs(_min_data - min_data[0]) < min_data[1]
        print " max (= ", _max_data, ") == ", max_data, " -> ", abs(_max_data - max_data[0]) < max_data[1]
        self.assertTrue(abs(_tm_data - tm_data) < tm_data*_tm_thre)
        self.assertTrue(abs(_min_data - min_data[0]) < min_data[1])
        self.assertTrue(abs(_max_data - max_data[0]) < max_data[1])

        # check acceleration
        flag = True
        for i in range(1, len(data)-1):
            for j in range(len(data[i])):
                p0 = data[i-1][j]
                p1 = data[i+0][j]
                p2 = data[i+1][j]
                v0 = p1 - p0
                v1 = p2 - p1
                if abs(v1 - v0) > acc_thre:
                    flag = False
                    print("Acceleration vaiorated! : n = %d, idx %d, p0 %f, p1 %f, p2 %f, v1 %f, v2 %f, acc %f (%f)" % (i, j, p0, p1, p2, v0, v1, v1-v0, (v1-v0)/180.0*math.pi))
        self.assertTrue(flag)

    def write_d_dd_data(self, name):
        name_d = os.path.splitext(name)[0]+".dq"
        name_dd = os.path.splitext(name)[0]+".ddq"
        f = open(name)
        f_d = open(name_d, 'w')
        f_dd = open(name_dd, 'w')
        line_0 = None
        line_1 = None
        line_2 = None
        for line_0 in f:
            if line_2 and line_1 :
                tm = float(line_1.split(' ')[0])
                f_d.write(str(tm))
                f_dd.write(str(tm))
                for i in range(1,len(line_0.split(' '))-1):
                    p0 = float(line_0.split(' ')[i])
                    p1 = float(line_1.split(' ')[i])
                    p2 = float(line_2.split(' ')[i])
                    v0 = p1 - p0
                    v1 = p2 - p1
                    f_d.write(" %.10f"%v0)
                    f_dd.write(" %.10f"%(v1 - v0))
                f_d.write('\n')
                f_dd.write('\n')
            line_2 = line_1
            line_1 = line_0
        f.close()
        f_d.close()
        f_dd.close()
        print "[%s] write %s"%(__file__, name_d)
        print "[%s] write %s"%(__file__, name_dd)

    def write_all_joint_pdf(self, name, pdf_name):
        print "[%s] write pdf %s from log data %s"%(__file__, pdf_name, name)
        self.write_d_dd_data(name)
        _pdf_names = []
        for _name in [name, os.path.splitext(name)[0]+".dq", os.path.splitext(name)[0]+".ddq"]:
            _pdf_names.append(_name+"_"+pdf_name)
            cmd = "gnuplot -p -e \"set terminal pdf; set output '"+_pdf_names[-1]+"'; plot "
            f = open(_name)
            l = f.readline().split(' ')[:-1]
            f.close()
            for i in range(1,len(l)):
                cmd += "'"+_name+"' using 1:"+str(i+1)+" title 'joint "+str(i)+"' with lines"
                if i != len(l)-1:
                    cmd += ","
            cmd += "\""
            os.system(cmd)
        cmd_str = 'pdfunite '+' '.join(_pdf_names) + ' ' + pdf_name
        cmd_str = cmd_str.replace('(', '\(')
        cmd_str = cmd_str.replace(')', '\)')
        os.system(cmd_str)
        return

    def check_acceleration(self, name):
        name1 = name+".q"
        name2 = name+".acc"
        f1 = open(name1)
        f2 = open(name2, 'w')
        line_0 = None
        line_1 = None
        line_2 = None
        for line_0 in f1:
            if line_2 and line_1 :
                tm = float(line_1.split(' ')[0])
                f2.write(str(tm))
                for i in range(1,len(line_0.split(' '))-1):
                    p0 = float(line_0.split(' ')[i])
                    p1 = float(line_1.split(' ')[i])
                    p2 = float(line_2.split(' ')[i])
                    v0 = p1 - p0
                    v1 = p2 - p1
                    if abs(v1 - v0) > 0.0001:
                        print("%s Acceleration vaiorated! : p0 %f, p1 %f, p2 %f, v1 %f, v2 %f, acc %f" % (name, p0, p1, p2, v0, v1, v1-v0))
                    self.assertTrue(abs(v1 - v0) < 0.0001)
                    f2.write(' ' + str(v1 - v0))
                f2.write('\n')
            line_2 = line_1
            line_1 = line_0
        f1.close()
        f2.close()


    def assertTrue(self,a):
        assert(a)

    def write_output_to_pdf (self,name):
        print ";; write log data to "+name
        global filenames
        import os
        cmd = "gnuplot -p -e \"set terminal pdf; set output '"+name+"'; plot "
        for name in self.filenames:
            cmd += "'"+name+"' using 0:8 title '"+name+"' with lines"
            if name != self.filenames[-1]:
                cmd += ","
        cmd += "\""
        os.system(cmd)
        return cmd


# for debug
# $ python -m unittest test_hironx.TestHiro.test_impedance_controller
#
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx', TestHiro) 

