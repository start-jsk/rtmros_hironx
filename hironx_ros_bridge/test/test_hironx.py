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

    def test_goInitial(self):
        self.limbbody_init()
        self.robot.goInitial()

#    def test_goOffPose(self):
#        self.limbbody_init()
#        self.robot.goOffPose()

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

    def fullbody_init (self):
        self.filenames = []
        self.robot.seq_svc.removeJointGroup("larm")
        self.robot.seq_svc.removeJointGroup("rarm")
        self.robot.seq_svc.removeJointGroup("head")
        #self.robot.seq_svc.removeJointGroup("torso") we use torso for sleep certain time, so not remove this
        self.filename_base = tempfile.mkstemp()[1]

    def test_fullbody_setJointAngles_Wait (self):
        self.fullbody_init()
        # move to initiali position
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
        self.robot.waitInterpolation()

        self.robot.clearLog()
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
        self.robot.waitInterpolation()
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
        self.robot.waitInterpolation()
        filename = self.filename_base + "-wait"
        self.robot.saveLog(filename)
        q_filename = filename+"."+self.robot.rh.name()+"_q"
        data = self.load_log_data(q_filename)

        print "check setJointAngles(wait=True)"
        self.check_log_data(data, 6, 10.0, -140.0, -100.0)
        return True

    def test_fullbody_setJointAngles_NoWait (self):
        self.fullbody_init()
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
        self.robot.waitInterpolation()
        for i in range(len(clear_time)):
            self.robot.clearLog()
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.setJointAnglesOfGroup("torso", [0], clear_time[i], wait=True);#  time.sleep(clear_time[i]);

            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.waitInterpolation()
            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            self.robot.saveLog(filename)
            q_filename = filename+"."+self.robot.rh.name()+"_q"
            data = self.load_log_data(q_filename)

            print "check setJointAngles(wait=True)"
            self.check_log_data(data, 6, (10.0 - (5 - clear_time[i])), [-140+i*40/len(clear_time),20], -100.0)

    def test_fullbody_setJointAngles_Clear (self):
        self.fullbody_init()
        # check if clear stops interpolation
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.waitInterpolation()
            self.robot.clearLog()
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5-clear_time[i]);
            self.robot.waitInterpolation()
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.setJointAnglesOfGroup("torso", [0], clear_time[i], wait=True);#  time.sleep(clear_time[i]);

            self.robot.clear()
            self.robot.waitInterpolation()
            filename = self.filename_base + "-clear-"+str(clear_time[i])
            self.robot.saveLog(filename)
            q_filename = filename+"."+self.robot.rh.name()+"_q"
            data = self.load_log_data(q_filename)

            print "check setJointAngles(Clear)"
            self.check_log_data(data, 6, 5, [-140+(i+1)*40/len(clear_time),20], -100.0, acc_thre=0.2)

    def test_fullbody_setJointAngles_minus(self):
        self.fullbody_init()
        self.robot.el_svc.setServoErrorLimit("all", 0.01) # default is 0.18
        
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 3);
        self.robot.waitInterpolation()
        self.robot.clearLog()
        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 1);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], -1);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 3);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 0);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 1);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 0.1);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 1.0);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 0.05);
        self.robot.waitInterpolation()

        self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 1.0);
        self.robot.waitInterpolation()

        filename = self.filename_base + "-minus"
        self.robot.saveLog(filename)
        # write pdf file
        q_filename = filename+"."+self.robot.rh.name()+"_q"
        self.write_all_joint_pdf(q_filename, "full_minus_check.pdf")

        # assertion
        data = self.load_log_data(q_filename)
        print "check setJointAngles(minus)"
        self.check_log_data(data, 6, 7.19, -140, -120.0, acc_thre = 1.0)

        self.robot.el_svc.setServoErrorLimit("all", 0.18) # default is 0.18

    ####

    def limbbody_init (self):
        self.filenames = []
        self.robot.setSelfGroups()
        self.filename_base = tempfile.mkstemp()[1]

    def assertTrue(self,a):
        assert(a)

    def test_rarm_setJointAngles_Wait (self):
        self.limbbody_init()
        # move to initiali position
        self.robot.setJointAnglesOfGroup("rarm",[-0.6, 0, -120, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.clearLog()

        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")

        filename = self.filename_base + "-wait"
        self.robot.saveLog(filename)
        q_filename = filename+"."+self.robot.rh.name()+"_q"
        data = self.load_log_data(q_filename)

        print "check setJointAnglesOfGroup(wait=True)"
        self.check_log_data(data, 6, 15.0, -140.0, -100.0)
        return True

    def test_rarm_setJointAngles_NoWait (self):
        self.limbbody_init()
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        for i in range(len(clear_time)):
            self.robot.clearLog()

            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.setJointAnglesOfGroup("larm", [-0.6, 0, -100, 15.2, 9.4, 3.2], clear_time[i], wait=True);#  time.sleep(clear_time[i]);
            self.robot.setJointAnglesOfGroup("rarm",[-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")

            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            self.robot.saveLog(filename)
            q_filename = filename+"."+self.robot.rh.name()+"_q"
            data = self.load_log_data(q_filename)

            print "check setJointAnglesOfGroup(wait=True) "+str(clear_time[i])
            self.check_log_data(data, 6, (10.0 - (5 - clear_time[i])), [(-140+i*40/len(clear_time)),20], -100.0)


    def test_rarm_setJointAngles_Clear (self):
        self.limbbody_init()
        # check if clear stops interpolation
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.clearLog()

            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.setJointAnglesOfGroup("larm", [-0.6, 0, -100, 15.2, 9.4, 3.2], clear_time[i], wait=True);#  time.sleep(clear_time[i]);
            self.robot.clearOfGroup("rarm")
            self.robot.waitInterpolationOfGroup("rarm")

            filename = self.filename_base + "-clear-"+str(clear_time[i])
            self.robot.saveLog(filename)
            q_filename = filename+"."+self.robot.rh.name()+"_q"
            data = self.load_log_data(q_filename)

            print "check setJointAnglesOfGroup(clear) "+str(clear_time[i])
            self.check_log_data(data, 6, (5 + clear_time[i]), [(-140+i*40/len(clear_time)),20], -100.0)

    def test_rarm_setJointAnglesOfGroup_Override_Acceleration (self):
        self.limbbody_init()
        #disconnectPorts(self.robot.rh.port("q"), self.robot.log.port("q"))
        #connectPorts(self.robot.el.port("q"), self.robot.log.port("q"))

        #self.robot.setJointAnglesOfGroup("rarm", [ 25, -139, -157,  45, 0, 0], 3, wait=False);
        #self.robot.setJointAnglesOfGroup("larm", [-25, -139, -157, -45, 0, 0], 3, wait=False);
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 3, wait=False);
        self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 3, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.clearLog()
        #self.robot.log_svc.maxLength(200*30)

        # self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 3, wait=False);
        # time.sleep(1.5);
        # self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 3, wait=False);
        # self.robot.waitInterpolationOfGroup("rarm")
        # self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 3, wait=False);
        # self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], 3, wait=False);
        # time.sleep(1.5);
        # self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 3, wait=False);
        # self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 3, wait=False);
        # self.robot.waitInterpolationOfGroup("rarm")
        # self.robot.waitInterpolationOfGroup("larm")
        # self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 0.1, wait=False);
        # self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 0.1, wait=False);
        # self.robot.waitInterpolationOfGroup("rarm")
        # self.robot.waitInterpolationOfGroup("larm")
        # self.robot.setJointAnglesOfGroup("rarm", [ 25, -139, -157,  45, 0, 0], 3, wait=False);
        # self.robot.setJointAnglesOfGroup("larm", [-25, -139, -157, -45, 0, 0], 3, wait=False);
        # time.sleep(2.5);
        # for i in range(1,10):
        #     time.sleep(i/30.0)
        #     self.robot.setJointAnglesOfGroup("larm",
        #                                      [ 0.6, 0, -120,-15.2, 9.4,-3.2]+numpy.random.normal(0,1,6),
        #                                      3, wait=False);

        for i in range(3):
            self.robot.setJointAnglesOfGroup("rarm",
                                             [-0.6, 0, -100, 15.2, 9.4, 3.2]+numpy.random.normal(0,1,6),
                                             3, wait=False);
            self.robot.setJointAnglesOfGroup("larm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 1.5, wait=True);#  time.sleep(clear_time[i]);

        #self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 1, wait=False);
        #self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 1, wait=False);
        #self.robot.waitInterpolationOfGroup("rarm")
        #self.robot.waitInterpolationOfGroup("larm")
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 3, wait=False);
        self.robot.setJointAnglesOfGroup("larm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 1.5, wait=True);#  time.sleep(clear_time[i]);
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 3, wait=False);
        #time.sleep(1.5)
        #self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 0, wait=False);
        #self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 0, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")

        filename = self.filename_base + "-no-wait2"
        self.robot.saveLog(filename)

        # write pdf file
        q_filename = filename+"."+self.robot.rh.name()+"_q"
        self.write_all_joint_pdf(q_filename, "rarm_accel_check.pdf")

        # assertion
        data = self.load_log_data(q_filename)
        print "check setJointAnglesOfGroup(Override_acceleratoin)"
        self.check_log_data(data, 6, 9, -135, -100.0)


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

    def test_rarm_setJointAnglesOfGroup_minus(self):
        self.limbbody_init()
        self.robot.el_svc.setServoErrorLimit("all", 0.01) # default is 0.18

        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 3, wait=False);
        self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 3, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.clearLog()
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 1, wait=False);
        self.robot.setJointAnglesOfGroup("larm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 1, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")

        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], -1, wait=True);

        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], -1, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], 3, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 0, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 1.0, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], 0.1, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], 1.0, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 0.05, wait=True);
        self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -120,-15.2, 9.4,-3.2], 1.0, wait=True);
        #self.robot.setJointAnglesOfGroup("rarm", [ 0.6, 0, -140,-15.2, 9.4,-3.2], 1.0, wait=True);

        filename = self.filename_base + "-minus"
        self.robot.saveLog(filename)
        # write pdf file
        q_filename = filename+"."+self.robot.rh.name()+"_q"
        self.write_all_joint_pdf(q_filename, "rarm_minus_check.pdf")

        # assertion
        data = self.load_log_data(q_filename)
        print "check setJointAnglesOfGroup(minus)"
        self.check_log_data(data, 6, 7.19, -140, -120.0, acc_thre = 1.5, tm_thre = 0.3)

        self.robot.el_svc.setServoErrorLimit("all", 0.18) # default is 0.18

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
        rpy2 = self.robot.getCurrentRPY('LARM_JOINT5', 'CHEST_JOINT0')
        numpy.testing.assert_array_almost_equal(numpy.array(pos1),numpy.array(pos2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rot1),numpy.array(rot2), decimal=3)
        numpy.testing.assert_array_almost_equal(numpy.array(rpy1),numpy.array(rpy2), decimal=3)

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx', TestHiro) 

