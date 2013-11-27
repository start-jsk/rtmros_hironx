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

import numpy
import unittest
import time
import tempfile

class TestHiroNX(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot = hironx.HIRONX()
        self.robot.init()

    def test_goInitial(self):
        self.limbbody_init()
        self.robot.goInitial()

    def angle_vector_generator(self):
        step = 80
        for i0 in range(-80,80,step):
            for i1 in range(-130,50,step):
                for i2 in range(-150,10,step):
                    for i3 in range(-160,100,step):
                        for i4 in range(-80,80,step):
                            for i5 in range(-160,160,step):
                                yield [i0, i1, i2, i3, i4, i5]

    def _test_ik_joint_angle(self):
        lav = self.angle_vector_generator().next()
        for av in self.angle_vector_generator():
            print "av", av
            self.robot.setJointAnglesOfGroup("LARM", av, 2)
            self.robot.waitInterpolationOfGroup("LARM")
            pos1 = self.robot.getReferencePosition("LARM_JOINT5")
            rpy1 = self.robot.getReferenceRPY("LARM_JOINT5")
            if numpy.linalg.norm(numpy.array(lav) - numpy.array(av)) > 10:
                lav = av
            self.robot.setJointAnglesOfGroup("LARM", lav, 2)
            self.robot.waitInterpolationOfGroup("LARM")
            self.assertTrue(self.robot.setTargetPose("LARM", pos1, rpy1, 5))
            self.robot.waitInterpolationOfGroup("LARM")
            pos2 = self.robot.getReferencePosition("LARM_JOINT5")
            rpy2 = self.robot.getReferenceRPY("LARM_JOINT5")
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3) # 0.005 m
            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<5.0e-3) # 0.286 deg
            lav = av

    def target_point_generator(self,xmin,xmax,ymin,ymax,zmin,zmax,step):
        for x in range(xmin,xmax,step):
            for y in range(ymin,ymax,step):
                for z in range (zmin,zmax,step):
                    yield [x/1000.0, y/1000.0, z/1000.0]

    def test_ik_left(self):
        self.limbbody_init()
        self.pos_ik_test("LARM", 220,400, -5,320, 5,300, 100) # zmax=390

    def test_ik_right(self):
        self.limbbody_init()
        self.pos_ik_test("RARM", 220,400, -320,5,  5,300, 100) # zmax=390

    def pos_ik_test(self, arm, xmin,xmax,ymin,ymax,zmin,zmax,step):
        arm_target = arm+"_JOINT5"
        rot1 = [[0,0,-1],[0,1,0],[1,0,0]]
        for pos1 in self.target_point_generator(xmin,xmax,ymin,ymax,zmin,zmax,step):
            self.robot.goInitial()
            print self.robot.getReferenceRPY(arm_target)
            self.assertTrue(self.robot.setTargetPose(arm, pos1, euler_from_matrix(rot1), 5))
            self.robot.waitInterpolationOfGroup(arm)
            pos2 = self.robot.getReferencePosition(arm_target)
            rot2 = self.robot.getReferenceRotation(arm_target)
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rot1, rot2, numpy.linalg.norm(numpy.array(rot1)-numpy.array(rot2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3) # 0.005 m
            self.assertTrue(numpy.linalg.norm(numpy.array(rot1)-numpy.array(rot2))<5.0e-3) # 0.286 deg

    def test_goOffPose(self):
        self.limbbody_init()
        self.robot.goOffPose()

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

    # load from log data
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

    def fullbody_init (self):
        self.filenames = []
        self.robot.seq_svc.removeJointGroup("larm")
        self.robot.seq_svc.removeJointGroup("rarm")
        self.robot.seq_svc.removeJointGroup("head")
        self.robot.seq_svc.removeJointGroup("torso")
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
        data = self.check_q_data(filename)
        data_time = data[-1][0] - data[0][0]
        min_data = min([d[1] for d in data])
        max_data = max([d[1] for d in data])
        print "check setJointAngles(wait=True),  tm = ", data_time, ", ok?", abs(data_time - 10.0) < 0.1
        print "                                 min = ", min_data, ", ok?", abs(min_data - -140) < 5
        print "                                 max = ", max_data, ", ok?", abs(max_data - -100) < 5

    def test_fullbody_setJointAngles_NoWait (self):
        self.fullbody_init()
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -120, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.waitInterpolation()
            self.robot.clearLog()
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -140, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            time.sleep(clear_time[i]);
            self.robot.setJointAngles([0, 0, 0, 0,   -0.6, 0, -100, 15.2, 9.4, 3.2,  0,0,0,0,    -0.6, 0, -100,-15.2, 9.4,-3.2,   0, 0, 0, 0], 5);
            self.robot.waitInterpolation()
            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            self.robot.saveLog(filename)
            data = self.check_q_data(filename)
            data_time = data[-1][0] - data[0][0]
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAngles(wait=False), tm = ", data_time, ", ok?", abs(data_time - (10.0 - (5 - clear_time[i]))) < 0.1
            print "                                 min = ", min_data, ", ok?", abs(min_data - (-140+i*40/len(clear_time))) < 10, " ", -140+i*40/len(clear_time)
            print "                                 max = ", max_data, ", ok?", abs(max_data - -100) < 5

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
            time.sleep(clear_time[i]);
            self.robot.clear()
            self.robot.waitInterpolation()
            filename = self.filename_base + "-clear-"+str(clear_time[i])
            self.robot.saveLog(filename)
            data = self.check_q_data(filename)
            data_time = data[-1][0] - data[0][0]
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAngles(clear),      tm = ", data_time, ", ok?", abs(data_time - 5) < 0.1
            print "                                 min = ", min_data, ", ok?", abs(min_data - (-140+(i+1)*40/len(clear_time))) < 10, " ", -140+(i+1)*40/len(clear_time)
            print "                                 max = ", max_data, ", ok?", abs(max_data - -100) < 5

    ####

    def limbbody_init (self):
        self.filenames = []
        self.robot.setSelfGroups()
        self.filename_base = tempfile.mkstemp()[1]

    def test_rarm_setJointAngles_Wait (self):
        self.limbbody_init()
        # move to initiali position
        self.robot.setJointAnglesOfGroup("rarm",[-0.6, 0, -120, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.clearLog()
        time.sleep(1.0);
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")
        self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
        self.robot.waitInterpolationOfGroup("rarm")

        time.sleep(1.0);
        filename = self.filename_base + "-wait"
        self.robot.saveLog(filename)
        data = self.check_q_data(filename)
        data_time = data[-1][0] - data[0][0]
        min_data = min([d[1] for d in data])
        max_data = max([d[1] for d in data])
        print "check setJointAnglesOfGroup(wait=True),  tm = ", data_time, ", ok?", abs(data_time - 10.0) < 0.1
        self.assertTrue(abs(data_time - 10.0) < 0.1)
        print "                                        min = ", min_data, ", ok?", abs(min_data - -140) < 5
        self.assertTrue(abs(min_data - -140) < 5)
        print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
        self.assertTrue(abs(max_data - -100) < 5)


    def test_rarm_setJointAngles_NoWait (self):
        self.limbbody_init()
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.clearLog()
            time.sleep(1.0);
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
            time.sleep(clear_time[i]);
            self.robot.setJointAnglesOfGroup("rarm",[-0.6, 0, -100, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            time.sleep(1.0);
            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            self.robot.saveLog(filename)
            data = self.check_q_data(filename)
            data_time = data[-1][0] - data[0][0]
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(wait=False), tm = ", data_time, ", ok?", abs(data_time - (10.0 - (5 - clear_time[i]))) < 0.1
            self.assertTrue(abs(data_time - (10.0 - (5 - clear_time[i]))) < 0.1)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+i*40/len(clear_time))) < 20, " ", -140+i*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+i*40/len(clear_time))) < 20)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

    def test_rarm_setJointAngles_Clear (self):
        self.limbbody_init()
        # check if clear stops interpolation
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -120, 15.2, 9.4, 3.2], 5, wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.clearLog()
            time.sleep(1.0);
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -100, 15.2, 9.4, 3.2], 5-clear_time[i], wait=False);
            self.robot.waitInterpolationOfGroup("rarm")
            self.robot.setJointAnglesOfGroup("rarm", [-0.6, 0, -140, 15.2, 9.4, 3.2], 5, wait=False);
            time.sleep(clear_time[i]);
            self.robot.clearOfGroup("rarm")
            self.robot.waitInterpolationOfGroup("rarm")
            time.sleep(1.0);
            filename = self.filename_base + "-clear-"+str(clear_time[i])
            self.robot.saveLog(filename)
            data = self.check_q_data(filename)
            data_time = data[-1][0] - data[0][0]
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(clear),      tm = ", data_time, ", ok?", abs(data_time - 5) < 0.1
            self.assertTrue(abs(data_time - 5) < 0.1)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+(i+1)*40/len(clear_time))) < 20, " ", -140+(i+1)*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+(i+1)*40/len(clear_time))) < 20)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

    def write_output_to_pdf (self,name):
        import os
        cmd = "gnuplot -p -e \"set terminal pdf; set output '"+name+"'; plot "
        for name in self.filenames:
            cmd += "'"+name+"' using 0:8 title '"+name+"' with lines"
            if name != self.filenames[-1]:
                cmd += ","
        cmd += "\""
        os.system(cmd)
        return cmd


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx', TestHiroNX) 




