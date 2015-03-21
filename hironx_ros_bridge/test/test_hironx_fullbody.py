#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx import *

class TestHiroFullbody(TestHiro):

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

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_fullbody', TestHiroFullbody) 
