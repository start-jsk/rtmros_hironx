#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx import *

class TestHiroLimb(TestHiro):

    def limbbody_init (self):
        self.filenames = []
        self.robot.setSelfGroups()
        self.filename_base = tempfile.mkstemp()[1]

    def test_goInitial(self):
        self.limbbody_init()
        self.robot.goInitial()

#    def test_goOffPose(self):
#        self.limbbody_init()
#        self.robot.goOffPose()

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

    def test_movejoints_neck_waist(self):
        '''
        Move neck and waist joints to the positional limit defined in
        the user's manual (with an arbitrary safety coefficient)
        using hrpsys commands.

        This test is originally made to catch issues like
        https://github.com/start-jsk/rtmros_hironx/issues/411
        '''
        safety_coeffiecient = 0.8
        durtion_operation = 10
        # Values claimed by manufacturer
        max_neck_pitch = 0.63
        min_neck_pitch = -0.4
        max_neck_yaw = 1.22
        min_neck_yaw = -1.22
        max_waist_yaw = 2.84
        min_waist_yaw = -2.84

        self.limbbody_init()
        # Here multiplied by safety coefficient to avoid the robot tries
        # the maximum angles, in case this test is used with real robots.
        self.assertTrue(self.robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dp=max_neck_pitch*safety_coeffiecient, tm=durtion_operation))
        self.robot.goInitial()
        self.assertTrue(self.robot.setTargetPoseRelative('head', 'HEAD_JOINT1', dp=min_neck_pitch*safety_coeffiecient, tm=durtion_operation))
        self.robot.goInitial()
        self.assertTrue(self.robot.setTargetPoseRelative('head', 'HEAD_JOINT0', dw=max_neck_yaw*safety_coeffiecient, tm=durtion_operation))
        self.robot.goInitial()
        self.assertTrue(self.robot.setTargetPoseRelative('head', 'HEAD_JOINT0', dw=min_neck_yaw*safety_coeffiecient, tm=durtion_operation))
        self.robot.goInitial()
        self.assertTrue(self.robot.setTargetPoseRelative('torso', 'CHEST_JOINT0', dw=max_waist_yaw*safety_coeffiecient, tm=durtion_operation))
        self.robot.goInitial()
        self.assertTrue(self.robot.setTargetPoseRelative('torso', 'CHEST_JOINT0', dw=min_waist_yaw*safety_coeffiecient, tm=durtion_operation))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_limb', TestHiroLimb) 
