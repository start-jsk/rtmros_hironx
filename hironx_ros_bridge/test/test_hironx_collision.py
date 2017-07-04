#!/usr/bin/env python


from test_hironx import TestHiro
import sys
import math

from distutils.version import StrictVersion

PKG = 'hironx_ros_bridge'

def vector_equal_eps (vec1, vec2, eps=1e-5):
    if len(vec1) == len(vec2):
        for e1, e2 in zip(vec1, vec2):
            if abs(e1 - e2) > eps:
                return False
        return True
    else:
        return False

class TestHiroCollision(TestHiro):

    @classmethod
    def setUpClass(cls):
        super(TestHiroCollision, cls).setUpClass()
        cls.fout = open('/tmp/check-test_hironx_collision.txt', 'w')
        cls.fout.write('Condition\tLoop\tComp Time\tRecov Time\tLoop for check\n')

        cls.curr_loop = 1
        try:
            cls.robot.co_svc.setCollisionLoop(cls.curr_loop)
            cls.use_set_collision_loop = True
        except:
            cls.use_set_collision_loop = False

        cls.init_pose = [0]*29
        # col_safe_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,0.15708,-0.113446,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
        # col_fail_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.845363,0.03992,0.250074,-1.32816,0.167513,0.016204,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
        cls.col_safe_pose = [0.0, 0.0, 0.0,
                              0.0, -0.010471975511965976, 0.0,
                              -1.7453292519943295, 0.26529004630313807, 0.16406094968746698,
                              0.05585053606381855, 0.0, 0.0,
                              0.0, 0.0, 0.010471975511963548,
                              0.0, -1.745329251994327, -0.265290046303138,
                              0.16406094968746654, -0.055850536063817735, 0.0,
                              0.0, 0.0, 0.0]
        cls.col_fail_pose = [0.0, 0.0, 0.0,
                              0.0, -0.010471975511965976, 0.0, 
                              -1.7453292519943295, 0.26529004630313807, 0.16406094968746698,
                              0.05585053606381855, 0.0, 0.0,
                              0.0, 0.0, -0.9357196648099176,
                              -0.5020351200853644, -0.7480480183116466, -0.15644273591164157,
                              -0.10807458370637157, 0.9688350378358652, 0.0,
                              0.0, 0.0, 0.0]
        print >> sys.stderr, "hrpsys_version = %s" % cls.robot.hrpsys_version

    def printCollisionState(self, cs):
        print >> sys.stderr, "Collision State:"
        print >> sys.stderr, "\tTime: %f" % cs.time
        print >> sys.stderr, "\tComputation time: %f" % cs.computation_time
        print >> sys.stderr, "\tSafe Posture: %s" % cs.safe_posture
        print >> sys.stderr, "\tRecover time: %f" % cs.recover_time
        print >> sys.stderr, "\tLoop for check: %d" % cs.loop_for_check
        # print >> sys.stderr, cs.angle
        # print >> sys.stderr, cs.collide
        # print >> sys.stderr, cs.lines

    def outputCollisionState(self, cs, condition):
        s = "%.1f\t\t\t%d\t\t%f\t%f\t%d\n" % (condition, self.curr_loop, cs.computation_time, cs.recover_time, cs.loop_for_check)
        self.fout.write(s)
        self.fout.flush()

    # demo functions
    def demoCollisionCheckSafe (self):
        print >> sys.stderr, "1. CollisionCheck in safe pose"
        self.robot.seq_svc.setJointAngles(self.col_safe_pose, 3.0);
        self.robot.waitInterpolation();
        counter = 0
        while (counter < 20) and (not vector_equal_eps([x / 180 * math.pi  for x in self.robot.getJointAngles()], self.col_safe_pose)):
            time.sleep(0.2)
            counter = counter + 1
            # print >> sys.stderr, counter
        assert(counter != 20)
        cs = self.robot.co_svc.getCollisionStatus()[1]
        if cs.safe_posture:
            print >> sys.stderr, "  => Safe pose"
            self.outputCollisionState(cs, 1)
            self.printCollisionState(cs)
        assert(cs.safe_posture is True)

    def demoCollisionCheckFail (self):
        print >> sys.stderr, "2. CollisionCheck in fail pose"
        self.robot.seq_svc.setJointAngles(self.col_fail_pose, 3.0);
        self.robot.waitInterpolation();
        cs = self.robot.co_svc.getCollisionStatus()[1]
        if not cs.safe_posture:
            print >> sys.stderr, "  => Successfully stop fail pose"
            self.outputCollisionState(cs, 2.0)
            self.printCollisionState(cs)
        assert((not cs.safe_posture) is True)
        self.robot.seq_svc.setJointAngles(self.col_safe_pose, 3.0);
        self.robot.waitInterpolation();
        cs=self.robot.co_svc.getCollisionStatus()[1]
        if cs.safe_posture:
            print >> sys.stderr, "  => Successfully return to safe pose"
            self.outputCollisionState(cs, 2.1)
            self.printCollisionState(cs)
        assert(cs.safe_posture is True)

    def demoCollisionCheckFailWithSetTolerance (self):
        print >> sys.stderr, "3. CollisionCheck in fail pose with 0.1[m] tolerance"
        self.robot.co_svc.setTolerance("all", 0.1); # [m]
        self.robot.seq_svc.setJointAngles(self.col_fail_pose, 1.0);
        self.robot.waitInterpolation();
        cs = self.robot.co_svc.getCollisionStatus()[1]
        if not cs.safe_posture:
            print >> sys.stderr, "  => Successfully stop fail pose (0.1[m] tolerance)"
            self.outputCollisionState(cs, 3.0)
            self.printCollisionState(cs)
        if StrictVersion(self.robot.hrpsys_version) >= StrictVersion('315.10.0'): # https://github.com/fkanehiro/hrpsys-base/pull/993 ???
            assert((not cs.safe_posture) is True)
        else:
            print >> sys.stderr, "  => WARNING Failed to stop fail pose (0.1[m] tolerance), because of https://github.com/fkanehiro/hrpsys-base/pull/993 ???"
        self.robot.co_svc.setTolerance("all", 0.0); # [m]
        self.robot.seq_svc.setJointAngles(self.col_safe_pose, 3.0);
        self.robot.waitInterpolation();
        cs = self.robot.co_svc.getCollisionStatus()[1]
        if cs.safe_posture:
            print >> sys.stderr, "  => Successfully return to safe pose"
            self.outputCollisionState(cs, 3.1)
            self.printCollisionState(cs)
        assert(cs.safe_posture is True)

    def demoCollisionDisableEnable (self):
        print >> sys.stderr, "4. CollisionDetection enable and disable"
        self.robot.seq_svc.setJointAngles(self.col_safe_pose, 1.0);
        self.robot.waitInterpolation();
        if self.robot.co_svc.disableCollisionDetection():
            print >> sys.stderr, "  => Successfully disabled when no collision"
        assert(self.robot.co_svc.disableCollisionDetection() is True)
        if self.robot.co_svc.enableCollisionDetection():
            print >> sys.stderr, "  => Successfully enabled when no collision"
        assert(self.robot.co_svc.enableCollisionDetection() is True)
        self.robot.seq_svc.setJointAngles(self.col_fail_pose, 1.0);
        self.robot.waitInterpolation();
        if not self.robot.co_svc.disableCollisionDetection():
            print >> sys.stderr, "  => Successfully inhibit disabling when collision"
        assert((not self.robot.co_svc.disableCollisionDetection()) is True)
        self.robot.seq_svc.setJointAngles(self.col_safe_pose, 1.0);
        self.robot.waitInterpolation();

    def test_CollisionLoopChange(self):
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--maxloop', action='store', type=int, nargs='?', default=3)
        args,unknown=parser.parse_known_args()
        if not self.use_set_collision_loop :
            args.maxloop=1
        for i in range(1, args.maxloop+1):
            self.curr_loop = i
            if self.use_set_collision_loop:
                self.robot.co_svc.setCollisionLoop(self.curr_loop)
            self.demoCollisionCheckSafe()
            self.demoCollisionCheckFail()
            self.demoCollisionCheckFailWithSetTolerance()
            self.demoCollisionDisableEnable()

        self.fout.close()

if __name__ == '__main__':
    import rostest
    print("===================================================")
    print("# Please consult test result with following process")
    print("tail -f /tmp/check-test_hironx_collision.txt")
    print("# You can run individual test with following command")
    print("python -m unittest test_hironx_collision.TestHiroCollision.test_CollisionLoopChange")
    rostest.rosrun(PKG, 'test_hronx_collision', TestHiroCollision)
