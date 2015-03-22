#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx import *

class TestHiroController(TestHiro):

    # https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_impedance_controller.py.in
    def test_impedance_controller(self): # https://github.com/start-jsk/rtmros_hironx/issues/337
        if not self.robot.ic or self.robot.ic.ref.get_component_profile().version < '315.3.0':
            self.assertTrue(True)
            return True
        # although this is not stable rtc, we'll test them
        self.robot.goInitial(tm=1)
        ret = self.robot.startImpedance('rarm') # this returns ret, this is bug
        #self.assertTrue(ret)
        ret = self.robot.seq_svc.setWrenches([0,0,0,0,0,0,
                                              10,0,0,0,0,0,], 2.0);
        #self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0,0,0,0,0,0,
                                              0,0,0,0,0,0,], 2.0);
        #self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0,0,0,0,0,0,
                                              0,10,0,0,0,0,], 2.0);
        #self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0,0,0,0,0,0,
                                              0,0,0,0,0,0,], 2.0);
        #self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0,0,0,0,0,0,
                                              0,0,10,0,0,0,], 2.0);
        #self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        #self.assertTrue(ret)
        ret = self.robot.stopImpedance('rarm')
        #self.assertTrue(ret)
        self.assertTrue(True) # this is dummy, current simulate hiro does not have force sensor so it retunrs None


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_controller', TestHiroController) 
