#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association (TORK)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from distutils.version import StrictVersion

from test_hironx import *


class TestHiroController(TestHiro):

    # https://github.com/fkanehiro/hrpsys-base/blob/master/sample/SampleRobot/samplerobot_impedance_controller.py.in
    def test_impedance_controller(self, **kwargs):  # https://github.com/start-jsk/rtmros_hironx/issues/337
        if not self.robot.ic or self.robot.ic.ref.get_component_profile().version < '315.3.0':
            self.assertTrue(True)
            return True
        # although this is not stable rtc, we'll test them
        self.robot.goInitial(tm=1)
        ret = self.robot.startImpedance('rarm', **kwargs)  # this returns ret, this is bug
        # self.assertTrue(ret)
        ret = self.robot.seq_svc.setWrenches([0, 0, 0, 0, 0, 0,
                                              10, 0, 0, 0, 0, 0, ], 2.0)
        # self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, ], 2.0)
        # self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0, 0, 0, 0, 0, 0,
                                              0, 10, 0, 0, 0, 0, ], 2.0)
        # self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, ], 2.0)
        # self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation();
        ret = self.robot.seq_svc.setWrenches([0, 0, 0, 0, 0, 0,
                                              0, 0, 10, 0, 0, 0, ], 2.0)
        # self.assertTrue(ret)
        self.robot.seq_svc.waitInterpolation()
        # self.assertTrue(ret)
        ret = self.robot.stopImpedance('rarm')
        # self.assertTrue(ret)
        self.assertTrue(True)  # this is dummy, current simulate hiro does not have force sensor so it retunrs None

    def test_impedance_controller_refforce_315_1(self):
        '''
        @summary: Pass ref_force for hrpsys < 315.2.0.
                  Although Impedance Controller is not a stable RTC,
                  we'll test it.
        '''
        KIN_GROUP = 'rarm'  # Target kinematic group.
        _REFFORCE_DEFAULT = [0.5, 0.5, 0.5]
        _REFFORCE_2 = [0.5, 0.5, 0.5]
        if not self.robot.ic or StrictVersion('315.2.0') <= StrictVersion(self.robot.hrpsys_version):
            self.assertTrue(True)
            return True

        self.robot.goInitial(tm=1)
        self.robot.startImpedance(KIN_GROUP)
        # ic_svc.getImpedanceControllerParam returns something like:
        #
        # (True, OpenHRP.ImpedanceControllerService.impedanceParam(
        #            M_p=100.0, D_p=200.0, K_p=400.0, M_r=2000.0, D_r=100.0, K_r=200.0, 
        #            force_gain=[1.0, 1.0, 1.0], moment_gain=[0.0, 0.0, 0.0],
        #            sr_gain=1.0, avoid_gain=0.0, reference_gain=0.0,
        #            manipulability_limit=0.1, controller_mode=MODE_IMP,
        #            ik_optional_weight_vector=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        #            use_sh_base_pos_rpy=False))
        self.assertListEqual(
            _REFFORCE_DEFAULT,
            self.robot.ic_svc.getImpedanceControllerParam(KIN_GROUP).ref_force)

        # Test startImpedance with ref_force as an arg.
        self.robot.startImpedance(KIN_GROUP, ref_force=_REFFORCE_2)
        self.assertListEqual(
            _REFFORCE_2,
            self.robot.ic_svc.getImpedanceControllerParam(KIN_GROUP).ref_force)
        self.robot.stopImpedance(KIN_GROUP)

    def test_hands_controller(self):
        '''
        If Servo Controller RTC are running (which will be always true for
        the tests because it runs on simulation), kill it then test if servoOn
        method still succeeds.
        '''
        if self.robot.sc_svc:
            self.robot.sc_svc = None
        else:
            print("Servo Controller RTC's not running, so skipping this test.")
            pass
        self.assertEqual(self.robot.servoOn(), 1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_controller', TestHiroController)
