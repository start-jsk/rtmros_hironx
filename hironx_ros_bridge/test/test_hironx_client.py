#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TORK
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of TORK (Tokyo Opensource Robotics Kyokai Association). 
# nor the names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
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

from hironx_ros_bridge.hironx_client import HIRONX
from test_hironx import TestHiro

PKG = 'hironx_ros_bridge'


class TestHiroClient(TestHiro):

    _RTC_LIST_CUSTOM = [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['rmfo', 'RemoveForceSensorLinkOffset']
        ]

    def _compare_2dlist(self, twodim_list_a, twodim_list_b):
        '''
        Compare the first element in all elements of the 2nd list.
        E.g. For [['a0', 'a1'], ['b0', 'b1'],..., ['n0', 'n1']], this method
             checks a0, b0, n0
        @rtype bool
        '''
        return set([a[0] for a in twodim_list_a]) == set(
            [b[0] for b in twodim_list_b])

    def test_getRTCList(self):
        '''
        Depending on the hrpsys version, different RTC implementation can be
        returned, e.g. for "rmfo", older returns AbsoluteForceSensor while
        newer does RemoveForceSensorLinkOffset. So in this testcase we only
        check the first element of the returned list (e.g. "rmfo" instead of
        its implementation).
        '''
        self.assertTrue(
            self._compare_2dlist(
                self.robot.getRTCList(),
                # Accessing a private member var only for testing purpose. 
                self.robot._RTC_list))

    def test_getRTCList_customrtcs_args_correct(self):
        '''
        Test when the RTC list was passed from the client.

        Because this uses HIRONX.init(), which is already done in the
        superclass, HIRONX class instance is re-generated within this method,
        which is not elegant but as of now I can't think of a better way.
        '''
        self.robot = HIRONX()
        # Passing 1st elems from _RTC_LIST_CUSTOM, to init method that calls
        # internally getRTCList.
        self.robot.init(rtcs='seq, sh, fk')
        self.assertTrue(
            self._compare_2dlist(
                self.robot.getRTCList(), self._RTC_LIST_CUSTOM))

    def test_getRTCList_customrtcs_args_wrong(self):
        '''
        Test when the RTC list was passed from the client, in wrong format.
        '''
        # Passing the list of RTCs falling short of requirement.
        self.assertRaises(
            ValueError, self.robot.getRTCList, rtcs_str='seq, sh')

        # Passing 1st elems from _RTC_LIST_CUSTOM,
        # but list is not the right type of arg.
        ## http://stackoverflow.com/a/6103930/577001
        self.assertRaises(
            TypeError, lambda: self.robot.getRTCList,
                rtcs_str=['seq', 'sh', 'fk', 'el', 'co', 'log'])

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_client', TestHiroClient)
