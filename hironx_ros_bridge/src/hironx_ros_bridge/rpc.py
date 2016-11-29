# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, TORK
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
#  * Neither the name of TORK. nor the
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

import actionlib
from hironx_ros_bridge.hironx_client import HIRONX
import hironx_ros_bridge.msg as hironxoaction
from hrpsys import rtm
import rospy


class HironxRPC(object):
    '''
    RPC (Remote Procedure Call) for methods in HIRONX class.
    '''

    def __init__(self, host='', port=15005, robot="RobotHardware0", modelfile=''):
        '''
        @param args: TODO
        '''
        rospy.init_node('hironx_rpc')
        self.robot = HIRONX()
        if host:
            rtm.nshost = host if host else 'localhost'
        if port:
            rtm.nsport = port
        if not robot:
            robot = "RobotHardware0" if host else "HiroNX(Robot)0"
        if not modelfile:
            modelfile = "/opt/jsk/etc/HIRONX/model/main.wrl" if host else ""
        self.robot.init(robotname=robot, url=modelfile)

        # Initialize action clients
        self._aclient_goInitial = actionlib.SimpleActionServer('goInitial', hironxoaction.GoInitialAction, execute_cb=self._cb_goInitial, auto_start=False)
        self._aclient_goInitial.start()

    def _cb_goInitial(self, goal):
        _result   = hironxoaction.GoInitialResult()
        # check that preempt has not been requested by the client
        if self._aclient_goInitial.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._aclient_goInitial.set_preempted()
        _res_from_remote = self.robot.goInitial(goal.tm, goal.wait, goal.init_pose_type)

        if _res_from_remote:
            _result.res = _res_from_remote
            self._aclient_goInitial.set_succeeded(_result)
