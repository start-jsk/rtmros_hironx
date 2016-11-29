#!/usr/bin/env python
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
import hironx_ros_bridge.msg as hironxoaction
from hironx_ros_bridge.rpc import HironxRPC


class SampleHironxRPC(object):
    '''
    RPC sample methods for Hironx. Intended to be called from main method
    within this same python file.
    '''

    def __init__(self, host='', port=15005, robot="RobotHardware0", modelfile=''):
        '''
        Overriding because we need to instantiate
        hironx_ros_bridge.rpc.HironxRPC class.
        '''
        # Start an action server that handles various ROS Actions.
        self._action_server = HironxRPC(host, port, robot, modelfile)

    def sample_goInitial(self, tm=7, wait=True, init_pose_type=0):
        '''
        Args are taken from hironx__client.HIRONX class. See its api doc for
        the detail http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html
        '''
        aclient = actionlib.SimpleActionClient('goInitial', hironxoaction.GoInitialAction)
        aclient.wait_for_server()
        goal = hironxoaction.GoInitialGoal(tm, wait, init_pose_type)
        aclient.send_goal(goal)
        aclient.wait_for_result()
        # TODO return value
