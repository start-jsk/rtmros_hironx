# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, TORK (Tokyo Opensource Robotics Kyokai Association)
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

import abc


class AbstAcceptanceTest():

    _MSG_ERR_NOTIMPLEMENTED = 'The method is not implemented in the derived class'

    GRNAME_LEFT_ARM = 'larm'
    GRNAME_RIGHT_ARM = 'rarm'
    GRNAME_TORSO = 'torso'
    GRNAME_HEAD = 'head'

    def __init__(self, robot_client, default_task_duration=7.0):
        '''
        @type robot_client: hironx_ros_bridge.ros_client.ROS_Client or
                            hrpsys.hrpsys_config.HrpsysConfigurator
        '''
        self._robotclient = robot_client
        self._default_task_duration = default_task_duration

    @abc.abstractmethod
    def go_initpos(self, default_task_duration=7.0):
        raise NotImplementedError(self._MSG_ERR_NOTIMPLEMENTED)

    @abc.abstractmethod
    def set_joint_angles(self, joint_group, joint_angles,
                  msg_tasktitle=None, task_duration=7.0, do_wait=True):
        '''
        Move by passing joint angles of an arm.

        @type joint_group: str
        @type joint_angles: [double]
        @type msg_tasktitle: str
        @type task_duration: double
        '''
        raise NotImplementedError(self._MSG_ERR_NOTIMPLEMENTED)

    @abc.abstractmethod
    def set_pose(self, joint_group, pose, rpy, msg_tasktitle=None,
                 task_duration=7.0, do_wait=True, ref_frame_name=None):
        raise NotImplementedError(self._MSG_ERR_NOTIMPLEMENTED)

    @abc.abstractmethod
    def set_pose_relative(
                        self, joint_group, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0,
                        msg_tasktitle=None, task_duration=7.0, do_wait=True):
        '''        '''
        raise NotImplementedError(self._MSG_ERR_NOTIMPLEMENTED)
