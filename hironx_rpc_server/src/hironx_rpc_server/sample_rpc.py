#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 TORK (Tokyo Opensource Robotics Kyokai Association)
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

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from hrpsys_ros_bridge.srv import (
    OpenHRP_ForwardKinematicsService_getCurrentPose,
    OpenHRP_ForwardKinematicsService_getReferencePose,
    OpenHRP_StateHolderService_goActual,
    OpenHRP_ServoControllerService_getJointAngles
)
import rospy

from hironx_rpc_msgs.srv import (
    CheckEncoders, GoInitial
)


class SampleClientHironxRPC(object):
    '''
    RPC sample methods for Hironx. Intended to be called from main method
    within this same Python file.
    '''

    def __init__(self):
        '''        '''
        # Start an action server that handles various ROS Actions.
        rospy.init_node('hironx_rpc_sample')

    def sample_checkEncoders(self):
        _srv_name = 'srv_checkEncoders'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, CheckEncoders)
            _response = _srv_proxy()
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_getCurrentPose(self):
        '''
        '''
        _srv_name = 'ForwardKinematicsServiceROSBridge/getCurrentPose'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_ForwardKinematicsService_getCurrentPose)
            linkname = 'LARM_JOINT5'
            _response = _srv_proxy(linkname)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_getJointAngles(self):
        '''
        @note: TODO: Not working. Srv is defined but no service is run by
               hrpsys_ros_bridge.
        '''
        _srv_name = 'ServoControllerServiceROSBridge/getJointAngles'

        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_ServoControllerService_getJointAngles)
            _response = _srv_proxy()
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_getReferencePose(self):
        _srv_name = 'ForwardKinematicsServiceROSBridge/getReferencePose'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_ForwardKinematicsService_getReferencePose)
            linkname = 'LARM_JOINT5'
            _response = _srv_proxy(linkname)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_goActual(self):
        # This must be either
        # - the same name defined in ActionServiceNameDict.
        # - existing service launched somewhere else.
        _srv_name = 'StateHolderServiceROSBridge/goActual'

        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_StateHolderService_goActual)
            _response = _srv_proxy()
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_goInitial(self, tm = 7, wait = True, init_pose_type = 0):
        '''
        @see: http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a295b0b4950cb580273b224dc659c8a23
        @rtype: bool
        @return: If the service call returns, true.
        '''
        _srv_name = 'srv_goInitial'  # Must be the same name defined in ActionServiceNameDict.
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GoInitial)
            _response = _srv_proxy(tm, wait, init_pose_type)
            return _response.success
        except rospy.ServiceException, e:
            raise e
