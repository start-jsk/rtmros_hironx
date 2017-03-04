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
    OpenHRP_ServoControllerService_getJointAngles,
    OpenHRP_SequencePlayerService_setJointAngle,
    OpenHRP_SequencePlayerService_setJointAngles,
    OpenHRP_SequencePlayerService_setJointAnglesOfGroup,
    OpenHRP_SequencePlayerService_waitInterpolation,
    OpenHRP_SequencePlayerService_waitInterpolationOfGroup
)
import rospy

from hironx_rpc_msgs.srv import (
    CheckEncoders, GetCartesianCommon, GetSensors, GoInitOffPoses, RobotState,
    ServoOperation
)


class SampleClientHironxRPC(object):
    '''
    RPC sample methods for Hironx. Intended to be called from main method
    within this same Python file.
    '''
    JOINTANGLES_LEFTARM_INIT_FACTORY = [-7.951386703658792e-16, 0.0, -130.0, 2.5444437451708134e-14, 0.0, 0.0]
    JOINTANGLES_WHOLE_INIT_FACTORY = [0.0, 0.0, 0.0, -6.361109362927034e-14,
                                7.951386703658792e-16, 0.0, -130.0, -2.5444437451708134e-14, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 
                                JOINTANGLES_LEFTARM_INIT_FACTORY,
                                0.0, 0.0, 0.0, 0.0]
    JOINTANGLES_WHOLE_HEADDOWN = [0.0, 0.0, 0.0, 9.99999999999999, -0.600001403060998,
                          0.0, -100.00004285756798, 15.19999734702561,
                          9.4000028826958, 3.2000265815851607, 0.0, 0.0, 0.0,
                          0.0, 0.600001403060998, 0.0, -100.00004285756798,
                          -15.19999734702561, 9.4000028826958, -3.2000265815851607,
                          0.0, 0.0, 0.0, 0.0]  # 10 deg down after init pose.

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

    def sample_getActualState(self):
        '''
        @summary: Out put of HrpsysConfigurator.getActualState() contains a few
                  multi-dimensional lists that are namely servoState and force,
                  which is not well supported in ROS messaging architecture
                  as of March 2017. tork_rpc_util/{Int8List, Float32List} are
                  introduced in order to work this around. This results in the
                  format of the output value of the service
                  "srv_getActualState" slightly differs from that of
                  getActualState method.
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a6f23370709d801b8165e157e1de5bb89
        @rtype: TBD
        '''
        _srv_name = 'srv_getActualState'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, RobotState)
            _response = _srv_proxy()
            _response.servoState
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

    def sample_getCurrentPosition(self):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getCurrentPosition'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(1, linkname, framename)
            return _response.vec3
        except rospy.ServiceException, e:
            raise e

    def sample_getCurrentRotation(self):
        '''
        @rtype tork_rpc_util/Float32List[]
        '''
        #TODO: We may want to extract internal logic and reuse.
        _srv_name = 'srv_getCurrentRotation'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(2, linkname, framename)
            return _response.rot
        except rospy.ServiceException, e:
            raise e

    def sample_getCurrentRPY(self):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getCurrentRPY'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(3, linkname, framename)
            return _response.vec3
        except rospy.ServiceException, e:
            raise e

    def sample_getReferencePosition(self):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getReferencePosition'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(4, linkname, framename)
            return _response.vec3
        except rospy.ServiceException, e:
            raise e

    def sample_getReferenceRotation(self):
        '''
        @rtype tork_rpc_util/Float32List[]
        '''
        _srv_name = 'srv_getReferenceRotation'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(5, linkname, framename)
            return _response.rot
        except rospy.ServiceException, e:
            raise e

    def sample_getReferenceRPY(self):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getReferenceRPY'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            linkname = 'LARM_JOINT5'
            framename = None
            _response = _srv_proxy(6, linkname, framename)
            return _response.vec3
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

    def sample_getSensors(self):
        '''
        @see http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a5d85768a0fb0e98c61970f33952d49a7
        '''
        _srv_name = 'srv_getSensors'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, GetSensors)
            # The file_url value hardcoded here is what you get by running
            # the following command:
            #     rospack find hironx_ros_bridge`/models/kawada-hironx.dae
            file_url = '/opt/ros/indigo/share/hironx_ros_bridge/models/kawada-hironx.dae'
            _response = _srv_proxy(modelfile_url=file_url)
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

    def sample_goInitial(self, tm=7, wait=True, init_pose_type=0):
        '''
        @see: http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a295b0b4950cb580273b224dc659c8a23
        @rtype: bool
        @return: If the service call returns, true.
        '''
        _srv_name = 'srv_goInitial'  # Must be the same name defined in ActionServiceNameDict.
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GoInitOffPoses)
            _response = _srv_proxy(method_type_id=1, time=tm, wait=wait,
                                   init_pose_type=init_pose_type)
            return _response.success
        except rospy.ServiceException, e:
            raise e

    def sample_goOffPose(self):
        '''
        @see: http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#ae1a0cefa2121d5ba4071e4f230936400
        '''
        _srv_name = 'srv_goOffPose'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GoInitOffPoses)
            _response = _srv_proxy(method_type_id=2, time=7)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_servoOff(self):
        '''
        @see: fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a31eadc49bdf2ce66014c510500614c07
        @rtype: bool
        @return: If the service call returns, true.
        '''
        _srv_name = 'srv_servoOff'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, ServoOperation)
            _response = _srv_proxy(method_type_id=2, joint_name='all',
                                   wait=True)
            return _response.result_off
        except rospy.ServiceException, e:
            raise e

    def sample_servoOn(self):
        '''
        @see: fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a31eadc49bdf2ce66014c510500614c07
        @rtype: bool
        @return: If the service call returns, true.
        '''
        _srv_name = 'srv_servoOn'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, ServoOperation)
            _response = _srv_proxy(method_type_id=1, joint_name='all', time=4)
            return _response.result_on
        except rospy.ServiceException, e:
            raise e

    def sample_waitInerpolation(self):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#aa0c2ce9825c19956c7647208bf193309
        @return: None.
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/waitInterpolation'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_SequencePlayerService_waitInterpolation)
            _response = _srv_proxy()
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_waitInerpolationOfGroups(self, groupname='larm'):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a8613814b3a8152647b54ae5f28e35dd9
        @param groupname str: Name of the joint group.
        @return: None.
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/waitInterpolationOfGroup'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name,
                OpenHRP_SequencePlayerService_waitInterpolationOfGroup)
            _response = _srv_proxy(groupname)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_setJointAngle(self, jname='HEAD_JOINT1', angle=10, tm=2):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#afa4d06b8c1f38b6128a3b59836d5c067
        @param jname: Default HEAD_JOINT1 is pitch axis.
        @rtype OpenHRP_SequencePlayerService_setJointAngleResponse
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/setJointAngle'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_SequencePlayerService_setJointAngle)
            _response = _srv_proxy(jname, angle, tm)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_setJointAngles(self, angles=JOINTANGLES_WHOLE_HEADDOWN, tm=2):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a8f6c980d05cc00f41d9667036ebd436f
        @rtype OpenHRP_SequencePlayerService_setJointAnglesResponse
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/setJointAngles'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_SequencePlayerService_setJointAngles)
            _response = _srv_proxy(angles, tm)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_setJointAnglesOfGroup(self, groupname='larm',
                                     jvs=JOINTANGLES_LEFTARM_INIT_FACTORY, tm=2):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a31eadc49bdf2ce66014c510500614c07
        @rtype OpenHRP_SequencePlayerService_setJointAnglesOfGroupResponse
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/setJointAnglesOfGroup'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_SequencePlayerService_setJointAnglesOfGroup)
            _response = _srv_proxy(groupname, jvs, tm)
            return _response
        except rospy.ServiceException, e:
            raise e
