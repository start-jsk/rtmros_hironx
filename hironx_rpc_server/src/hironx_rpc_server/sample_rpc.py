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
    OpenHRP_SequencePlayerService_setJointAngle,
    OpenHRP_SequencePlayerService_setJointAngles,
    OpenHRP_SequencePlayerService_setJointAnglesOfGroup,
    OpenHRP_SequencePlayerService_setTargetPose,
    OpenHRP_SequencePlayerService_waitInterpolation,
    OpenHRP_SequencePlayerService_waitInterpolationOfGroup,
)
import rospy

from hironx_rpc_msgs.srv import (
    CalibrationOperation, GetCartesianCommon, GetJointAngles,
    GetKinematicsGroups, GetRTCList, GetSensors, GoInitOffPoses, LoadPattern,
    RobotState, ServoOperation, SetEffort, SetHandJointAngles
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
        pass

    def sample_checkEncoders(self):
        _srv_name = 'srv_checkEncoders'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, CalibrationOperation)
            _response = _srv_proxy(method_type_id=1)
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

    def sample_getCurrentPosition(self, linkname='LARM_JOINT5'):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getCurrentPosition'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(1, linkname, framename)
            res = (_response.vec3.x, _response.vec3.y, _response.vec3.z)
            return res
        except rospy.ServiceException, e:
            raise e

    def sample_getCurrentRotation(self, linkname='LARM_JOINT5'):
        '''
        @rtype tork_rpc_util/Float32List[]
        '''
        #TODO: We may want to extract internal logic and reuse.
        _srv_name = 'srv_getCurrentRotation'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(2, linkname, framename)
            res_list = []
            for row in _response.rot:
                row_list = []
                for e in row.data:
                    row_list.append(e)
                res_list.append(row_list)
            return res_list
        except rospy.ServiceException, e:
            raise e

    def sample_getCurrentRPY(self, linkname='LARM_JOINT5'):
        '''
        @rtype (float, float, float)
        '''
        _srv_name = 'srv_getCurrentRPY'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(3, linkname, framename)
            res = (_response.vec3.x, _response.vec3.y, _response.vec3.z)
            return res
        except rospy.ServiceException, e:
            raise e

    def sample_getJointAngles(self):
        '''
        @note: Since srv is defined but no service is run by hrpsys_ros_bridge,
               use service defined in hironx_rpc_server.
        @rtype [float]
        @return list of joint angles.
        '''
        _srv_name = 'srv_getJointAngles'

        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetJointAngles)
            _response = _srv_proxy()
            return _response.angles
        except rospy.ServiceException, e:
            raise e

    def sample_groups(self):
        '''
        @summary: From the service receive hironx_rpc_msgs.GetKinematicsGroups
                  then convert back to the Hironx.groups format to return. 
        @rtype [[str, [str]]]
        @see For return type http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a13732eef4afc0e25e587f3858bb568c6
        '''
        _srv_name = 'srv_groups'
        rospy.wait_for_service(_srv_name)
        groups_list = []
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetKinematicsGroups)
            _response = _srv_proxy()
            for g in _response.groups:
                groups_list.append([g.groupname, g.joints])
            return groups_list
        except rospy.ServiceException, e:
            raise e

    def sample_getReferencePosition(self, linkname='LARM_JOINT5'):
        '''
        @rtype geometry_msgs/Vector3
        '''
        _srv_name = 'srv_getReferencePosition'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(4, linkname, framename)
            res = (_response.vec3.x, _response.vec3.y, _response.vec3.z)
            return res
        except rospy.ServiceException, e:
            raise e

    def sample_getReferenceRotation(self, linkname='LARM_JOINT5'):
        '''
        @summary: API doc:
            Definition: robot.getReferenceRotation(self, lname, frame_name=None)
            Docstring:
            !@brief
            Returns the current commanded rotation of the specified joint.
            cf. getCurrentRotation that returns physical value.

            @type lname: str
            @param lname: Name of the link.
            @param frame_name str: set reference frame name (from 315.2.5)
            @rtype: list of float
            @return: Rotational matrix of the given joint in 2-dimensional list,
                     that is:

            erbatim
                     [[a11, a12, a13],
                      [a21, a22, a23],
                      [a31, a32, a33]]
            \endverbatim

        @rtype [[float]]
        '''
        _srv_name = 'srv_getReferenceRotation'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(5, linkname, framename)
            res_list = []
            for row in _response.rot:
                row_list = []
                for e in row.data:
                    row_list.append(e)
                res_list.append(row_list)
            return res_list
        except rospy.ServiceException, e:
            raise e

    def sample_getReferenceRPY(self, linkname='LARM_JOINT5'):
        '''
        @rtype (float, float, float) 
        '''
        _srv_name = 'srv_getReferenceRPY'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetCartesianCommon)
            framename = None
            _response = _srv_proxy(6, linkname, framename)
            res = (_response.vec3.x, _response.vec3.y, _response.vec3.z) 
            return res
        except rospy.ServiceException, e:
            raise e

    def sample_getRTCList(self):
        '''
        @summary: The output from HrpsysConfigurator.py looks like:

                robot.getRTCList()
                Out[2]:
                [['seq', 'SequencePlayer'],
                 ['sh', 'StateHolder'],
                 ['fk', 'ForwardKinematics'],
                 ['ic', 'ImpedanceController'],
                 ['el', 'SoftErrorLimiter'],
                 ['sc', 'ServoController'],
                 ['log', 'DataLogger'],
                 ['rmfo', 'RemoveForceSensorLinkOffset']]

            However, because of the limitation in ROS messaging, which doesn't
            support multi-dimensional array, it is impossible to return a list
            like above. This method instead returns a list where each list
            element is converted into string, like the following:

                 sample_rpc.sample_getRTCList()
                 Out[3]:
                 ["'seq', 'SequencePlayer'",
                  "'sh', 'StateHolder'",
                  "'fk', 'ForwardKinematics'",
                  "'ic', 'ImpedanceController'",
                  "'el', 'SoftErrorLimiter'",
                  "'sc', 'ServoController'",
                  "'log', 'DataLogger'",
                  "'rmfo', 'RemoveForceSensorLinkOffset'"]

        @see http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a5614e5504ba16881c14eaf3ea61adc81
        @return: List of string. Unlike the example above that's the return
                 value from HrpsysConfigurator.py, each element is converted
                 as a string.
        @rtype [str]
        '''
        _srv_name = 'srv_getRTCList'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetRTCList)
            _response = _srv_proxy()
            return _response.rtcs
        except rospy.ServiceException, e:
            raise e

    def sample_getRTCInstanceList(self):
        '''
        @see http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a888185b5cff537487c060f77f2e8ef26
        @return: List of the instance name. Example:

            robot.getRTCInstanceList()
            Out[50]:
            [<hrpsys.rtm.RTcomponent instance at 0x7fd554ec7488>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd555b16c20>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd5548f0d40>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd554f3fb48>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd554950b00>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd55494c5f0>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd5549635f0>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd554eb4128>,
             <hrpsys.rtm.RTcomponent instance at 0x7fd55496c908>]
        @rtype [str]
        '''
        _srv_name = 'srv_getRTCInstanceList'
        rospy.wait_for_service(_srv_name)
        try:
            # Use the same srv type as getRTCList
            _srv_proxy = rospy.ServiceProxy(_srv_name, GetRTCList)
            _response = _srv_proxy()
            return _response.rtcs
        except rospy.ServiceException, e:
            raise e

    def sample_getReferencePose(self, linkname='LARM_JOINT5'):
        '''
        @summary: No API doc available online for the ROS service used for
                  this. .srv file content:

                    $ more /opt/ros/indigo/share/hrpsys_ros_bridge/srv/OpenHRP_ForwardKinematicsService_getReferencePose.srv
                    string linkname
                    ---
                    bool operation_return
                    RTC_TimedDoubleSeq pose
        @rtype 
        '''
        _srv_name = 'ForwardKinematicsServiceROSBridge/getReferencePose'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_ForwardKinematicsService_getReferencePose)
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

    def sample_isServoOn(self):
        '''
        @see: http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#afe037d52b45f48775224c39511e1c038
        @rtype int
        @return: 1 when true. -1 otherwise.
        '''
        _srv_name = 'srv_isServoOn'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, ServoOperation)
            _response = _srv_proxy(method_type_id=3, joint_name='all')
            return _response.result_on
        except rospy.ServiceException, e:
            raise e

    def sample_isCalibDone(self):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a955c6b358765ef15591da00d4c16f43b
        @rtype bool
        '''
        _srv_name = 'srv_isCalibDone'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, CalibrationOperation)
            _response = _srv_proxy(method_type_id=2)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_loadPattern(self, pattern_filepath, tm=10):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#aaea0081b80df5d4a994c13723e7498df
        @return -1 if error happened internally.
        '''
        if not pattern_filepath:
            rospy.logerr('pattern_filepath is not set. Returning the method.')
            return -1
        _srv_name = 'srv_loadPattern'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, LoadPattern)
            _response = _srv_proxy(file_path=pattern_filepath, tm=tm)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_set_effort(self):
        '''
        @summary: setHandEffort can be called from here.
        @see http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a9412965040afbd04c50bd1ea5d9bc501
        @return: Nothing returned.
        '''
        _srv_name = 'srv_setEffort'
        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, SetEffort)
            _response = _srv_proxy(effort=50)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_setHandJointAngles(
            self, srv_name='srv_setHandJointAngles', hand='lhand', av=[0.0, 0.0, 0,0, 0.0], tm=2):
        '''
        @see http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#ad4945bdcd78468c77888ee2506227cfe
        @rtype bool
        @return: TODO Because the upstream method setHandJointAngles returns
                 nothing on the simulation, for now this service always return
                 True. Once the correct behavior figured with the real robot,
                 this should also be fixed. 
        '''
        _srv_name = srv_name
        method_type_id = 1
        if _srv_name == 'srv_moveHand':
            method_type_id = 2

        rospy.wait_for_service(_srv_name)
        try:
            _srv_proxy = rospy.ServiceProxy(_srv_name, SetHandJointAngles)
            _response = _srv_proxy(
                method_type_id=method_type_id, handgroup_name='lhand',
                angles=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), time=2)
            return _response
        except rospy.ServiceException, e:
            raise e

    def sample_moveHand(self):
        '''
        @see http://docs.ros.org/indigo/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#ad44dd1e89b9662fab181638630d4e706
        '''
        self.sample_setHandJointAngles(srv_name='srv_moveHand')

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

    def sample_setTargetPose(self):
        '''
        @see: http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html#a918664913b98e332ef8a6a23f5da379c
        @rtype OpenHRP_SequencePlayerService_setTargetPose
        '''
        _srv_name = '/SequencePlayerServiceROSBridge/setTargetPose'
        rospy.wait_for_service(_srv_name)
        name = 'rarm'
        rospy.loginfo('Assuming this is run at the initial pose. If not, '
                      'run goInitial first.')
        xyz = [0.3255627368715471, -0.1823638733778268, 0.07462449717662004]
        rpy = (3.0732189053889805, -1.5690225912054285, -3.0730289207320203)
        tm = 3
        try:
            _srv_proxy = rospy.ServiceProxy(
                _srv_name, OpenHRP_SequencePlayerService_setTargetPose)
            _response = _srv_proxy(name, xyz, rpy, tm)
            return _response
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
