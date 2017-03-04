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

from geometry_msgs.msg import Pose, Quaternion, Vector3
from hironx_ros_bridge import hironx_client
from hrpsys import rtm
import rospy
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
import sys

from hironx_rpc_msgs.srv import (
    CheckEncoders, CheckEncodersResponse,
    GetCartesianCommon, GetCartesianCommonResponse,
    GetSensors, GetSensorsResponse,
    GoInitOffPoses, GoInitOffPosesResponse,
    RobotState, RobotStateResponse,
    ServoOperation, ServoOperationResponse
)
from tork_rpc_util.rpc_servers_handler import ActionServiceInfo, RpcServersHandler
from tork_rpc_util.msg import Float32List, Int8List


class ActionServiceNameDict(object):
    '''
    Static, entity class to hold name for ROS Actions for the robot.
    '''
    checkEncoders = 'srv_checkEncoders'
    getActualState = 'srv_getActualState'
    getCurrentPosition = 'srv_getCurrentPosition'
    getCurrentRotation = 'srv_getCurrentRotation'
    getCurrentRPY = 'srv_getCurrentRPY'
    getReferencePosition = 'srv_getReferencePosition'
    getReferenceRotation = 'srv_getReferenceRotation'
    getReferenceRPY = 'srv_getReferenceRPY'
    getSensors = 'srv_getSensors'
    goInitial = 'srv_goInitial'
    goOffPose = 'srv_goOffPose'
    servoOff = 'srv_servoOff'
    servoOn = 'srv_servoOn'


class HironxRpcServer(RpcServersHandler):
    '''
    This class works as a server of RPC client nodes.

    Don't get confused that this is still a "client" w.r.t the robot's
    controller, same as hironx_ros_bridge.{Hironx, ROS_Client} classes --
    This class is called a "server" only in RPC context.

    @see: also the Hironx' RPC-specific diagram: https://docs.google.com/drawings/d/1UNUasiORaGl-QQQEIa4Pex165YAyxgprAq9F5XbaSJA/edit
    '''

    def __init__(self):
        ''' '''
        rospy.init_node('hironx_rpc_server')

        self._robotif = self._init_hironx_rtmclient()

        self.action_infos = {
            ActionServiceNameDict.checkEncoders: ActionServiceInfo(ActionServiceNameDict.checkEncoders, CheckEncoders, self._cb_checkEncoders),
            ActionServiceNameDict.getActualState: ActionServiceInfo(ActionServiceNameDict.getActualState, RobotState, self._cb_getActualState),
            ActionServiceNameDict.getCurrentPosition: ActionServiceInfo(ActionServiceNameDict.getCurrentPosition, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getCurrentRotation: ActionServiceInfo(ActionServiceNameDict.getCurrentRotation, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getCurrentRPY: ActionServiceInfo(ActionServiceNameDict.getCurrentRPY, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getReferencePosition: ActionServiceInfo(ActionServiceNameDict.getReferencePosition, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getReferenceRotation: ActionServiceInfo(ActionServiceNameDict.getReferenceRotation, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getReferenceRPY: ActionServiceInfo(ActionServiceNameDict.getReferenceRPY, GetCartesianCommon, self._cb_getCartesian),
            ActionServiceNameDict.getSensors: ActionServiceInfo(ActionServiceNameDict.getSensors, GetSensors, self._cb_getSensors),
            ActionServiceNameDict.goInitial: ActionServiceInfo(ActionServiceNameDict.goInitial, GoInitOffPoses, self._cb_goInitOffPoses),
            ActionServiceNameDict.goOffPose: ActionServiceInfo(ActionServiceNameDict.goOffPose, GoInitOffPoses, self._cb_goInitOffPoses),
            ActionServiceNameDict.servoOff: ActionServiceInfo(ActionServiceNameDict.servoOff, ServoOperation, self._cb_servoOperation),
            ActionServiceNameDict.servoOn: ActionServiceInfo(ActionServiceNameDict.servoOn, ServoOperation, self._cb_servoOperation)
        }
        super(HironxRpcServer, self).__init__(self.action_infos)

        rospy.loginfo(sys._getframe().f_code.co_name + '__init__ done.')

    def _cb_checkEncoders(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.CheckEncoders
        '''
        jname = service_req.jname
        option = service_req.option
        rospy.loginfo('Service requested {} '.format(service_req))
        ret = self._robotif.checkEncoders(jname=jname, option=option)
        return CheckEncodersResponse(ret)

    def _cb_getActualState(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.RobotState
        '''
        rospy.loginfo('Service requested {} '.format(service_req))
        ret = self._robotif.getActualState()

        service_response = RobotStateResponse(
            angle=ret.angle, command=ret.command, torque=ret.torque,
            rateGyro=ret.rateGyro, accel=ret.accel, voltage=ret.voltage,
            current=ret.current)
        list_servostate = []
        for state_each_servo in ret.servoState:
            for each_servovalue in state_each_servo:
                s = Int8List([each_servovalue])
                list_servostate.append(s)
        service_response.servoState = list_servostate

        list_forces = []
        forces_per_eef = None
        for forces_each_eef in ret.force:
            forces_per_eef = []  # Initialize
            for each_force in forces_each_eef:
                forces_per_eef.append(each_force)
            eef = Float32List()
            eef.data = forces_per_eef
            list_forces.append(eef)
        service_response.force = list_forces
        # servoState HrpsysConfigurator.getActualState returns comes in
        # 2-dimensional list.
        # Filling data for servoState multi array.
        rospy.logdebug('ret.servoState={}\nlist_servo: {}\n'.format(
            ret.servoState, service_response.servoState))

        return service_response

    def _cb_getCartesian(self, service_req):
        '''
        @summary: A method to handle the services for
                  get{Current, Reference}{Position, RPY} methods. Internally
                  distinguish the service type by
                  GetCartesianCommon.method_type_id.
        @type service_req: hironx_rpc_msgs.srv.GetCartesianCommon
        '''
        method_id = service_req.method_type_id
        linkname = service_req.linkname
        frame_name = service_req.frame_name
        rospy.loginfo('Service requested {} '.format(service_req))
        service_response = None
        ret = []
        if method_id in (1, 3, 4, 6):  # return value: vec3
            if 1 == method_id:
                ret = self._robotif.getCurrentPosition(linkname, frame_name)
            elif 3 == method_id:
                ret = self._robotif.getCurrentRPY(linkname, frame_name)
            elif 4 == method_id:
                ret = self._robotif.getReferencePosition(linkname, frame_name)
            elif 6 == method_id:
                ret = self._robotif.getReferenceRPY(linkname, frame_name)
            else:
                return False  # TODO: Throw exception or service
            service_response = GetCartesianCommonResponse(
                vec3=Vector3(ret[0], ret[1], ret[2]))
        else:   # return value: rot
            if 2 == method_id:
                ret = self._robotif.getCurrentRotation(linkname, frame_name)
            elif 5 == method_id:
                ret = self._robotif.getReferenceRotation(linkname, frame_name)
            else:
                return False  # TODO: Throw exception or service
            rot_matrix = []
            for each_row in ret:
                vals_per_eef = []  # Initialize
                for each_val in each_row:
                    vals_per_eef.append(each_val)
                eef = Float32List()
                eef.data = vals_per_eef
                rot_matrix.append(eef)
            service_response = GetCartesianCommonResponse(rot=rot_matrix)
        return service_response

    def _cb_getSensors(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.GetSensors
        '''
        modelfile_url = service_req.modelfile_url
        rospy.loginfo('Service requested {} '.format(service_req))
        ret = self._robotif.goSensors(url=modelfile_url)
        return GetSensorsResponse(ret.strings)

    def _cb_goInitOffPoses(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.GoInitOffPoses
        '''
        method_type_id = service_req.method_type_id
        time = service_req.time
        wait = service_req.wait
        init_pose_type = service_req.init_pose_type
        rospy.loginfo('Service requested {} '.format(service_req))
        if 1 == method_type_id:
            ret = self._robotif.goInitial(tm=time, wait=wait, init_pose_type=init_pose_type)
            return GoInitOffPosesResponse(ret)
        elif 2 == method_type_id:
            self._robotif.goOffPose(tm=time)
            return GoInitOffPosesResponse()
        else:
            return False  # TODO: Throw exception or service

    def _cb_servoOperation(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.ServoOperation
        '''
        method_type_id = service_req.method_type_id
        joint_name = service_req.joint_name
        time = service_req.time
        wait = service_req.wait
        rospy.loginfo('Service requested {} '.format(service_req))
        if 1 == method_type_id:
            ret = self._robotif.servoOn(jname=joint_name, tm=time)
            return ServoOperationResponse(result_on=ret)
        elif 2 == method_type_id:
            ret = self._robotif.servoOff(jname=joint_name, wait=wait)
            return ServoOperationResponse(result_off=ret)

    def _init_hironx_rtmclient(self):
        ros_ns = 'rpc_servers_handler'
        modelfile_location_realrobot = '/opt/jsk/etc/HIRONX/model/main.wrl'
        CORBA_NAMESERVER_NAME = rospy.get_param(ros_ns + "/CORBA_NAMESERVER_NAME", 'hiro')
        CORBA_PORT = rospy.get_param(ros_ns + "/CORBA_PORT", 15005)
        MODELFILE_HRPSYS = rospy.get_param(ros_ns + "/MODELFILE_HRPSYS", modelfile_location_realrobot)
        ROBOTNAME_HRPSYS = rospy.get_param(ros_ns + "/ROBOTNAME_HRPSYS", 'RobotHardware')

        rtm.nshost = CORBA_NAMESERVER_NAME
        rtm.nsport = CORBA_PORT
        robot = hiro = hironx_client.HIRONX()
        robot.init(robotname=ROBOTNAME_HRPSYS, url=MODELFILE_HRPSYS)
        return robot
