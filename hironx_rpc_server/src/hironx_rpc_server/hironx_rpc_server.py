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

from geometry_msgs.msg import Pose, Quaternion
from hironx_ros_bridge import hironx_client
from hrpsys import rtm
import rospy
import sys

from hironx_rpc_msgs.srv import (
    CheckEncoders, CheckEncodersResponse,
    GoInitial, GoInitialResponse
)
from tork_rpc_util.rpc_servers_handler import ActionServiceInfo, RpcServersHandler


class ActionServiceNameDict(object):
    '''
    Static, entity class to hold name for ROS Actions for the robot.
    '''
    checkEncoders = 'srv_checkEncoders'
    goInitial = 'srv_goInitial'


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
            ActionServiceNameDict.goInitial: ActionServiceInfo(ActionServiceNameDict.goInitial, GoInitial, self._cb_goInitial)
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

    def _cb_goInitial(self, service_req):
        '''
        @type service_req: hironx_rpc_msgs.srv.GoInitial
        '''
        tm = service_req.tm
        wait = service_req.wait
        init_pose_type = service_req.init_pose_type
        rospy.loginfo('Service requested {} '.format(service_req))
        ret = self._robotif.goInitial(tm=tm, wait=wait, init_pose_type=init_pose_type)
        return GoInitialResponse(ret)

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
