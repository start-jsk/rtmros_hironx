## -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Tokyo Opensource Robotics Kyokai Association (TORK)
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

import math

import actionlib
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

_MSG_ASK_ISSUEREPORT = 'Your report to ' + \
                       'https://github.com/start-jsk/rtmros_hironx/issues ' + \
                       'about the issue you are seeing is appreciated.'


class ROS_Client(object):
    '''
    This class:

    - holds methods that are specific to Kawada Industries' dual-arm
    robot called Hiro, via ROS.
    - As of July 2014, this class is only intended to be used through HIRONX
      class.
    '''

    #TODO: Address the following concern.
    # This duplicates "Group" definition in HIRONX, which is bad.
    # Need to consider consolidating the definition either in hironx_ros_bridge
    # or somewhere in the upstream, e.g.:
    # https://github.com/fkanehiro/hrpsys-base/pull/253
    _GR_TORSO = 'torso'  # Default values are set.
    _GR_HEAD = 'head'
    _GR_LARM = 'larm'
    _GR_RARM = 'rarm'

    def __init__(self, jointgroups=None):
        '''
        @type jointgroups: [str]
        '''
        rospy.init_node('hironx_ros_client')
        if jointgroups:
            self._set_groupnames(jointgroups)

    def init_action_clients(self):
        self._aclient_larm = actionlib.SimpleActionClient(
             '/larm_controller/joint_trajectory_action', JointTrajectoryAction)
        self._aclient_rarm = actionlib.SimpleActionClient(
             '/rarm_controller/joint_trajectory_action', JointTrajectoryAction)
        self._aclient_head = actionlib.SimpleActionClient(
             '/head_controller/joint_trajectory_action', JointTrajectoryAction)
        self._aclient_torso = actionlib.SimpleActionClient(
            '/torso_controller/joint_trajectory_action', JointTrajectoryAction)

        self._aclient_larm.wait_for_server()
        rospy.loginfo('ros_client; 1')
        self._goal_larm = JointTrajectoryGoal()
        rospy.loginfo('ros_client; 2')
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT0")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT1")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT2")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT3")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT4")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT5")
        rospy.loginfo('ros_client; 3')

        self._aclient_rarm.wait_for_server()
        self._goal_rarm = JointTrajectoryGoal()
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT0")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT1")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT2")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT3")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT4")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT5")

        self._aclient_head.wait_for_server()
        self._goal_head = JointTrajectoryGoal()
        self._goal_head.trajectory.joint_names.append('HEAD_JOINT0')
        self._goal_head.trajectory.joint_names.append('HEAD_JOINT1')

        self._aclient_torso.wait_for_server()
        self._goal_torso = JointTrajectoryGoal()
        self._goal_torso.trajectory.joint_names.append('CHEST_JOINT0')

        rospy.loginfo('Joint names; ' +
                      'Torso: {}\n\tHead: {}\n\tL-Arm: {}\n\tR-Arm: {}'.format(
                                    self._goal_torso.trajectory.joint_names,
                                    self._goal_head.trajectory.joint_names,
                                    self._goal_larm.trajectory.joint_names,
                                    self._goal_rarm.trajectory.joint_names))
        
    def _set_groupnames(self, groupnames):
        '''
        @type groupnames: [str]
        @param groupnames: List of the joint group names. Assumes to be in the
                           following order:
                               torso, head, right arm, left arm.
                           This current setting is derived from the order of
                           Groups argument in HIRONX class. If other groups
                           need to be defined in the future, this method may
                           need to be modified.
        '''
        rospy.loginfo('_set_groupnames; groupnames={}'.format(groupnames))
        self._GR_TORSO = groupnames[0]
        self._GR_HEAD = groupnames[1]
        self._GR_RARM = groupnames[2]
        self._GR_LARM = groupnames[3]

    def go_init(self, task_duration=7.0):
        '''
        Init positions are taken from HIRONX.
        TODO: Need to add factory position too that's so convenient when
              working with the manufacturer.
        @type task_duration: float
        '''
        rospy.loginfo('*** go_init begins ***')
        POSITIONS_TORSO_DEG = [0.0]
        self.set_joint_angles_deg(self._GR_TORSO, POSITIONS_TORSO_DEG, task_duration)
        POSITIONS_HEAD_DEG = [0.0, 0.0]
        self.set_joint_angles_deg(self._GR_HEAD, POSITIONS_HEAD_DEG, task_duration)
        POSITIONS_LARM_DEG = [0.6, 0, -100, -15.2, 9.4, -3.2]
        self.set_joint_angles_deg(self._GR_LARM, POSITIONS_LARM_DEG, task_duration)
        POSITIONS_RARM_DEG = [-0.6, 0, -100, 15.2, 9.4, 3.2]
        self.set_joint_angles_deg(self._GR_RARM, POSITIONS_RARM_DEG,
                                  task_duration, wait=True)
        rospy.loginfo(self._goal_larm.trajectory.points)

    def set_joint_angles_rad(self, groupname, positions_radian, duration=7.0,
                             wait=False):
        '''
        @type groupname: str
        @param groupname: This should exist in self.groupnames.
        @type positions_radian: [float]
        @type duration: float
        @type wait: bool
        '''
        if groupname == self._GR_TORSO:
            action_client = self._aclient_torso
            goal = self._goal_torso
        elif groupname == self._GR_HEAD:
            action_client = self._aclient_head
            goal = self._goal_head
        elif groupname == self._GR_LARM:
            action_client = self._aclient_larm
            goal = self._goal_larm
        elif groupname == self._GR_RARM:
            action_client = self._aclient_rarm
            goal = self._goal_rarm
        else:
            #TODO: Throw exception; a valid group name isn't passed.
            pass

        try:
            pt = JointTrajectoryPoint()
            pt.positions = positions_radian
            pt.time_from_start = rospy.Duration(duration)
            goal.trajectory.points = [pt]
            goal.trajectory.header.stamp = \
                             rospy.Time.now() + rospy.Duration(duration)

            action_client.send_goal(goal)
            if wait:
                rospy.loginfo('wait_for_result for the joint group {} = {}'.format(
                                   groupname, action_client.wait_for_result()))
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e.str())

    def set_joint_angles_deg(self, groupname, positions_deg, duration=7.0,
                             wait=False):
        '''
        @type groupname: str
        @param groupname: This should exist in self.groupnames.
        @type positions_deg: [float]
        @type duration: float
        @type wait: bool
        '''
        self.set_joint_angles_rad(groupname, self._to_rad_list(positions_deg),
                                  duration, wait)

    def _to_rad_list(self, list_degree):
        '''
        @TODO Needs to be replaced by something more common, or at least moved
              somewhere more common.

        @type list_degree: [float]
        @param list_degree: A list length of the number of joints.
        '''
        list_rad = []
        for deg in list_degree:
            rad = math.radians(deg)
            list_rad.append(rad)
            rospy.logdebug('Position deg={} rad={}'.format(deg, rad))
        return list_rad
