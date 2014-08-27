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

import copy
import math

import actionlib
from moveit_commander import MoveGroupCommander
from moveit_commander import MoveItCommanderException
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

from hironx_ros_bridge.constant import Constant

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

    _MSG_NO_MOVEGROUP_FOUND = ('\nMake sure you\'ve launched MoveGroup ' +
                               '(e.g. by launching ' +
                               'moveit_planning_execution.launch)')

    def __init__(self, jointgroups=None):
        '''
        @type jointgroups: [str]
        '''
        rospy.init_node('hironx_ros_client')
        if jointgroups:
            self._set_groupnames(jointgroups)
        self._init_action_clients()

        self._movegr_larm = self._movegr_rarm = None
        try:
            self._init_moveit_commanders()
        except RuntimeError as e:
            rospy.logerr(str(e) + self._MSG_NO_MOVEGROUP_FOUND)

    def _init_moveit_commanders(self):
        '''
        @raise RuntimeError: When MoveGroup is not running.
        '''
        # left_arm, right_arm are fixed in nextage_moveit_config pkg.

        try:
            self._movegr_larm = MoveGroupCommander(Constant.GRNAME_LEFT_ARM_MOVEGROUP)
            self._movegr_rarm = MoveGroupCommander(Constant.GRNAME_RIGHT_ARM_MOVEGROUP)
        except RuntimeError as e:
            raise e

    def _init_action_clients(self):
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
        self.set_joint_angles_deg(Constant.GRNAME_TORSO, POSITIONS_TORSO_DEG, task_duration)
        POSITIONS_HEAD_DEG = [0.0, 0.0]
        self.set_joint_angles_deg(Constant.GRNAME_HEAD, POSITIONS_HEAD_DEG, task_duration)
        POSITIONS_LARM_DEG = [0.6, 0, -100, -15.2, 9.4, -3.2]
        self.set_joint_angles_deg(Constant.GRNAME_LEFT_ARM, POSITIONS_LARM_DEG, task_duration)
        POSITIONS_RARM_DEG = [-0.6, 0, -100, 15.2, 9.4, 3.2]
        self.set_joint_angles_deg(Constant.GRNAME_RIGHT_ARM, POSITIONS_RARM_DEG,
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
        if groupname == Constant.GRNAME_TORSO:
            action_client = self._aclient_torso
            goal = self._goal_torso
        elif groupname == Constant.GRNAME_HEAD:
            action_client = self._aclient_head
            goal = self._goal_head
        elif groupname == Constant.GRNAME_LEFT_ARM:
            action_client = self._aclient_larm
            goal = self._goal_larm
        elif groupname == Constant.GRNAME_RIGHT_ARM:
            action_client = self._aclient_rarm
            goal = self._goal_rarm
        else:
            #TODO: Throw exception; a valid group name isn't passed.
            rospy.logerr('groupname {} not assigned'.format(groupname))

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

    def set_pose(self, joint_group, position, rpy=None, task_duration=7.0,
                 do_wait=True, ref_frame_name=None):
        '''
        Accept pose defined by position and RPY in Cartesian format.

        @type joint_group: str
        @type position: [float]
        @param position: x, y, z.
        @type rpy: [float]
        @param rpy: If None, keep the current orientation by using
                    MoveGroupCommander.set_position_target. See:
                    http://moveit.ros.org/doxygen/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#acfe2220fd85eeb0a971c51353e437753
        @param ref_frame_name: TODO: Not utilized yet. Need to be implemented.
        '''
        # Check if MoveGroup is instantiated.
        if not self._movegr_larm or not self._movegr_rarm:
            try:
                self._init_moveit_commanders()
            except RuntimeError as e:
                rospy.logerr(self._MSG_NO_MOVEGROUP_FOUND)
                raise e

        # Locally assign the specified MoveGroup
        movegr = None
        rospy.loginfo('Constant.GRNAME_LEFT_ARM={}'.format(Constant.GRNAME_LEFT_ARM))
        if Constant.GRNAME_LEFT_ARM == joint_group:
            rospy.loginfo('222')
            movegr = self._movegr_larm
        elif Constant.GRNAME_RIGHT_ARM == joint_group:
            movegr = self._movegr_rarm
            rospy.loginfo('333')
        else:
            rospy.loginfo('444')

        # If no RPY specified, give position and return the method.
        if not rpy:
            try:
                movegr.set_position_target(position)
            except MoveItCommanderException as e:
                rospy.logerr(str(e))
            return

        # Not necessary to convert from rpy to quaternion, since
        # MoveGroupCommander.set_pose_target can take rpy format too.
        # orientation_quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose = copy.deepcopy(position)
        pose.extend(rpy)
        rospy.loginfo('ROS set_pose joint_group={} movegroup={} pose={} position={} rpy={}'.format(
                                     joint_group, movegr, pose, position, rpy))
        try:
            movegr.set_pose_target(pose)
        except MoveItCommanderException as e:
            rospy.logerr(str(e))
        except Exception as e:
            rospy.logerr(str(e))

        movegr.go(do_wait) or movegr.go(do_wait) or rospy.logerr(
                          'MoveGroup.go fails; jointgr={}'.format(joint_group))
