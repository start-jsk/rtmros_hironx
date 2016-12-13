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
import socket

import actionlib
import moveit_commander
from moveit_commander import MoveGroupCommander, MoveItCommanderException, RobotCommander
import rospy
from rospy import ROSInitException
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix, translation_from_matrix, euler_from_matrix
import numpy

from hironx_ros_bridge.constant import Constant

_MSG_ASK_ISSUEREPORT = 'Your report to ' + \
                       'https://github.com/start-jsk/rtmros_hironx/issues ' + \
                       'about the issue you are seeing is appreciated.'


# workaround for core dump whenever exiting Python MoveIt script (https://github.com/ros-planning/moveit_commander/issues/15#issuecomment-34441531)
import atexit, os
atexit.register(lambda : os._exit(0))

class ROS_Client(RobotCommander):
    '''
    This class holds methods that are specific to Kawada Industries' dual-arm
    robot called Hiro, via ROS.

    @since: December 2014: Now this class is replacing the default programming interface
            with HIRONX (hironx_client.py).
    @since: July 2014: this class is only intended to be used through HIRONX
      class.
    '''

    _MSG_NO_MOVEGROUP_FOUND = ('\nMake sure you\'ve launched MoveGroup ' +
                               '(e.g. by launching ' +
                               'moveit_planning_execution.launch)')

    def __init__(self, jointgroups=None):
        '''
        @param jointgroups [str]: Deprecated. No need after version 1.1.4 onward.
        '''
        # if we do not have ros running, return 
        try:
            rospy.get_master().getSystemState()
        except socket.error as e:
            errormsg = 'No ROS Master found. Without it, you cannot use ROS from this script, but you can still use RTM without issues. ' + \
                       'To use ROS, do not forget to run rosbridge. How to do so? --> http://wiki.ros.org/rtmros_nextage/Tutorials/Operating%20Hiro%2C%20NEXTAGE%20OPEN'
            raise ROSInitException(errormsg)
        except Exception as e:
            errormsg = '[ros_client] Unknown exception occurred, so do not create ros client...'
            rospy.logerr(errormsg)
            raise e

        super(ROS_Client, self).__init__()  # This solves https://github.com/start-jsk/rtmros_hironx/issues/300

        rospy.init_node('hironx_ros_client')

        if not rospy.has_param('robot_description'):
            rospy.logwarn('ROS Bridge is not started yet, Assuming you want just to use RTM')
            return

        # See the doc in the method for the reason why this line is still kept
        # even after this class has shifted MoveIt! intensive.
        self._init_action_clients()

        if not rospy.has_param('robot_description_semantic'):
            rospy.logwarn('Moveit is not started yet, if you want to use MoveIt!' + self._MSG_NO_MOVEGROUP_FOUND)
            return

        try:
            self._init_moveit_commanders()
        except RuntimeError as e:
            rospy.logerr(str(e) + self._MSG_NO_MOVEGROUP_FOUND)

#    def __getattr__(self, name):
#        '''
#        Trying to resolve https://github.com/start-jsk/rtmros_hironx/issues/300
#        '''
#        return object.__getattr__(self, name)

    def _init_moveit_commanders(self):
        '''
        @raise RuntimeError: When MoveGroup is not running.
        '''
        # left_arm, right_arm are fixed in nextage_moveit_config pkg.
        try:
            self.MG_LARM = self.get_group(Constant.GRNAME_LEFT_ARM_MOVEGROUP)
            self.MG_RARM = self.get_group(Constant.GRNAME_RIGHT_ARM_MOVEGROUP)
            self.MG_BOTHARMS = self.get_group(Constant.GRNAME_BOTH_ARMS)
            self.MG_HEAD = self.get_group(Constant.GRNAME_HEAD)
            self.MG_TORSO = self.get_group(Constant.GRNAME_TORSO)
            self.MG_UPPERBODY = self.get_group(Constant.GRNAME_UPPERBODY)
            self.MG_LARM.set_planner_id("RRTConnectkConfigDefault")
            self.MG_RARM.set_planner_id("RRTConnectkConfigDefault")
            self.MG_BOTHARMS.set_planner_id("RRTConnectkConfigDefault")
            self.MG_HEAD.set_planner_id("RRTConnectkConfigDefault")
            self.MG_TORSO.set_planner_id("RRTConnectkConfigDefault")
            self.MG_UPPERBODY.set_planner_id("RRTConnectkConfigDefault")

            # TODO: Why the ref frames need to be kept as member variables?
            self._movegr_larm_ref_frame = self.MG_LARM.get_pose_reference_frame()
            self._movegr_rarm_ref_frame = self.MG_RARM.get_pose_reference_frame()
            self._movegr_botharms_ref_frame = self.MG_BOTHARMS.get_pose_reference_frame()
        except RuntimeError as e:
            raise e

    def _init_action_clients(self):
        '''
        This is only needed for accessing Actionlib clients directly, which
        is no longer needed for this class now that it inherits
        RobotCommander from MoveIt!. Still this line is kept for the methods
        deprecated but remain for backward compatibility.
        '''
        self._aclient_larm = actionlib.SimpleActionClient(
            '/larm_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
        self._aclient_rarm = actionlib.SimpleActionClient(
            '/rarm_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
        self._aclient_head = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)
        self._aclient_torso = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory_action', FollowJointTrajectoryAction)

        self._aclient_larm.wait_for_server()
        self._goal_larm = FollowJointTrajectoryGoal()
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT0")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT1")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT2")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT3")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT4")
        self._goal_larm.trajectory.joint_names.append("LARM_JOINT5")

        self._aclient_rarm.wait_for_server()
        self._goal_rarm = FollowJointTrajectoryGoal()
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT0")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT1")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT2")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT3")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT4")
        self._goal_rarm.trajectory.joint_names.append("RARM_JOINT5")

        self._aclient_head.wait_for_server()
        self._goal_head = FollowJointTrajectoryGoal()
        self._goal_head.trajectory.joint_names.append('HEAD_JOINT0')
        self._goal_head.trajectory.joint_names.append('HEAD_JOINT1')

        self._aclient_torso.wait_for_server()
        self._goal_torso = FollowJointTrajectoryGoal()
        self._goal_torso.trajectory.joint_names.append('CHEST_JOINT0')

        rospy.loginfo('Joint names; ' +
                      'Torso: {}\n\tHead: {}\n\tL-Arm: {}\n\tR-Arm: {}'.format(
                          self._goal_torso.trajectory.joint_names,
                          self._goal_head.trajectory.joint_names,
                          self._goal_larm.trajectory.joint_names,
                          self._goal_rarm.trajectory.joint_names))

    def go_init(self, init_pose_type=0, task_duration=7.0):
        '''
        Change the posture of the entire robot to the pre-defined init pose.

        This method is equivalent to
        hironx_ros_bridge.hironx_client.HIRONX.goInitial method (https://github.com/start-jsk/rtmros_hironx/blob/83c3ff0ad2aabd8525631b08276d33b09c98b2bf/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py#L199),
        with an addition of utilizing MoveIt!, which means e.g. if there is
        possible perceived collision robots will take the path MoveIt! computes
        with collision avoidance taken into account.

        @type task_duration: float
        @param init_pose_type:
               0: default init pose (specified as _InitialPose)
               1: factory init pose (specified as _InitialPose_Factory)
        '''
        rospy.loginfo('*** go_init begins ***')
        posetype_str = ''
        if 0 == init_pose_type:
            posetype_str = 'init_rtm'
        elif 1 == init_pose_type:
            posetype_str = 'init_rtm_factory'
        else:
            rospy.logerr("unsupporeted init_pose_type " + str(init_pose_type))
        self.MG_BOTHARMS.set_named_target(posetype_str)
        self.MG_BOTHARMS.go()

    def go_offpose(self, task_duration=7.0):
        self.MG_UPPERBODY.set_named_target(Constant.POSE_OFF)
        self.MG_UPPERBODY.go()

    def goInitial(self, init_pose_type=0, task_duration=7.0):
        '''
        This method internally calls self.go_init.

        This method exists solely because of compatibility purpose with
        hironx_ros_bridge.hironx_client.HIRONX.goInitial, which
        holds a method "goInitial".

        @param init_pose_type:
               0: default init pose (specified as _InitialPose)
               1: factory init pose (specified as _InitialPose_Factory)
        '''
        return self.go_init(init_pose_type, task_duration)

    def set_joint_angles_rad(self, groupname, positions_radian, duration=7.0,
                             wait=False):
        '''
        @deprecated: Use MoveitCommander.set_joint_value_target instead.

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
            # TODO: Throw exception; a valid group name isn't passed.
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
                rospy.loginfo(
                    'wait_for_result for the joint group {} = {}'.format(
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
        @deprecated: Use set_pose_target (from MoveGroupCommander) directly.
        Accept pose defined by position and RPY in Cartesian format.

        @type joint_group: str
        @type position: [float]
        @param position: x, y, z.
        @type rpy: [float]
        @param rpy: If None, keep the current orientation by using
                    MoveGroupCommander.set_position_target. See:
                     'http://moveit.ros.org/doxygen/' +
                     'classmoveit__commander_1_1move__group_1_1' +
                     'MoveGroupCommander.html#acfe2220fd85eeb0a971c51353e437753'
        @param ref_frame_name: reference frame for target pose, i.e. "LARM_JOINT5_Link".
        '''
        # convert to tuple to list
        position = list(position)
        if not rpy is None:
            rpy = list(rpy)
        #
        # Check if MoveGroup is instantiated.
        if not self.MG_LARM or not self.MG_RARM:
            try:
                self._init_moveit_commanders()
            except RuntimeError as e:
                rospy.logerr(self._MSG_NO_MOVEGROUP_FOUND)
                raise e

        # Locally assign the specified MoveGroup
        movegr = None
        if Constant.GRNAME_LEFT_ARM == joint_group:
            movegr = self.MG_LARM
        elif Constant.GRNAME_RIGHT_ARM == joint_group:
            movegr = self.MG_RARM
        else:
            rospy.logerr('joint_group must be either %s, %s or %s'%(Constant.GRNAME_LEFT_ARM,
                                                                    Constant.GRNAME_RIGHT_ARM,
                                                                    Constant.GRNAME_BOTH_ARMS))
            return

        # set reference frame
        if ref_frame_name :
            ref_pose = movegr.get_current_pose(ref_frame_name).pose
            ref_mat = compose_matrix(
                translate = [ref_pose.position.x, ref_pose.position.y, ref_pose.position.z],
                angles = list(euler_from_quaternion([ref_pose.orientation.x,ref_pose.orientation.y,ref_pose.orientation.z,ref_pose.orientation.w])))
            target_mat = numpy.dot(ref_mat, compose_matrix(translate = position, angles = rpy or [0, 0, 0]))
            position = list(translation_from_matrix(target_mat))
            rpy = list(euler_from_matrix(target_mat))

        # If no RPY specified, give position and return the method.
        if not rpy:
            try:
                movegr.set_position_target(position)
            except MoveItCommanderException as e:
                rospy.logerr(str(e))
            (movegr.go(do_wait) or movegr.go(do_wait) or
             rospy.logerr('MoveGroup.go fails; jointgr={}'.format(joint_group)))
            return

        # Not necessary to convert from rpy to quaternion, since
        # MoveGroupCommander.set_pose_target can take rpy format too.
        pose = copy.deepcopy(position)
        pose.extend(rpy)
        rospy.loginfo('setpose jntgr={} mvgr={} pose={} posi={} rpy={}'.format(
                      joint_group, movegr, pose, position, rpy))
        try:
            movegr.set_pose_target(pose)
        except MoveItCommanderException as e:
            rospy.logerr(str(e))
        except Exception as e:
            rospy.logerr(str(e))

        (movegr.go(do_wait) or movegr.go(do_wait) or
         rospy.logerr('MoveGroup.go fails; jointgr={}'.format(joint_group)))
