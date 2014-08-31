#!/usr/bin/env python

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
#
# Author: @k-okada, @130s

import rospy
import tf
#import kbhit
import math

import hironx_ros_bridge
from hironx_ros_bridge import hironx_client as hironx

from visualization_msgs.msg import Marker

rospy.init_node('test')
marker_pub = rospy.Publisher('box', Marker)

# display  box
marker = Marker()
marker.header.frame_id = "/box"
marker.header.stamp = rospy.Time.now()

marker.ns = "box"
marker.id = 0

marker.type = Marker.CUBE
marker.action = Marker.ADD

marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0
marker.pose.orientation.y = 0
marker.pose.orientation.z = 0
marker.pose.orientation.w = 1
marker.scale.x = 0.05
marker.scale.y = 0.50
marker.scale.z = 0.25
marker.color.r = 0
marker.color.g = 1
marker.color.b = 0
marker.color.a = 1
marker.lifetime = rospy.Duration()

# tf
tfbroadcaster_graspobj = tf.TransformBroadcaster()
tfbroadcaster_larm = tf.TransformBroadcaster()
tfbroadcaster_rarm = tf.TransformBroadcaster()
tf_listener   = tf.TransformListener()

# robot
robot = hironx.HIRONX()
robot.init()
robot.goInitial()

# loop; this motion may by jaggy, run this on simulaiton and get `trajectory_all_joints`
rate = rospy.Rate(20)
i = 0
duration = 5
trajectory_all_joints = []  # [[float]]. This list of joint angles represents a trajectory of all joint groups (both arms, head, waist).
FRAME_BOX = '/box'
FRAME_WAIST= '/WAIST'
FRAME_BOX_LARM = "/box_left_handle"
FRAME_BOX_RARM = "/box_right_handle"

posi_box_init_x = 0.4
posi_box_init_y = 0.1
posi_box_init_z = 0.4

while i < 20 * math.pi * 2:  #TODO: What is `20 * math.pi * 2`? Loop reaches 126.
    now =  rospy.Time.now()
    changerate_y = math.cos(i / 20.0)
    changerate_x = math.cos(i / 10.0)
    marker.header.stamp = now
    marker_pub.publish(marker)
    tfbroadcaster_graspobj.sendTransform((posi_box_init_x - 0.05 * changerate_x, 
                                          posi_box_init_y * changerate_y,
                                          posi_box_init_z),  # position
                                         tf.transformations.quaternion_from_euler(0, 0, changerate_y * 0.1),  # orientation
                                         now, FRAME_BOX, FRAME_WAIST)
    tfbroadcaster_larm.sendTransform((0, 0.32, 0),  # Centroid to the rim of the box?
                                     tf.transformations.quaternion_from_euler(0, math.pi, -math.pi/2),
                                     now, FRAME_BOX_LARM, FRAME_BOX)
    tfbroadcaster_rarm.sendTransform((0, -0.32, 0),  # Centroid to the rim of the box?
                                     tf.transformations.quaternion_from_euler(0, math.pi, math.pi/2),
                                     now, FRAME_BOX_RARM, FRAME_BOX)
    try:
        (posi_global_larm, rot_global_larm) = tf_listener.lookupTransform(FRAME_WAIST, FRAME_BOX_LARM, rospy.Time(0))
        (posi_global_rarm, rot_global_rarm) = tf_listener.lookupTransform(FRAME_WAIST, FRAME_BOX_RARM, rospy.Time(0))
        rot_global_larm = tf.transformations.euler_from_quaternion(rot_global_larm)
        rot_global_rarm = tf.transformations.euler_from_quaternion(rot_global_rarm)
        robot.setTargetPose('larm', posi_global_larm, rot_global_larm, duration)
        robot.setTargetPose('rarm', posi_global_rarm, rot_global_rarm, duration)
        robot.waitInterpolationOfGroup('larm')
        robot.waitInterpolationOfGroup('rarm')
        rospy.loginfo("#{}; LARM posi={}, rot={}".format(i, posi_global_larm, rot_global_larm))
        rospy.loginfo("     RARM posi={}, rot={}".format(posi_global_rarm, rot_global_rarm))
        trajectory_all_joints.append(robot.getJointAngles())
        duration = 0.1
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    rate.sleep()
    i += 1
    rospy.loginfo('Timestamp: {}'.format(now))

i = 0
for joint_angles in trajectory_all_joints:
    rospy.loginfo('#{} angles: {}'.format(i, joint_angles))
    i += 1

# using play pattern of group, this may be smoother

trajectory_larm = []  # Though defined as 1-D list, used as [[float]].
trajectory_rarm = []  # Internal list represents the joint angles at a pose.
duration = 2
durations = []  # [float]: list of durations at a pose.
for joint_angles in trajectory_all_joints:
    jointangles_larm = joint_angles[14:20]
    jointangles_rarm = joint_angles[4:10]

    # radian to degree for playPatternOfGroup.
    jointangles_larm = [x * math.pi / 180.0 for x in jointangles_larm]
    jointangles_rarm = [x * math.pi / 180.0 for x in jointangles_rarm]

    trajectory_larm.append(jointangles_larm)
    trajectory_rarm.append(jointangles_rarm)
    durations.append(duration)
    duration = 0.1

robot.seq_svc.playPatternOfGroup('larm', trajectory_larm, durations)
robot.seq_svc.playPatternOfGroup('rarm', trajectory_rarm, durations)  #TODO: this method should be added to hrpsys_config.py
robot.waitInterpolationOfGroup('rarm')

