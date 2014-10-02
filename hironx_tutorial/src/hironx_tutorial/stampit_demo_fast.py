#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Tokyo Opensource Robotics Kyokai Association
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

import argparse
import time

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import rospy

#from hrpsys.hrpsys_config import *
#import rtm
from nextage_ros_bridge.nextage_client import NextageClient

rospy.init_node("stampit_demo")

rarm = MoveGroupCommander("right_arm")
# rarm.set_workspace([0.29, 0, 0.55, 0.9, 1.10, 0.95])
larm = MoveGroupCommander("left_arm")
# larm.set_workspace([0.29, 0, 0.55, 0.9, 1.10, 0.95])


def set_pose_target(arm, pos):
    '''
    Set the pose of the end-effector, if one is available.

    @type arm: moveit_commander.MoveGroupCommander
    @param pos: destination position
    @type pos: geometry_msgs.Pose
    '''
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = '/WAIST_Link'  # '/odom'
    pose_stamped.pose.position.x = pos[0]
    pose_stamped.pose.position.y = pos[1]
    pose_stamped.pose.position.z = pos[2]
    # tf.transformations.quaternion_from_euler(0.0, 0.0, 1.57, 'sxyz')
    pose_stamped.pose.orientation.x = pos[3]
    pose_stamped.pose.orientation.y = pos[4]  # 0.5  # -0.1
    pose_stamped.pose.orientation.z = pos[5]  # 0.5  # 1.03  # 0.70682518
    # pose_stamped.pose.orientation.w = 1  # 0.5  # 1.03  # 0.70682518
    # arm.set_pose_target(pose_stamped)
    arm.set_pose_target(pos)
    rospy.loginfo("setting pose at {}".format(pose_stamped))

    # This is tken from demo method. Not sure if calling it here works.
    arm.go() or arm.go() or rospy.logerr('arm.go fails {}'.format(''))
    return pose_stamped


def demo(nextage_client, num_repeat=1):
    _num_repeat = num_repeat
    _sleeptime_general = 0.0
    _sleeptime_drawin = 1.0

    _h_table = 0.0195  # 0.0145
    _h_seal = 0.105
    _h_paperbox = 0.005  # 0.0118  # 0.0115  # 0.012

    _x_l_away = 0.35
    _x_r_away = 0.325
    _x_r_grab_diff = 0.005
    _x_r_away_above = 0.37
    _x_stamp = 0.3190117808463611 + 0.17
    _x_r_givepaper = _x_l_away + 0.18
    _y_l_away = 0.48
    _y_box_outer = -0.31
    _y_stamp = 0.12
    _y_stamp_place = 0.20
    _y_stamp_grab = 0.18
    _y_hoist = 0.07
    _y_box_center = -0.055254676830894994
    _y_givepaper = -0.12
    _z_stamp = _h_table + _h_seal
    _h_onemomemnt = 0.04  # 0.02
    _h_onemomemnt_postgrab = 0.015
    _z_stamp_onemoment = _z_stamp + _h_onemomemnt
    _z_papergrab = _h_table + _h_paperbox
    _z_paper_release_gentle = _h_table + _h_onemomemnt
    _z_papergrab_onemoment = _z_papergrab + _h_onemomemnt_postgrab
    _z_hoist = 0.31
    _z_hoist_diff = _z_hoist - _z_papergrab_onemoment

    _roll_stamp = 0.05
    _pitch_normal = -1.6
    _pitch_l_normal = -1.57
    _pitch_normal_headup = _pitch_l_normal + 0.002
    _pitch_normal_headdown = _pitch_l_normal - 0.002
    _pitch_grab = -1.57

    _L1 = [_x_l_away, _y_l_away, _z_hoist, 0, _pitch_normal, 0]  # Move Larm away

    _R1 = [_x_r_away, _y_box_center, _z_hoist, 0, _pitch_grab, 0]  # Rarm init pos.
    _R1_1 = [_x_r_away, _y_box_center, _z_papergrab_onemoment, 0, _pitch_grab, 0]  # 1moment before grab
    _R2 = [_x_r_away, _y_box_center, _z_papergrab, 0, _pitch_grab, 0]  # grabing paper.
    _R2_2 = [_x_r_away + _x_r_grab_diff, _y_box_center, _z_papergrab_onemoment, 0, _pitch_grab, 0]  # post grab onemoememt
    _R2_3 = [_x_r_away + _x_r_grab_diff, _y_box_center, _z_papergrab_onemoment + _z_hoist_diff*0.10 , 0, _pitch_grab, 0]  # post grab onemoememt
    _R2_4 = [_x_r_away + _x_r_grab_diff, _y_box_center, _z_papergrab_onemoment + _z_hoist_diff*0.40, 0, _pitch_grab, 0]  # post grab onemoememt
    _R2_5 = [_x_r_away + _x_r_grab_diff, _y_box_center, _z_papergrab_onemoment + _z_hoist_diff*0.70, 0, _pitch_grab, 0]  # post grab onemoememt
    _HEAD3 = 'h3'  # Turn head to left
    _R3 = [_x_r_away, _y_box_center, _z_hoist, 0, _pitch_normal, 0]  # Hoist above.
    _R4 = [_x_r_away, _y_stamp_place, _z_hoist, 0, _pitch_normal, 0]     # Move to stamping area above.
    _R5 = [_x_r_away, _y_stamp_place, _z_papergrab, 0, _pitch_grab, 0]   # Release paper.
    _R6 = _R1                                 # Move back to init pos

    _L2 = [_x_l_away, _y_stamp, _z_hoist, 0, _pitch_l_normal, 0]  # Move to above stamping pos
    _L3_1 = [_x_stamp, _y_stamp, _z_stamp_onemoment, 0, _pitch_l_normal, 0]  # Stamp!!!!
    _L3_2 = [_x_stamp, _y_stamp, _z_stamp, 0, _pitch_normal_headdown, 0]  # Stamp!!!!
    #_L3_3 = [_x_l_away, _y_stamp, _z_stamp, 0, _pitch_normal, 0]  # Grind seal head
    #_L3_4 = [_x_l_away, _y_stamp, _z_stamp, 0, _pitch_normal_headup, 0]  # Grind seal head
    _L4 = [_x_stamp, _y_stamp, _z_hoist, 0, _pitch_normal, 0]    # Hoist back up a little.
    _L5 = [_x_l_away, _y_l_away, _z_hoist, 0, _pitch_normal, 0]  # Move away.

    _R7 = _R4                                 # Move to stamping pos to grab
    _R8 = [_x_r_away, _y_stamp_grab, _h_table, 0, _pitch_grab, 0]  # Grab a stamped paper.
    _R8_2 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment, 0, _pitch_grab, 0]  # One moment
    _R9_1 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment + _z_hoist_diff*0.10  , 0, _pitch_grab, 0]  # Hoist up moving x-y to avoid void
    _R9_2 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment + _z_hoist_diff*0.30  , 0, _pitch_grab, 0]  # Hoist up moving x-y to avoid void
    _R9_3 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment + _z_hoist_diff*0.5  , 0, _pitch_normal, 0]
    _R9_4 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment + _z_hoist_diff*0.7  , 0, _pitch_normal, 0]
    _R9_5 = [_x_r_away, _y_stamp_grab, _z_papergrab_onemoment + _z_hoist_diff*0.9  , 0, _pitch_normal, 0]
    # Goal position.
    _HEAD10 = 'h10'
    _R10 = [_x_r_away_above, _y_box_outer, _z_hoist, 0, _pitch_normal, 0]  # Above final pos
    _HEAD11 = 'h11'
    _R11 = [_x_r_away_above, _y_box_outer, _z_paper_release_gentle, 0, _pitch_normal, 0]  # Final pos.
    _R12 = [_x_r_away_above, _y_box_outer, _z_hoist, 0, _pitch_normal, 0]  # hoist up final.

    _positions_larm_1 = [_L1]
    #_positions_rarm_1 = [_R1, _R1_1, _R2]
    _positions_rarm_1_1 = [_R2_2, _R2_3, _R2_4, _R2_5, _R3, _HEAD3, _R4, _R5]
    _positions_rarm_1_2 = [_R6]
    #_positions_larm_2 = [_L2, _L3_1, _L3_2, _L3_3, _L3_4, _L4, _L5]
    _positions_larm_2 = [_L2, _L3_1, _L3_2, _L3_1, _L4, _L5]
    _positions_rarm_2 = [_R7, _R8]
    _positions_rarm_3 = [_R8_2, _R9_1, _R9_2, _R9_3, _R9_4, _HEAD10, _R10, _HEAD11, _R11]
    _positions_rarm_4 = [_R12]

    _LINKGRP_HEAD = 'head'

    for i in range(_num_repeat):
        nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [0, 30], 0.5)
        for pos_next in _positions_larm_1:
            set_pose_target(larm, pos_next)
            time.sleep(_sleeptime_general)
        nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [0, 50], 0.5)

        nextage_client.setTargetPoseRelative('rarm', 'RARM_JOINT5',
                                              dy=0.05, )

        for pos_next in _positions_rarm_1:
            set_pose_target(rarm, pos_next)
            time.sleep(_sleeptime_general)
        nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [0, 40], 0.5)
#        nextage_client.setTargetPoseRelative('rarm', 'RARM_JOINT5',
#                                              dz=-(_z_hoist-_z_papergrab))
        nextage_client.airhand_drawin_r()
        time.sleep(_sleeptime_drawin)
        #nextage_client.setTargetPoseRelative('rarm', 'RARM_JOINT5', 0.05, tm=2)
        nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [0, 30], 0.5)
        for pos_next in _positions_rarm_1_1:
            if _HEAD3 == pos_next:
                nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [20, 20], 0.5, False)
                continue
            set_pose_target(rarm, pos_next)
            time.sleep(_sleeptime_general)
        nextage_client.airhand_release_r()
        for pos_next in _positions_rarm_1_2:
            set_pose_target(rarm, pos_next)
            rospy.loginfo('SET {}th paper.'.format(i))
            time.sleep(_sleeptime_general)
        nextage_client.airhand_keep_r()
        for pos_next in _positions_larm_2:
            set_pose_target(larm, pos_next)
            rospy.loginfo('STAMPED {}th paper.'.format(i))
            time.sleep(_sleeptime_general)
        for pos_next in _positions_rarm_2:
            set_pose_target(rarm, pos_next)
            time.sleep(_sleeptime_general)
        nextage_client.airhand_drawin_r()
        time.sleep(_sleeptime_drawin)
        for pos_next in _positions_rarm_3:
            if _HEAD10 == pos_next:
                nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [-20, 30], 0.5, False)
                continue
            elif _HEAD11 == pos_next:
                nextage_client.setJointAnglesOfGroup(_LINKGRP_HEAD, [-30, 30], 0.5, False)
                continue
            set_pose_target(rarm, pos_next)
            time.sleep(_sleeptime_general)
        nextage_client.airhand_release_r()
        nextage_client.airhand_keep_r()
        for pos_next in _positions_rarm_4:
            set_pose_target(rarm, pos_next)
            time.sleep(_sleeptime_general)
        rospy.loginfo('DONE {}th paper.'.format(i))
        time.sleep(0.5)
# https://github.com/start-jsk/open-industrial-controllers/commit/d3d6c5276bdc7bcd7a9413cf67208538dddaf38b

if __name__ == '__main__':
    # Copied from hironx.py

    parser = argparse.ArgumentParser(description='hiro command line interpreters')
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('--modelfile', help='robot model file nmae')
    parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
    args, unknown = parser.parse_known_args()

    if args.host:
        rtm.nshost = args.host
    if args.port:
        rtm.nsport = args.port
    if not args.robot:
        args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
    if not args.modelfile:
        args.modelfile = ""

    # support old style format
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]

    nc = NextageClient()  # hiro.init(robotname=args.robot, url=args.modelfile)
    rospy.loginfo('Nextage goinit started.')
    nc.init(robotname=args.robot, url=args.modelfile)
    nc.goInitial(3)
    rospy.loginfo('Nextage goinit done.')
    demo(nc, 20)
