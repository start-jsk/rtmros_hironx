#!/usr/bin/env python

#############################################################################################
 # Copyright (c) 2014 Daiki Maekawa and ROS JAPAN Users Group All Rights Reserved.         #
 #                                                                                         #
 # @file nextage_command_server.py                                                         #
 # @brief Moving the robot arms and Subscribing to Pose.                                   #
 # @author Daiki Maekawa                                                                   #
 # @date 2014-08-26                                                                        #
#############################################################################################

import moveit_commander
import rospy
import geometry_msgs.msg

class NextageCommandServer:
    def __init__(self):
        self._robot = moveit_commander.RobotCommander()
        rospy.Subscriber("right_arm/delta_pose", geometry_msgs.msg.Pose, self.delta_pose_r_cb)
        rospy.Subscriber("left_arm/delta_pose", geometry_msgs.msg.Pose, self.delta_pose_l_cb)

        self._rarm = moveit_commander.MoveGroupCommander("right_arm")
        self._larm = moveit_commander.MoveGroupCommander("left_arm")
    
    def delta_pose_r_cb(self, pose):
        target_pose_r = self._rarm.get_current_pose().pose

        target_pose_r.position.x += pose.position.x
        target_pose_r.position.y += pose.position.y
        target_pose_r.position.z += pose.position.z
        target_pose_r.orientation.w += pose.orientation.w
        target_pose_r.orientation.x += pose.orientation.x
        target_pose_r.orientation.y += pose.orientation.y
        target_pose_r.orientation.z += pose.orientation.z

        self._rarm.set_pose_target(target_pose_r)
        self._rarm.go()
        
    def delta_pose_l_cb(self, pose):
        target_pose_l = self._larm.get_current_pose().pose
        
        target_pose_l.position.x += pose.position.x
        target_pose_l.position.y += pose.position.y
        target_pose_l.position.z += pose.position.z
        target_pose_l.orientation.w += pose.orientation.w
        target_pose_l.orientation.x += pose.orientation.x
        target_pose_l.orientation.y += pose.orientation.y
        target_pose_l.orientation.z += pose.orientation.z
        
        self._larm.set_pose_target(target_pose_l)
        self._larm.go()
    
    def run(self):
        rospy.spin()

def main():
    rospy.init_node("nextage_command_server")
    nextage_command_sender = NextageCommandServer()
    nextage_command_sender.run()

if __name__ == '__main__':
    main()

