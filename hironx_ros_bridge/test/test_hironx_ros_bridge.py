#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'hironx_ros_bridge'
# rosbuild needs load_manifest
try:
    import roslib
    import hironx_ros_bridge
except:
    import roslib; roslib.load_manifest(PKG)
    import hironx_ros_bridge

from hironx_ros_bridge import hironx_client as hironx
import rospy, actionlib, math, numpy
import tf
from tf.transformations import quaternion_matrix, euler_from_matrix

import unittest

# for catkin compiled environment, pr2_controller_msgs is not catkinized
roslib.load_manifest('pr2_controllers_msgs')
import pr2_controllers_msgs.msg
import trajectory_msgs.msg

from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from hrpsys_ros_bridge.srv import *

import time
import tempfile

class TestHiroROSBridge(unittest.TestCase):

    def joint_states_cb(self, msg):
        self.joint_states.append(msg)
        self.joint_states = self.joint_states[0:3000]

    def __init__(self, *args, **kwargs):
        super(TestHiroROSBridge, self).__init__(*args, **kwargs)
        #
        self.joint_states = []
        rospy.init_node('hironx_ros_bridge_test')
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_cb)
        self.filename_base = tempfile.mkstemp()[1]
        self.filenames = []

    @classmethod
    def setUpClass(self):
        self.robot = hironx.HIRONX()
        self.robot.init()

        self.listener = tf.TransformListener()

        self.larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.rarm = actionlib.SimpleActionClient("/rarm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.torso = actionlib.SimpleActionClient("/torso_controller/joint_trajectory_action", JointTrajectoryAction)
        self.head = actionlib.SimpleActionClient("/head_controller/joint_trajectory_action", JointTrajectoryAction)
        self.larm.wait_for_server()
        self.rarm.wait_for_server()
        self.torso.wait_for_server()
        self.head.wait_for_server()

        rospy.wait_for_service('/SequencePlayerServiceROSBridge/setTargetPose')
        self.set_target_pose = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/setTargetPose', OpenHRP_SequencePlayerService_setTargetPose)
        rospy.wait_for_service('/SequencePlayerServiceROSBridge/waitInterpolationOfGroup')
        self.wait_interpolation_of_group = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/waitInterpolationOfGroup', OpenHRP_SequencePlayerService_waitInterpolationOfGroup)


    def tearDown(self):
        self.reset_Pose()
        True

    def setUp(self):
        self.reset_Pose()

    def reset_Pose(self):
        larm_goal = self.goal_LArm()
        larm_goal = self.setup_Positions(larm_goal, [[ 0.0, -40.0, -90.0, 0.0, 0.0, 0.0]])
        rarm_goal = self.goal_RArm()
        rarm_goal = self.setup_Positions(rarm_goal, [[ 0.0, -40.0, -90.0, 0.0, 0.0, 0.0]])
        head_goal = self.goal_Head()
        head_goal = self.setup_Positions(head_goal, [[0, 20]])
        torso_goal = self.goal_Torso()
        torso_goal = self.setup_Positions(torso_goal, [[0]])
        self.larm.send_goal(larm_goal)
        self.rarm.send_goal(rarm_goal)
        self.head.send_goal(head_goal)
        self.torso.send_goal(torso_goal)
        self.larm.wait_for_result()
        self.rarm.wait_for_result()
        self.head.wait_for_result()
        self.torso.wait_for_result()

    def goal_LArm(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("LARM_JOINT0")
        goal.trajectory.joint_names.append("LARM_JOINT1")
        goal.trajectory.joint_names.append("LARM_JOINT2")
        goal.trajectory.joint_names.append("LARM_JOINT3")
        goal.trajectory.joint_names.append("LARM_JOINT4")
        goal.trajectory.joint_names.append("LARM_JOINT5")
        return goal

    def goal_RArm(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("RARM_JOINT0")
        goal.trajectory.joint_names.append("RARM_JOINT1")
        goal.trajectory.joint_names.append("RARM_JOINT2")
        goal.trajectory.joint_names.append("RARM_JOINT3")
        goal.trajectory.joint_names.append("RARM_JOINT4")
        goal.trajectory.joint_names.append("RARM_JOINT5")
        return goal

    def goal_Torso(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("CHEST_JOINT0")
        return goal

    def goal_Head(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("HEAD_JOINT0")
        goal.trajectory.joint_names.append("HEAD_JOINT1")
        return goal

    def setup_Positions(self, goal, positions, tm = 1.0):
        for p in positions:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = [ x * math.pi / 180.0 for x in p]
            point.time_from_start = rospy.Duration(tm)
            goal.trajectory.points.append(point)
            tm += tm
        return goal

    def check_q_data(self, name):
        import math
        data = []

        name = name+".q"
        f = open(name, 'w')
        for j in self.joint_states:
            current_time = j.header.stamp.to_sec();
            current_value = [p*180/math.pi for p in j.position]
            data.append([current_time, current_value[5]])
            f.write(str(current_time)+' '+' '.join([str(i) for i in current_value])+"\n")
        f.close()
        self.filenames.append(name)
        return data

    def write_output_to_pdf (self,name):
        import os
        cmd = "gnuplot -p -e \"set terminal pdf; set output '"+name+"'; plot "
        for name in self.filenames:
            cmd += "'"+name+"' using 0:7 title '"+name+"' with lines"
            if name != self.filenames[-1]:
                cmd += ","
        cmd += "\""
        os.system(cmd)
        return cmd

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge', TestHiroROSBridge)




