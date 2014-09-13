/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, TORK (Tokyo Opensource Robotics Kyokai Association)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of JSK Lab, University of Tokyo. nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>
#include <cmath>

class ROS_Client
{
  /*
   This class:

   - holds methods that are specific to Kawada Industries' dual-arm
   robot called Hiro, via ROS.
   - As of July 2014, this class is only intended to be used through HIRONX
   class.
   */

  // TODO(@iory): Address the following concern.
  // This duplicates "Group" definition in HIRONX, which is bad.
  // Need to consider consolidating the definition either in hironx_ros_bridge
  // or somewhere in the upstream, e.g.:
  // https://github.com/fkanehiro/hrpsys-base/pull/253
public:
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

  pr2_controllers_msgs::JointTrajectoryGoal goal;

  ROS_Client() :
      GR_TORSO("torso"), GR_HEAD("head"), GR_LARM("larm"), GR_RARM("rarm"), MSG_ASK_ISSUEREPORT(
          "Your report to https://github.com/start-jsk/rtmros_hironx/issues "
          "about the issue you are seeing is appreciated.")
  {
  }

  explicit ROS_Client(std::vector<std::string> joingroups) :
      MSG_ASK_ISSUEREPORT(
          "Your report to https://github.com/start-jsk/rtmros_hironx/issues "
          "about the issue you are seeing is appreciated.")
  {
    set_groupnames(joingroups);
  }

  ROS_Client(const ROS_Client& obj)
  {
    GR_TORSO = obj.GR_TORSO;
    GR_HEAD = obj.GR_HEAD;
    GR_LARM = obj.GR_LARM;
    GR_RARM = obj.GR_RARM;
  }

  ROS_Client& operator=(const ROS_Client& obj)
  {
    GR_TORSO = obj.GR_TORSO;
    GR_HEAD = obj.GR_HEAD;
    GR_LARM = obj.GR_LARM;
    GR_RARM = obj.GR_RARM;
    return *this;
  }

  ~ROS_Client()
  {
  }

  void init_action_clients()
  {
    static TrajClient aclient_larm("/larm_controller/joint_trajectory_action", true);
    static TrajClient aclient_rarm("/rarm_controller/joint_trajectory_action", true);
    static TrajClient aclient_head("/head_controller/joint_trajectory_action", true);
    static TrajClient aclient_torso("/torso_controller/joint_trajectory_action", true);

    aclient_larm.waitForServer();
    ROS_INFO("ros_client; 1");
    goal_larm = pr2_controllers_msgs::JointTrajectoryGoal();
    ROS_INFO("ros_client; 2");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT0");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT1");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT2");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT3");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT4");
    goal_larm.trajectory.joint_names.push_back("LARM_JOINT5");
    ROS_INFO("ros_client; 3");

    aclient_rarm.waitForServer();
    goal_rarm = pr2_controllers_msgs::JointTrajectoryGoal();
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT0");
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT1");
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT2");
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT3");
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT4");
    goal_rarm.trajectory.joint_names.push_back("RARM_JOINT5");

    aclient_head.waitForServer();
    goal_head = pr2_controllers_msgs::JointTrajectoryGoal();
    goal_head.trajectory.joint_names.push_back("HEAD_JOINT0");
    goal_head.trajectory.joint_names.push_back("HEAD_JOINT1");

    aclient_torso.waitForServer();
    goal_torso = pr2_controllers_msgs::JointTrajectoryGoal();
    goal_torso.trajectory.joint_names.push_back("CHEST_JOINT0");

    // Display Joint names
    ROS_INFO("\nJoint names;");
    std::cout << "Torso: [";
    for (std::vector<std::string>::iterator name = goal_torso.trajectory.joint_names.begin();
        name != goal_torso.trajectory.joint_names.end(); ++name)
      std::cout << *name << " ";
    std::cout << "]\nHead: [";
    for (std::vector<std::string>::iterator name = goal_head.trajectory.joint_names.begin();
        name != goal_head.trajectory.joint_names.end(); ++name)
      std::cout << *name << " ";
    std::cout << "]\nL-Arm: [";
    for (std::vector<std::string>::iterator name = goal_larm.trajectory.joint_names.begin();
        name != goal_larm.trajectory.joint_names.end(); ++name)
      std::cout << *name << " ";
    std::cout << "]\nR-Arm: [";
    for (std::vector<std::string>::iterator name = goal_rarm.trajectory.joint_names.begin();
        name != goal_rarm.trajectory.joint_names.end(); ++name)
      std::cout << *name << " ";
    std::cout << "]" << std::endl;
  }

  // Init positions are taken from HIRONX.
  // TODO(@iory): Need to add factory position too that's so convenient when
  // working with the manufacturer.
  void go_init(double task_duration = 7.0)
  {
    ROS_INFO("*** go_init begins ***");
    std::vector<double> POSITIONS_TORSO_DEG(1, 0.0);
    set_joint_angles_deg(GR_TORSO, POSITIONS_TORSO_DEG, task_duration);

    std::vector<double> POSITIONS_HEAD_DEG(2, 0.0);
    set_joint_angles_deg(GR_HEAD, POSITIONS_HEAD_DEG, task_duration);

    std::vector<double> POSITIONS_LARM_DEG(6);
    POSITIONS_LARM_DEG[0] = 0.6;
    POSITIONS_LARM_DEG[1] = 0.0;
    POSITIONS_LARM_DEG[2] = -100;
    POSITIONS_LARM_DEG[3] = -15.2;
    POSITIONS_LARM_DEG[4] = 9.4;
    POSITIONS_LARM_DEG[5] = -3.2;
    set_joint_angles_deg(GR_LARM, POSITIONS_LARM_DEG, task_duration);

    std::vector<double> POSITIONS_RARM_DEG(6);
    POSITIONS_RARM_DEG[0] = -0.6;
    POSITIONS_RARM_DEG[1] = 0;
    POSITIONS_RARM_DEG[2] = -100;
    POSITIONS_RARM_DEG[3] = 15.2;
    POSITIONS_RARM_DEG[4] = 9.4;
    POSITIONS_RARM_DEG[5] = 3.2;
    set_joint_angles_deg(GR_RARM, POSITIONS_RARM_DEG, task_duration, true);

    ROS_INFO("*** go_init_ends ***");
  }

  // This method takes the angles(radian) and
  // changes the angle of the joints of the robot.
  void set_joint_angles_rad(std::string groupname, std::vector<double> positions_radian, double duration = 7.0,
                            bool wait = false)
  {
    TrajClient* action_client;

    if (groupname == GR_TORSO)
    {
      action_client = new TrajClient("torso_controller/joint_trajectory_action", true);
      goal = goal_torso;
    }
    else if (groupname == GR_HEAD)
    {
      action_client = new TrajClient("head_controller/joint_trajectory_action", true);
      goal = goal_head;
    }
    else if (groupname == GR_LARM)
    {
      action_client = new TrajClient("larm_controller/joint_trajectory_action", true);
      goal = goal_larm;
    }
    else if (groupname == GR_RARM)
    {
      action_client = new TrajClient("rarm_controller/joint_trajectory_action", true);
      goal = goal_rarm;
    }

    try
    {
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions = positions_radian;
      pt.time_from_start = ros::Duration(duration);
      goal.trajectory.points.clear();
      goal.trajectory.points.push_back(pt);
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(duration);
      action_client->sendGoal(goal);
      if (wait)
      {
        ROS_INFO("waiting......");
        ROS_INFO("wait_for_result for the joint group %s = %d", groupname.c_str(), action_client->waitForResult());
      }
    }
    catch (const ros::Exception& e)
    {
      ROS_INFO("%s", e.what());
    }
    catch (...)
    {
      ROS_INFO("Exception");
    }
    delete action_client;
  }

  //  groupname: This should exist in groupnames.
  void set_joint_angles_deg(std::string groupname, std::vector<double> positions_deg, double duration = 7.0, bool wait =
                                false)
  {
    set_joint_angles_rad(groupname, to_rad_list(positions_deg), duration, wait);
  }

private:
  std::string GR_TORSO;
  std::string GR_HEAD;
  std::string GR_LARM;
  std::string GR_RARM;

  const std::string MSG_ASK_ISSUEREPORT;

  pr2_controllers_msgs::JointTrajectoryGoal goal_larm;
  pr2_controllers_msgs::JointTrajectoryGoal goal_rarm;
  pr2_controllers_msgs::JointTrajectoryGoal goal_head;
  pr2_controllers_msgs::JointTrajectoryGoal goal_torso;

  /*
   param groupnames: List of the joint group names. Assumes to be in the
   following order:
   torso, head, right arm, left arm.
   This current setting is derived from the order of
   Groups argument in HIRONX class. If other groups
   need to be defined in the future, this method may
   need to be modified.
   */
  void set_groupnames(std::vector<std::string> groupnames)
  {
    // output group name
    ROS_INFO("set_groupnames");
    for (std::vector<std::string>::iterator name = groupnames.begin(); name != groupnames.end(); ++name)
      std::cout << *name << " ";
    std::cout << std::endl;

    GR_TORSO = groupnames[0];
    GR_HEAD = groupnames[1];
    GR_RARM = groupnames[2];
    GR_LARM = groupnames[3];
  }

  // This method change degree to radian.
  // param list_degree: A list length of the number of joints.
  std::vector<double> to_rad_list(std::vector<double> list_degree)
  {
    std::vector<double> list_rad;
    list_rad.reserve(list_degree.size());
    for (std::vector<double>::iterator deg = list_degree.begin(); deg != list_degree.end(); ++deg)
    {
      double rad = *deg * M_PI / 180.0;
      list_rad.push_back(rad);
      ROS_INFO("Position deg=%lf rad=%lf", *deg, rad);
    }
    return list_rad;
  }
};

#endif
