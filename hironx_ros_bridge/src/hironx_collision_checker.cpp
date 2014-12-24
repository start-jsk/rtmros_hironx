/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Daiki Maekawa and TORK (Tokyo Opensource Robotics Kyokai Association)
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

#include <vector>

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/controller_manager/controller_manager.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "collision_checker");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group_r("right_arm");
  moveit::planning_interface::MoveGroup group_l("left_arm");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  psm.startStateMonitor("joint_states");
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  while(!(psm.getStateMonitor()->waitForCurrentState(1.0)));
  robot_state::RobotStatePtr current_state = psm.getStateMonitor()->getCurrentState();
  robot_state::RobotState planning_state = planning_scene.getCurrentStateNonConst();
  double *pos = current_state->getVariablePositions();
  planning_state.setVariablePositions(pos);
  planning_scene.checkCollision(collision_request, collision_result, *current_state, acm);
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it){
    acm.setEntry(it->first.first, it->first.second, true);
    ROS_INFO("Contact between: %s and %s now allowed.", it->first.first.c_str(), it->first.second.c_str());
  }

  collision_detection::CollisionResult old_collision_result;
  geometry_msgs::PoseStamped pose_r = group_r.getCurrentPose();
  geometry_msgs::PoseStamped pose_l = group_l.getCurrentPose();

  while(ros::ok()){
    if(psm.getStateMonitor()->waitForCurrentState(1.0)){
      collision_result.clear();
      current_state = psm.getStateMonitor()->getCurrentState();
      planning_state = planning_scene.getCurrentStateNonConst();
      pos = current_state->getVariablePositions();
      planning_state.setVariablePositions(pos);
      planning_scene.checkCollision(collision_request, collision_result, *current_state, acm);
      ROS_INFO_STREAM("Test : Current state is "
                      << (collision_result.collision ? "in" : "not in")
                      << " self collision");

      for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it){
        ROS_INFO("Contact between: %s and %s",
                 it->first.first.c_str(),
                 it->first.second.c_str());
      }
      
      if(!old_collision_result.collision && collision_result.collision){
        group_r.clearPoseTargets();
        group_l.clearPoseTargets();
        
        //TODO: Stop Motion
        
        //group_r.setPoseTarget(group_r.getCurrentPose());
        //group_l.setPoseTarget(group_l.getCurrentPose());
      }

      old_collision_result = collision_result;
    }
  }

  return 0;
} 
