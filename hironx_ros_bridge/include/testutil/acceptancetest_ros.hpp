#ifndef ACCEPTANCE_TEST_HPP
#define ACCEPTANCE_TEST_HPP

#include <ros/ros.h>
#include <testutil/abst_acceptancetest.hpp>
#include <ros_client.cpp>

#include <vector>

class AcceptanceTestRos : public AbstAcceptanceTest{
public:
  AcceptanceTestRos(){};
  AcceptanceTestRos(ROS_Client rclient)
    : AbstAcceptanceTest(rclient)
  {
    robot_client.init_action_clients();
  }
  ~AcceptanceTestRos(){}

  void go_initpos(double dtaskduration = 7.0){
    robot_client.go_init(dtaskduration);
  }

  void set_joint_angles(std::string joint_group,
                        std::vector<double>  joint_angles,
                        std::string msg_tasktitle = "",
                        double task_duration = 7.0,
                        bool do_wait = true
                        ){
    /*
      Move by passing joint angles of an arm.
      @type joint_group: str
      @type joint_angles: [double]
      @type msg_tasktitle: str
      @type task_duration: double
    */
    ROS_INFO("'''%s'''",msg_tasktitle.c_str());
    robot_client.set_joint_angles_deg(joint_group,joint_angles,task_duration,do_wait);
  }

  void set_pose(std::string joint_group,
                std::vector<double> pose,
                std::vector<double> rpy,
                std::string msg_tasktitle="",
                double task_duration=7.0,
                bool do_wait=true,
                std::string ref_frame_name=""){
    ROS_INFO("This method has not been implemented yet.");
  }

  void set_pose_relative(std::string joint_group,
                         int dx=0, int dy=0,int dz=0, int dr=0,int dp=0,int dw=0,
                         std::string msg_tasktitle="",
                         double task_duration=7.0,
                         bool do_wait=true){
    ROS_INFO("This method has not been implemented yet.");
  }

};
#endif
