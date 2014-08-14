#ifndef ABST_ACCEPTANCETEST_HPP
#define ABST_ACCEPTANCETEST_HPP

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_client.cpp>

class AbstAcceptanceTest{
public:
  const std::string MSG_ERR_NOTIMPLEMENTED;

  double default_task_duration;

  static std::string GRNAME_LEFT_ARM;
  static std::string GRNAME_RIGHT_ARM;
  static std::string GRNAME_TORSO;
  static std::string GRNAME_HEAD;

  ROS_Client robot_client;

  AbstAcceptanceTest(){};
  AbstAcceptanceTest(ROS_Client rclient)
    :default_task_duration(7.0),
     MSG_ERR_NOTIMPLEMENTED("The method is not implemented in the derived class"),
     robot_client(rclient)
  {
    /*
      @type robot_client: hironx_ros_bridge.ros_client.ROS_Client or
      hrpsys.hrpsys_config.HrpsysConfigurator
    */
  };

  AbstAcceptanceTest(ROS_Client rclient, double default_task_duration)
    :default_task_duration(default_task_duration),
     MSG_ERR_NOTIMPLEMENTED("The method is not implemented in the derived class"),
     robot_client(rclient)
  {
    /*
      @type robot_client: hironx_ros_bridge.ros_client.ROS_Client or
      hrpsys.hrpsys_config.HrpsysConfigurator
    */
  };

  virtual ~AbstAcceptanceTest(){};

  virtual void go_initpos(double dtaskduration=7.0){
    //throw NotImplementedError(MSG_ERR_NOTIMPLEMENTED);
  }

  virtual void set_joint_angles(std::string joint_group,
                                std::vector<double> joint_angles,
                                std::string msg_tasktitle="",
                                double task_duration=7.0,
                                bool do_wait=true)
  {
    /*
      Move by passing joint angles of an arm.
      @type joint_group: str
      @type joint_angles: [double]
      @type msg_tasktitle: str
      @type task_duration: double
    */
    //throw NotImplementedError(MSG_ERR_NOTIMPLEMENTED);
  }

  virtual void set_pose(std::string joint_group,
                        std::vector<double> pose,
                        std::vector<double> rpy,
                        std::string msg_tasktitle="",
                        double task_duration=7.0,
                        bool do_wait=true,
                        std::string ref_frame_name=""){
    //throw NotImplementedError(MSG_ERR_NOTIMPLEMENTED);
  }

  virtual void set_pose_relative(std::string joint_group,
                                 int dx=0, int dy=0,int dz=0, int dr=0,int dp=0,int dw=0,
                                 std::string msg_tasktitle="",
                                 double task_duration=7.0,
                                 bool do_wait=true){
    //throw NotImplementedError(MSG_ERR_NOTIMPLEMENTED);
  }

};

std::string AbstAcceptanceTest::GRNAME_LEFT_ARM = "larm";
std::string AbstAcceptanceTest::GRNAME_RIGHT_ARM = "rarm";
std::string AbstAcceptanceTest::GRNAME_TORSO = "torso";
std::string AbstAcceptanceTest::GRNAME_HEAD  = "head";


#endif
