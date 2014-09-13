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

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_client.cpp>
#include <testutil/abst_acceptancetest.hpp>
#include <testutil/acceptancetest_ros.hpp>

#include <sstream>

template<class T>
  std::string to_string(T a)
  {
    std::stringstream ss;
    ss << a;
    return ss.str();
  }

class AcceptanceTest_Hiro
{
  /*
   This class holds methods that can be used for verification of the robot
   Kawada Industries' dual-arm robot Hiro (and the same model of other robots
   e.g. NEXTAGE OPEN).

   This test class is:
   - Intended to be run as nosetests, ie. roscore isn't available by itself.
   - Almost all the testcases don't run without an access to a robot running.

   Above said, combined with a rostest that launches roscore and robot (either
   simulation or real) hopefully these testcases can be run, both in batch
   manner by rostest and in nosetest manner.

   Prefix for methods 'at' means 'acceptance test'.

   All time arguments are second.

   */
public:
  typedef std::vector<double> DVEC;
  typedef std::vector<DVEC> DMAT;

  static const double positions_larm_deg_up[6];
  static const double positions_larm_deg_down[6];
  static const double positions_rarm_deg_down[6];
  static const double positions_larm_deg_up_sync[6];
  static const double positions_rarm_deg_up_sync[6];
  static const double rotation_angles_head_1[2][2];
  static const double rotation_angles_head_2[2][2];
  static const double positions_torso_deg[2][1];
  static const double R_Z_SMALLINCREMENT;
  // WORKSPACE; x near, y far
  static const double pos_l_x_near_y_far[3];
  static const double rpy_l_x_near_y_far[3];
  static const double pos_r_x_near_y_far[3];
  static const double rpy_r_x_near_y_far[3];
  // workspace; x far, y far
  static const double pos_l_x_far_y_far[3];
  static const double rpy_l_x_far_y_far[3];
  static const double pos_r_x_far_y_far[3];
  static const double rpy_r_x_far_y_far[3];

  static const double TASK_DURATION_DEFAULT;
  static const double TASK_DURATION_TORSO;
  static const double DURATION_EACH_SMALLINCREMENT;
  static const double TASK_DURATION_HEAD;
  static const int SLEEP_TIME;
  static const int SLEEP_OVERWRITE;

  static const int DURATION_TOTAL_SMALLINCREMENT;  // second

  DVEC POSITIONS_LARM_DEG_UP;
  DVEC POSITIONS_LARM_DEG_DOWN;
  DVEC POSITIONS_RARM_DEG_DOWN;
  DVEC POSITIONS_LARM_DEG_UP_SYNC;
  DVEC POSITIONS_RARM_DEG_UP_SYNC;
  DMAT ROTATION_ANGLES_HEAD_1;
  DMAT ROTATION_ANGLES_HEAD_2;
  DMAT POSITIONS_TORSO_DEG;

  // WORKSPACE; X NEAR, Y FAR
  DVEC POS_L_X_NEAR_Y_FAR;
  DVEC RPY_L_X_NEAR_Y_FAR;
  DVEC POS_R_X_NEAR_Y_FAR;
  DVEC RPY_R_X_NEAR_Y_FAR;
  // WORKSPACE; X FAR, Y FAR
  DVEC POS_L_X_FAR_Y_FAR;
  DVEC RPY_L_X_FAR_Y_FAR;
  DVEC POS_R_X_FAR_Y_FAR;
  DVEC RPY_R_X_FAR_Y_FAR;

#define INIT_VEC(name, num) name, name+num
#define VEC_COPY(from, to, N1, N2) for (int i = 0; i < N1; ++i) for (int j = 0; j < N2; ++j)to[i][j] = from[i][j];
  AcceptanceTest_Hiro(std::string name = "Hironx(Robot)0", std::string url = "") :
      rtm_robotname(name), rtm_url(url), ros(ROS_Client()), acceptance_ros_client(ros),
      POSITIONS_LARM_DEG_UP(INIT_VEC(positions_larm_deg_up, 6)),
      POSITIONS_LARM_DEG_DOWN(INIT_VEC(positions_larm_deg_down, 6)),
      POSITIONS_RARM_DEG_DOWN(INIT_VEC(positions_rarm_deg_down, 6)),
      POSITIONS_LARM_DEG_UP_SYNC(INIT_VEC(positions_larm_deg_up_sync, 6)),
      POSITIONS_RARM_DEG_UP_SYNC(INIT_VEC(positions_rarm_deg_up_sync, 6)),

      ROTATION_ANGLES_HEAD_1(2, DVEC(2)), ROTATION_ANGLES_HEAD_2(2, DVEC(2)), POSITIONS_TORSO_DEG(2, DVEC(1)),

      // WORKSPACE, X NEAR, Y FAR
      POS_L_X_NEAR_Y_FAR(INIT_VEC(pos_l_x_near_y_far, 3)), RPY_L_X_NEAR_Y_FAR(INIT_VEC(rpy_l_x_near_y_far, 3)),
      POS_R_X_NEAR_Y_FAR(INIT_VEC(pos_r_x_near_y_far, 3)), RPY_R_X_NEAR_Y_FAR(INIT_VEC(rpy_r_x_near_y_far, 3)),
      // WORKSPACE, X FAR, Y FAR
      POS_L_X_FAR_Y_FAR(INIT_VEC(pos_l_x_far_y_far, 3)), RPY_L_X_FAR_Y_FAR(INIT_VEC(rpy_l_x_far_y_far, 3)),
      POS_R_X_FAR_Y_FAR(INIT_VEC(pos_r_x_far_y_far, 3)), RPY_R_X_FAR_Y_FAR(INIT_VEC(rpy_r_x_far_y_far, 3))
  {
    VEC_COPY(rotation_angles_head_1, ROTATION_ANGLES_HEAD_1, 2, 2)
    VEC_COPY(rotation_angles_head_2, ROTATION_ANGLES_HEAD_2, 2, 2)
    VEC_COPY(positions_torso_deg, POSITIONS_TORSO_DEG, 2, 1)
  }

  std::string retName()
  {
    return rtm_robotname;
  }

  void wait_input(std::string msg, bool do_wait_input = false)
  {
    /*
     @type msg: str
     @param msg: To be printed on prompt.
     @param do_wait_input: If True python commandline waits for any input
     to proceed.
     */
    if (!msg.empty())
      msg = "\n" + msg + "= ";
    if (do_wait_input)
    {
      // exception
    }
  }

  void run_test_ros(bool do_wait_input = false)
  {
    /*
     Run by ROS exactly the same actions that run_tests_rtm performs.
     @param do_wait_input: If True, the user will be asked to hit any key
     before each task to proceed.
     */
    ROS_INFO("********RUN_TEST_ROS************");
    run_tests(acceptance_ros_client, do_wait_input);
  }

  void run_tests(AbstAcceptanceTest& test_client, bool do_wait_input = false)
  {
    std::string msg_type_client = "";
    if (typeid(test_client) == typeid(AcceptanceTestRos))
      msg_type_client = "(ROS) ";
    // else if(typeid(test_client) == typeid(AcceptanceTestRTM))
    //     msg_type_client = "(RTM) ";
    std::string msg = msg_type_client;

    test_client.go_initpos();

    std::string msg_task = "TASK-1. Move each arm separately to the given pose by passing joint space.";
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    move_armbyarm_jointangles(test_client);

    msg_task =
        "TASK-1. Move each arm separately to the given pose by passing pose in hand space "
        "(i.e. orthogonal coordinate of eef).";
    ROS_INFO("%s", msg_task.c_str());
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    move_armbyarm_pose(test_client);

    msg_task = "TASK-2. Move both arms at the same time using relative distance and angle from the current pose.";
    ROS_INFO("%s", msg_task.c_str());
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    movearms_together(test_client);

    msg_task = "TASK-3. Move arm with very small increment (0.1mm/sec).\n\tRight hand 3 cm upward over 30 seconds.";
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    set_pose_relative(test_client);

    msg_task = "In the beginning you\"ll see the displacement of the previous task."
        "\nTASK-4. Move head using Joint angles in degree.";
    ROS_INFO("%s", msg_task.c_str());
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    move_head(test_client);

    msg_task = "TASK-5. Rotate torso to the left and right 130 degree.";
    ROS_INFO("%s", msg_task.c_str());
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    move_torso(test_client);

    msg_task = std::string("TASK-6. Overwrite ongoing action.\n\t6-1."
                           "While rotating torso toward left, it getscanceled and rotate toward "
                           "right.\n\t6-2. While lifting left hand, right hand also tries to reach "
			   "the same height that gets cancelled so that it stays lower than the left hand.");
    ROS_INFO("%s", msg_task.c_str());
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    overwrite_torso(test_client);
    // task6-2
    overwrite_arm(test_client);

    msg_task = "TASK-7. Show the capable workspace on front side of the robot.";
    msg = msg_type_client + msg_task;
    wait_input(msg, do_wait_input);
    show_workspace(test_client);
  }

  void move_armbyarm_jointangles(AbstAcceptanceTest& test_client)
  {
    /*
     @type test_client: hironx_ros_robotics.abst_acceptancetest.AbstAcceptanceTest
     */
    test_client.go_initpos(7.0);
    std::string arm = AbstAcceptanceTest::GRNAME_LEFT_ARM;
    test_client.set_joint_angles(arm, POSITIONS_LARM_DEG_UP, "Task1 " + to_string(arm));

    arm = AbstAcceptanceTest::GRNAME_RIGHT_ARM;
    test_client.set_joint_angles(arm, POSITIONS_RARM_DEG_DOWN, "Task1 " + to_string(arm));
    ros::Duration(SLEEP_TIME).sleep();
  }

  void move_armbyarm_pose(AbstAcceptanceTest& test_client)
  {
    ROS_INFO("** _move_armbyarm_pose isn\"t yet implemented");
  }

  void movearms_together(AbstAcceptanceTest& test_client)
  {
    // @type test_client: hironx_ros_robotics.abst_acceptancetest.AbstAcceptanceTest
    test_client.go_initpos();
    std::string arm = AbstAcceptanceTest::GRNAME_LEFT_ARM;
    test_client.set_joint_angles(arm, POSITIONS_LARM_DEG_UP_SYNC, to_string(arm), TASK_DURATION_DEFAULT, false);
    /*
     task2; Under current implementation, left arm
     always moves first, followed immediately by right arm")
     */
    arm = AbstAcceptanceTest::GRNAME_RIGHT_ARM;
    test_client.set_joint_angles(arm, POSITIONS_RARM_DEG_DOWN, to_string(arm), TASK_DURATION_DEFAULT, false);
    ros::Duration(SLEEP_TIME).sleep();
  }

  void set_pose_relative(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();
    double delta = R_Z_SMALLINCREMENT;
    double t = DURATION_EACH_SMALLINCREMENT;
    double t_total_sec = DURATION_TOTAL_SMALLINCREMENT;
    int total_increment = t_total_sec / t;
    std::string msg_1 = "Right arm is moving upward with " + to_string(delta) + "mm increment per " + to_string(t)
        + "s.";
    std::string msg_2 = " (Total " + to_string(delta * total_increment) + "cm over " + to_string(t_total_sec) + "s).";
    ROS_INFO("%s", (msg_1 + msg_2).c_str());
    for (int i = 0; i < total_increment; ++i)
    {
      std::string msg_eachloop = to_string(i);
      msg_eachloop += "th loop;";
      test_client.set_pose_relative(AbstAcceptanceTest::GRNAME_RIGHT_ARM, 0, 0, delta, 0, 0, 0, msg_eachloop, t, true);
    }
  }

  void move_head(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();

    for (std::vector<std::vector<double> >::iterator positions = ROTATION_ANGLES_HEAD_1.begin();
        positions != ROTATION_ANGLES_HEAD_1.end(); ++positions)
    {
      test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_HEAD, *positions, "(1);", TASK_DURATION_HEAD);
    }
    for (std::vector<std::vector<double> >::iterator positions = ROTATION_ANGLES_HEAD_2.begin();
        positions != ROTATION_ANGLES_HEAD_2.end(); ++positions)
    {
      test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_HEAD, *positions, "(2);", TASK_DURATION_HEAD);
    }
  }

  void move_torso(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();
    for (std::vector<std::vector<double> >::iterator positions = POSITIONS_TORSO_DEG.begin();
        positions != POSITIONS_TORSO_DEG.end(); ++positions)
    {
      test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_TORSO, *positions);
    }
  }

  void overwrite_torso(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();
    test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_TORSO, POSITIONS_TORSO_DEG[0], "(1)", TASK_DURATION_TORSO,
                                 false);

    ros::Duration(SLEEP_OVERWRITE).sleep();

    test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_TORSO, POSITIONS_TORSO_DEG[1], "(2)", TASK_DURATION_TORSO,
                                 true);

    ros::Duration(SLEEP_OVERWRITE).sleep();
  }

  void overwrite_arm(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();
    test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_LEFT_ARM, POSITIONS_LARM_DEG_UP_SYNC, "(1)",
                                 TASK_DURATION_DEFAULT, false);
    test_client.set_joint_angles(
        AbstAcceptanceTest::GRNAME_RIGHT_ARM,
        POSITIONS_RARM_DEG_UP_SYNC,
        std::string("(2) begins. Overwrite previous arm command.\n\t"
                    "In the beginning both arm starts to move to the\n\tsame height"
		    "but to the left arm interrupting\ncommand is sent and it goes downward."),
        TASK_DURATION_DEFAULT, false);

    ros::Duration(SLEEP_OVERWRITE).sleep();

    test_client.set_joint_angles(AbstAcceptanceTest::GRNAME_LEFT_ARM, POSITIONS_LARM_DEG_DOWN, "(3)",
                                 TASK_DURATION_DEFAULT, false);

    ros::Duration((int)TASK_DURATION_DEFAULT).sleep();
  }

  void show_workspace(AbstAcceptanceTest& test_client)
  {
    test_client.go_initpos();
    std::string msg = "; ";

    std::string larm = AbstAcceptanceTest::GRNAME_LEFT_ARM;
    std::string rarm = AbstAcceptanceTest::GRNAME_RIGHT_ARM;
    // X near, Y far.
    test_client.set_pose(larm, POS_L_X_NEAR_Y_FAR, RPY_L_X_NEAR_Y_FAR, msg + to_string(larm), TASK_DURATION_DEFAULT,
                         false);
    test_client.set_pose(rarm, POS_R_X_NEAR_Y_FAR, RPY_R_X_NEAR_Y_FAR, msg + to_string(rarm), TASK_DURATION_DEFAULT,
                         true);
    // X far, Y far.
    test_client.set_pose(larm, POS_L_X_FAR_Y_FAR, RPY_L_X_FAR_Y_FAR, msg + to_string(larm), TASK_DURATION_DEFAULT,
                         false);
    test_client.set_pose(rarm, POS_R_X_FAR_Y_FAR, RPY_R_X_FAR_Y_FAR, msg + to_string(rarm), TASK_DURATION_DEFAULT,
                         true);
  }

private:
  const std::string rtm_robotname;
  const std::string rtm_url;

  ROS_Client ros;
  AcceptanceTestRos acceptance_ros_client;
};

// init parameters
const double AcceptanceTest_Hiro::positions_larm_deg_up[6] = {-4.697, -2.012, -117.108, -17.180, 29.146, -3.739};
const double AcceptanceTest_Hiro::positions_larm_deg_down[6] = {6.196, -5.311, -73.086, -15.287, -12.906, -2.957};
const double AcceptanceTest_Hiro::positions_rarm_deg_down[6] = {-4.949, -3.372, -80.050, 15.067, -7.734, 3.086};
const double AcceptanceTest_Hiro::positions_larm_deg_up_sync[6] = {-4.695, -2.009, -117.103, -17.178, 29.138, -3.738};
const double AcceptanceTest_Hiro::positions_rarm_deg_up_sync[6] = {4.695, -2.009, -117.103, 17.178, 29.138, 3.738};
const double AcceptanceTest_Hiro::rotation_angles_head_1[2][2] = { {0.1, 0.0}, {50.0, 10.0}};
const double AcceptanceTest_Hiro::rotation_angles_head_2[2][2] = { {-50, -10}, {0, 0}};
const double AcceptanceTest_Hiro::positions_torso_deg[2][1] = { {130.0}, {-130.0}};

const double AcceptanceTest_Hiro::R_Z_SMALLINCREMENT = 0.0001;
// Workspace; X near, Y far
const double AcceptanceTest_Hiro::pos_l_x_near_y_far[3] = {0.326, 0.474, 1.038};
const double AcceptanceTest_Hiro::rpy_l_x_near_y_far[3] = {-3.075, -1.569, 3.074};
const double AcceptanceTest_Hiro::pos_r_x_near_y_far[3] = {0.326, -0.472, 1.048};
const double AcceptanceTest_Hiro::rpy_r_x_near_y_far[3] = {3.073, -1.569, -3.072};
// workspace; x far, y far
const double AcceptanceTest_Hiro::pos_l_x_far_y_far[3] = {0.47548142379781055, 0.17430276793604782, 1.0376878025614884};
const double AcceptanceTest_Hiro::rpy_l_x_far_y_far[3] = {-3.075954857224205, -1.5690261926181046, 3.0757659493049574};
const double AcceptanceTest_Hiro::pos_r_x_far_y_far[3] = {0.4755337947019357, -0.17242322190721648, 1.0476395479774052};
const double AcceptanceTest_Hiro::rpy_r_x_far_y_far[3] = {3.0715850722714944, -1.5690204449882248, -3.071395243174742};

const double AcceptanceTest_Hiro::TASK_DURATION_DEFAULT = 4.0;
const double AcceptanceTest_Hiro::DURATION_EACH_SMALLINCREMENT = 0.1;
const double AcceptanceTest_Hiro::TASK_DURATION_TORSO = AcceptanceTest_Hiro::TASK_DURATION_DEFAULT;
const double AcceptanceTest_Hiro::TASK_DURATION_HEAD = AcceptanceTest_Hiro::TASK_DURATION_DEFAULT;
const int AcceptanceTest_Hiro::SLEEP_TIME = 2;
const int AcceptanceTest_Hiro::SLEEP_OVERWRITE = 2;

const int AcceptanceTest_Hiro::DURATION_TOTAL_SMALLINCREMENT = 30;  // Second.
int main(int argc, char* argv[])
{
  // Init the ROS node
  ros::init(argc, argv, "hironx_ros_client");

  AcceptanceTest_Hiro test;
  test.run_test_ros();
  return 0;
}
