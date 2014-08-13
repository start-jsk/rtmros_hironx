# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, TORK (Tokyo Opensource Robotics Kyokai Association)
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
#  * Neither the name of TORK. nor the
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

import time

from hironx_ros_bridge.constant import Constant
from hironx_ros_bridge.hironx_client import HIRONX
from hironx_ros_bridge.ros_client import ROS_Client
from hironx_ros_bridge.testutil.abst_acceptancetest import AbstAcceptanceTest
from hironx_ros_bridge.testutil.acceptancetest_ros import AcceptanceTestROS
from hironx_ros_bridge.testutil.acceptancetest_rtm import AcceptanceTestRTM


class AcceptanceTest_Hiro():
    '''
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
    '''

    _POSITIONS_LARM_DEG_UP = [-4.697, -2.012, -117.108, -17.180, 29.146, -3.739]
    _POSITIONS_LARM_DEG_DOWN = [6.196, -5.311, -73.086, -15.287, -12.906, -2.957]
    _POSITIONS_RARM_DEG_DOWN = [-4.949, -3.372, -80.050, 15.067, -7.734, 3.086]
    _POSITIONS_LARM_DEG_UP_SYNC = [-4.695, -2.009, -117.103, -17.178, 29.138, -3.738]
    _POSITIONS_RARM_DEG_UP_SYNC = [4.695, -2.009, -117.103, 17.178, 29.138, 3.738]
    _ROTATION_ANGLES_HEAD_1 = [[0.1, 0.0], [50.0, 10.0]]
    _ROTATION_ANGLES_HEAD_2 = [[50, 10], [-50, -10], [0, 0]]
    _POSITIONS_TORSO_DEG = [[130.0], [-130.0]]
    _R_Z_SMALLINCREMENT = 0.0001
    # Workspace; X near, Y far
    _POS_L_X_NEAR_Y_FAR = [0.326, 0.474, 1.038]
    _RPY_L_X_NEAR_Y_FAR = (-3.075, -1.569, 3.074)
    _POS_R_X_NEAR_Y_FAR = [0.326, -0.472, 1.048]
    _RPY_R_X_NEAR_Y_FAR = (3.073, -1.569, -3.072)
    # Workspace; X far, Y far
    _POS_L_X_FAR_Y_FAR = [0.47548142379781055, 0.17430276793604782, 1.0376878025614884]
    _RPY_L_X_FAR_Y_FAR = (-3.075954857224205, -1.5690261926181046, 3.0757659493049574)
    _POS_R_X_FAR_Y_FAR = [0.4755337947019357, -0.17242322190721648, 1.0476395479774052]
    _RPY_R_X_FAR_Y_FAR = (3.0715850722714944, -1.5690204449882248, -3.071395243174742)

    _TASK_DURATION_DEFAULT = 5.0
    _DURATION_EACH_SMALLINCREMENT = 0.1
    _TASK_DURATION_TORSO = 4.0
    _TASK_DURATION_HEAD = 2.5

    _DURATON_TOTAL_SMALLINCREMENT = 30  # Second.

    def __init__(self, rtm_robotname='HiroNX(Robot)0', url=''):
        '''
        Initiate both ROS and RTM robot clients, keep them public so that they
        are callable from script. e.g. On ipython,

                In [1]: acceptance.ros.go_init()
                In [2]: acceptance.rtm.goOffPose()
        '''
        self._rtm_robotname = rtm_robotname
        self._rtm_url = url

        # Init RTM and ROS client.
        self.ros = ROS_Client()
        self._acceptance_ros_client = AcceptanceTestROS(self.ros)
        self.rtm = HIRONX()
        self.rtm.init(robotname=self._rtm_robotname, url=self._rtm_url)
        self._acceptance_rtm_client = AcceptanceTestRTM(self.rtm)

    def _wait_input(self, msg, do_wait_input=False):
        '''
        @type msg: str
        @param msg: To be printed on prompt.
        @param do_wait_input: If True python commandline waits for any input
                              to proceed.
        '''
        if msg:
            msg = '\n' + msg + '= '
        if do_wait_input:
            try:
                input('Waiting for any key. **Do NOT** hit enter multiple ' +
                      'times. ' + msg)
            except NameError:
                # On python < 3, `input` can cause errors like following with
                # any input string. So here just catch-release it.
                # e.g.
                #   NameError: name 'next' is not defined
                pass

    def run_tests_rtm(self, do_wait_input=False):
        '''
        @param do_wait_input: If True, the user will be asked to hit any key
                              before each task to proceed.
        '''
        self._run_tests(self._acceptance_rtm_client, do_wait_input)

    def run_tests_ros(self, do_wait_input=False):
        '''
        Run by ROS exactly the same actions that run_tests_rtm performs.

        @param do_wait_input: If True, the user will be asked to hit any key
                              before each task to proceed.
        '''
        self._run_tests(self._acceptance_ros_client, do_wait_input)

    def _run_tests(self, test_client, do_wait_input=False):
        '''
        @type test_client: hironx_ros_robotics.abst_acceptancetest.AbstAcceptanceTest
        '''

        _task_msgs = ['TASK-1-1. Move each arm separately to the given pose by passing joint space.',
                      'TASK-1-2. Move each arm separately to the given pose by specifying a pose.',
                      'TASK-2. Move both arms at the same time using relative distance and angle from the current pose.',
                      'TASK-3. Move arm with very small increment (0.1mm/sec).\n\tRight hand 3 cm upward over 30 seconds.',
                      'In the beginning you\'ll see the displacement of the previous task.' +
                       '\nTASK-4. Move head using Joint angles in degree.',
                      'TASK-5. Rotate torso to the left and right 130 degree.',
                      'TASK-6. Overwrite ongoing action.' +
                      '\n\t6-1. While rotating torso toward left, it gets canceled and rotate toward right.' +
                      '\n\t6-2. While lifting left hand, right hand also tries to reach the same height that gets cancelled so that it stays lower than the left hand.',
                      'TASK-7. Show the capable workspace on front side of the robot.']

        _msg_type_client = None
        if isinstance(test_client, AcceptanceTestROS):
            _msg_type_client = '(ROS) '
        elif isinstance(test_client, AcceptanceTestRTM):
            _msg_type_client = '(RTM) '

        test_client.go_initpos()

        msg_task = _task_msgs[0]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._move_armbyarm_jointangles(test_client)

        msg_task = _task_msgs[1]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._move_armbyarm_pose(test_client)

        msg_task = _task_msgs[2]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._movearms_together(test_client)

        msg_task = _task_msgs[3]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._set_pose_relative(test_client)

        msg_task = _task_msgs[4]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._move_head(test_client)

        msg_task = _task_msgs[5]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._move_torso(test_client)

        msg_task = _task_msgs[6]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._overwrite_torso(test_client)
        self._overwrite_arm(test_client)

        msg_task = _task_msgs[7]
        msg = _msg_type_client + msg_task
        self._wait_input(msg, do_wait_input)
        self._show_workspace(test_client)

    def _move_armbyarm_jointangles(self, test_client):
        '''
        @type test_client: hironx_ros_robotics.abst_acceptancetest.AbstAcceptanceTest  
        '''
        test_client.go_initpos()
        arm = Constant.GRNAME_LEFT_ARM
        test_client.set_joint_angles(arm, self._POSITIONS_LARM_DEG_UP,
                                     'Task1 {}'.format(arm))

        arm = Constant.GRNAME_RIGHT_ARM
        test_client.set_joint_angles(arm, self._POSITIONS_RARM_DEG_DOWN,
                                     'Task1 {}'.format(arm))
        time.sleep(2.0)

    def _move_armbyarm_pose(self, test_client):
        print('** _move_armbyarm_pose isn\'t yet implemented')
        pass

    def _movearms_together(self, test_client):
        '''
        @type test_client: hironx_ros_robotics.abst_acceptancetest.AbstAcceptanceTest  
        '''
        test_client.go_initpos()
        arm = Constant.GRNAME_LEFT_ARM
        test_client.set_joint_angles(
                 arm, self._POSITIONS_LARM_DEG_UP_SYNC, '{}'.format(arm),
                 self._TASK_DURATION_DEFAULT, False)
                #'task2; Under current implementation, left arm ' +
                #'always moves first, followed immediately by right arm')
        arm = Constant.GRNAME_RIGHT_ARM
        test_client.set_joint_angles(
                    arm, self._POSITIONS_RARM_DEG_DOWN, '{}'.format(arm),
                    self._TASK_DURATION_DEFAULT, False)
        time.sleep(2.0)

    def _set_pose_relative(self, test_client):
        test_client.go_initpos()

        delta = self._R_Z_SMALLINCREMENT
        t = self._DURATION_EACH_SMALLINCREMENT
        t_total_sec = self._DURATON_TOTAL_SMALLINCREMENT
        total_increment = int(t_total_sec / t)
        msg_1 = ('Right arm is moving upward with' +
                 '{}mm increment per {}s'.format(delta, t))
        msg_2 = ' (Total {}cm over {}s).'.format(delta * total_increment, t_total_sec)
        print(msg_1 + msg_2)
        for i in range(total_increment):
            msg_eachloop = '{}th loop;'.format(i)
            test_client.set_pose_relative(
                     Constant.GRNAME_RIGHT_ARM, dz=delta,
                     msg_tasktitle=msg_eachloop, task_duration=t, do_wait=True)

    def _move_head(self, test_client):
        test_client.go_initpos()

        for positions in self._ROTATION_ANGLES_HEAD_1:
            test_client.set_joint_angles(
                               Constant.GRNAME_HEAD,
                               positions, '(1);', self._TASK_DURATION_HEAD)

        for positions in self._ROTATION_ANGLES_HEAD_2:
            test_client.set_joint_angles(
                                     Constant.GRNAME_HEAD, positions,
                                     '(2);', self._TASK_DURATION_HEAD)

    def _move_torso(self, test_client):
        test_client.go_initpos()
        for positions in self._POSITIONS_TORSO_DEG:
            test_client.set_joint_angles(Constant.GRNAME_TORSO,
                                         positions, '')

    def _overwrite_torso(self, test_client):
        test_client.go_initpos()
        test_client.set_joint_angles(
                        Constant.GRNAME_TORSO, self._POSITIONS_TORSO_DEG[0],
                        '(1)', self._TASK_DURATION_TORSO, False)
        time.sleep(2.0)
        test_client.set_joint_angles(
                          Constant.GRNAME_TORSO, self._POSITIONS_TORSO_DEG[1],
                          '(2)', self._TASK_DURATION_TORSO, True)
        time.sleep(2.0)

    def _overwrite_arm(self, test_client):
        test_client.go_initpos()
        test_client.set_joint_angles(
                    Constant.GRNAME_LEFT_ARM, self._POSITIONS_LARM_DEG_UP_SYNC,
                    '(1)', self._TASK_DURATION_DEFAULT, False)
        test_client.set_joint_angles(
                   Constant.GRNAME_RIGHT_ARM, self._POSITIONS_RARM_DEG_UP_SYNC,
                   '(2) begins. Overwrite previous arm command.' +
                   '\n\tIn the beginning both arm starts to move to the' +
                   '\n\tsame height, but to the left arm interrupting' +
                   '\ncommand is sent and it goes downward.',
                   self._TASK_DURATION_DEFAULT, False)
        time.sleep(2.0)
        test_client.set_joint_angles(
                      Constant.GRNAME_LEFT_ARM, self._POSITIONS_LARM_DEG_DOWN,
                      '(3)', self._TASK_DURATION_DEFAULT, False)

    def _show_workspace(self, test_client):
        test_client.go_initpos()
        msg = '; '

        larm = Constant.GRNAME_LEFT_ARM
        rarm = Constant.GRNAME_RIGHT_ARM
        # X near, Y far.
        test_client.set_pose(
                   larm, self._POS_L_X_NEAR_Y_FAR, self._RPY_L_X_NEAR_Y_FAR,
                   msg + '{}'.format(larm), self._TASK_DURATION_DEFAULT, False)
        test_client.set_pose(
                   rarm, self._POS_R_X_NEAR_Y_FAR, self._RPY_R_X_NEAR_Y_FAR,
                   msg + '{}'.format(rarm), self._TASK_DURATION_DEFAULT, True)

        # X far, Y far.
        test_client.set_pose(
                   larm, self._POS_L_X_FAR_Y_FAR, self._RPY_L_X_FAR_Y_FAR,
                   msg + '{}'.format(larm), self._TASK_DURATION_DEFAULT, False)
        test_client.set_pose(
                    rarm, self._POS_R_X_FAR_Y_FAR, self._RPY_R_X_FAR_Y_FAR,
                    msg + '{}'.format(rarm), self._TASK_DURATION_DEFAULT, True)

import argparse

from hrpsys import rtm

if __name__ == '__main__':
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
        args.robot = 'RobotHardware0' if args.host else 'HiroNX(Robot)0'
    if not args.modelfile:
        args.modelfile = ''

    # support old style format
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
    acceptance = AcceptanceTest_Hiro()
