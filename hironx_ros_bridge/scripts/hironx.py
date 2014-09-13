#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, JSK Lab, University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of JSK Lab, University of Tokyo. nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
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


try:  # catkin does not requires load_manifest
    import hironx_ros_bridge
except:
    import roslib
    roslib.load_manifest('hironx_ros_bridge')

from hironx_ros_bridge import hironx_client
from hironx_ros_bridge.ros_client import ROS_Client

# See 'https://github.com/tork-a/rtmros_nextage/commit/' +
#     'd4268d81ec14a514bb4b3b52614c81e708dd1ecc#' +_
#     'diff-20257dd6ad60c0892cfb122c37a8f2ba'
from hrpsys import rtm
import argparse

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
        args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
    if not args.modelfile:
        args.modelfile = ""

    # support old style format
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
    robot = hiro = hironx_client.HIRONX()
    robot.init(robotname=args.robot, url=args.modelfile)

    # ROS Client
    ros = ROS_Client()

# for simulated robot
# $ ./hironx.py
#
# for real robot
# ../script/hironx.py -- --host hiro014
# ./ipython -i hironx.py -- --host hiro014
# for real robot with custom model file
# ../script/hironx.py -- --host hiro014 --modelfile /opt/jsk/etc/HIRONX/model/main.wrl
#
# See http://unix.stackexchange.com/questions/11376/what-does-double-dash-mean
# for the use of double-dash on unix/linux.
