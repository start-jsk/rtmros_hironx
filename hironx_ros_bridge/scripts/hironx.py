#!/usr/bin/env python
# -*- coding: utf-8 -*-

try: # catkin does not requires load_manifest
    import roslib
    import hironx_ros_bridge
except:
    import roslib; roslib.load_manifest('hironx_ros_bridge')

from hironx_ros_bridge import hironx_client

from hrpsys import rtm  # See https://github.com/tork-a/rtmros_nextage/commit/d4268d81ec14a514bb4b3b52614c81e708dd1ecc#diff-20257dd6ad60c0892cfb122c37a8f2ba 
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