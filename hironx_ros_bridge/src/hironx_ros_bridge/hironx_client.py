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
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Lab, University of Tokyo. nor the
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

import math
import numpy
import os
import time
import socket
import sys

import roslib; roslib.load_manifest("hrpsys")
from hrpsys.hrpsys_config import *
import OpenHRP
import OpenRTM_aist
import OpenRTM_aist.RTM_IDL
import rtm
from waitInput import waitInputConfirm, waitInputSelect

SWITCH_ON = OpenHRP.RobotHardwareService.SWITCH_ON
SWITCH_OFF = OpenHRP.RobotHardwareService.SWITCH_OFF
_MSG_ASK_ISSUEREPORT = 'Your report to ' + \
                       'https://github.com/start-jsk/rtmros_hironx/issues ' + \
                       'about the issue you are seeing is appreciated.'


class HIRONX(HrpsysConfigurator):
    '''
    @see: <a href = "http://code.google.com/p/hrpsys-base/source/browse/trunk/python/hrpsys_config.py">HrpsysConfigurator</a>

    This class holds methods that are specific to Kawada Industries' dual-arm
    robot called Hiro.
    '''

    Groups = [['torso', ['CHEST_JOINT0']],
              ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
              ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2',
                        'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']],
              ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2',
                        'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]]

    '''
    For OffPose and _InitialPose, the angles of each joint are listed in the
    ordered as defined in Groups variable.'''
    OffPose = [[0], [0, 0],
                   [25, -139, -157, 45, 0, 0],
                   [-25, -139, -157, -45, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]
    # With this pose the EEFs level up the tabletop surface.
    _InitialPose = [[0], [0, 0],
                   [-0.6, 0, -100, 15.2, 9.4, 3.2],
                   [0.6, 0, -100, -15.2, 9.4, -3.2],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]
    # This pose sets joint angles at the factory initial pose. No danger, but
    # no real advange either for in normal usage.
    # See https://github.com/start-jsk/rtmros_hironx/issues/107
    _InitialPose_Factory = [[0], [0, 0],
                   [-0, 0, -130, 0, 0, 0],
                   [0, 0, -130, 0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]
    INITPOS_TYPE_EVEN = 0
    INITPOS_TYPE_FACTORY = 1

    HandGroups = {'rhand': [2, 3, 4, 5], 'lhand': [6, 7, 8, 9]}

    RtcList = []

    # servo controller (grasper)
    sc = None
    sc_svc = None

    def init(self, robotname="HiroNX(Robot)0", url=""):
        '''
        Calls init from its superclass, which tries to connect RTCManager,
        looks for ModelLoader, and starts necessary RTC components. Also runs
        config, logger.
        Also internally calls setSelfGroups().

        @type robotname: str
        @type url: str
        '''
        HrpsysConfigurator.init(self, robotname=robotname, url=url)
        self.setSelfGroups()

    def goOffPose(self, tm=7):
        '''
        Move arms to the predefined (as member variable) pose where robot can
        be safely turned off.

        @type tm: float
        @param tm: Second to complete.
        '''
        for i in range(len(self.Groups)):
            self.setJointAnglesOfGroup(self.Groups[i][0], self.OffPose[i], tm,
                                       wait=False)
        for i in range(len(self.Groups)):
            self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        self.servoOff(wait=False)

    def goInitial(self, tm=7, wait=True, init_pose_type=0):
        '''
        Move arms to the predefined (as member variable) "initialized" pose.

        @type tm: float
        @param tm: Second to complete.
        @type wait: bool
        @param wait: If true, SequencePlayer.waitInterpolationOfGroup gets run.
                     (TODO: Elaborate what this means...Even after having taken
                     a look at its source code I can't tell what it means.)
        @type init_pose_type: int
        @param init_pose_type: 0: default init pose (specified as _InitialPose)
                               1: factory init pose (specified as
                                  _InitialPose_Factory)
        '''
        if init_pose_type == self.INITPOS_TYPE_FACTORY:
            _INITPOSE = self._InitialPose_Factory
        else:
            _INITPOSE = self._InitialPose

        ret = True
        for i in range(len(self.Groups)):
            # radangles = [x/180.0*math.pi for x in self._InitialPose[i]]
            print self.configurator_name, 'self.setJointAnglesOfGroup(', \
                  self.Groups[i][0], ',', _INITPOSE[i], ', ', tm, \
                  ',wait=False)'
            ret &= self.setJointAnglesOfGroup(self.Groups[i][0],
                                              _INITPOSE[i],
                                              tm, wait=False)
        if wait:
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        return ret

    def getRTCList(self):
        '''
        @see: HrpsysConfigurator.getRTCList

        @rtype [[str]]
        @rerutrn List of available components. Each element consists of a list
                 of abbreviated and full names of the component.
        '''
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['el', "SoftErrorLimiter"],
            # ['co', "CollisionDetector"],
            ['sc', "ServoController"],
            ['log', "DataLogger"]
            ]

    # hand interface
    # effort: 1~100[%]
    # hiro.HandOpen("rhand", effort)
    # hiro.HandOpen()        # for both hand
    # hiro.HandClose("rhand", effort)
    # hiro.HandClose()       # for both hand
    #
    def HandOpen(self, hand=None, effort=None):
        '''
        Set the stretch between two fingers of the specified hand as
        hardcoded value (100mm), by internally calling self.setHandWidth.

        @type hand: str
        @param hand: Name of the hand joint group. In the default 
                     setting of HIRONX, hand joint groups are defined
                     in member 'HandGroups' where 'lhand' and 'rhand'
                     are added.
        @type effort: int
        '''
        self.setHandWidth(hand, 100, effort=effort)

    def HandClose(self, hand=None, effort=None):
        '''
        Close 2-finger hand, by internally calling self.setHandWidth 
        setting 0 width.

        @type hand: str
        @param hand: Name of the hand joint group. In the default 
                     setting of HIRONX, hand joint groups are defined
                     in member 'HandGroups' where 'lhand' and 'rhand'
                     are added.
        @type effort: int
        '''
        self.setHandWidth(hand, 0, effort=effort)

    def setHandJointAngles(self, hand, angles, tm=1):
        '''
        @type hand: str
        @param hand: Name of the hand joint group. In the default 
                     setting of HIRONX, hand joint groups are defined
                     in member 'HandGroups' where 'lhand' and 'rhand'
                     are added.
        @type angles: OpenHRP::ServoControllerService::dSequence.
        @param angles: List of (TODO: document). Elements are in degree.
        @param tm: Time to complete the task.
        '''
        self.sc_svc.setJointAnglesOfGroup(hand, angles, tm)

    def setHandEffort(self, effort=100):
        '''
        Set maximum torque for all existing hand components.
        @type effort: int
        '''

        for i in [v for vs in self.HandGroups.values() for v in vs]:  # flatten
            self.sc_svc.setMaxTorque(i, effort)

    def setHandWidth(self, hand, width, tm=1, effort=None):
        '''
        @type hand: str
        @param hand: Name of the hand joint group. In the default 
                     setting of HIRONX, hand joint groups are defined
                     in member 'HandGroups' where 'lhand' and 'rhand'
                     are added.
        @param width: Max=100.
        @param tm: Time to complete the move.
        @type effort: int
        @param effort: Passed to self.setHandEffort if set. Not set by default.
        '''
        if effort:
            self.setHandEffort(effort)
        if hand:
            self.setHandJointAngles(hand, self.hand_width2angles(width), tm)
        else:
            for h in self.HandGroups.keys():
                self.setHandJointAngles(h, self.hand_width2angles(width), tm)

    def moveHand(self, hand, av, tm=1):  # direction av: + for open, - for close
        '''
        Negate the angle value for {2, 3, 6, 7}th element in av.

        @type hand: str
        @param hand: Specifies hand. (TODO: List the possible values. Should be
                     listed in setHandJointAngles so just copy from its doc.)
        @type av: [int]
        @param av: angle of each joint of the specified arm
                  (TODO: need verified. Also what's the length of the list?)
        @param tm: Time in second to complete the work.
        '''
        for i in [2, 3, 6, 7]:  # do not change this line if servo is different, change HandGroups
            av[i] = -av[i]
        self.setHandJointAngles(hand, av, tm)

    def hand_width2angles(self, width):
        '''
        TODO: Needs documented what this method does.

        @type width: float
        @return: None if the given width is invalid.
        '''
        safetyMargin = 3
        l1, l2 = (41.9, 19)  # TODO: What are these variables?

        if width < 0.0 or width > (l1 + l2 - safetyMargin) * 2:
            return None

        xPos = width / 2.0 + safetyMargin
        a2Pos = xPos - l2
        a1radH = math.acos(a2Pos / l1)
        a1rad = math.pi / 2.0 - a1radH

        return a1rad, -a1rad, -a1rad, a1rad

    def setSelfGroups(self):
        '''
        Set to the hrpsys.SequencePlayer the groups of links and joints that
        are statically defined as member variables (Groups) within this class.

        That said, override Groups variable if you prefer link and joint
        groups set differently.
        '''
        #TODO: Accept groups variable. The name of the method sounds more
        #      natural if it accepts it.
        for item in self.Groups:
            self.seq_svc.addJointGroup(item[0], item[1])
        for k, v in self.HandGroups.iteritems():
            if self.sc_svc:
                self.sc_svc.addJointGroup(k, v)

    def getActualState(self):
        '''
        Returns the physical state of robot.

        @rtype: <a href = "http://hrpsys-base.googlecode.com/svn/doc/df/d17/structOpenHRP_1_1RobotHardwareService_1_1RobotState.html">OpenHRP::RobotHardwareService::RobotState</a>
        @return: Robot's hardware status object that contains the following
                 variables accessible: angle, command, torque, servoState,
                 force, rateGyro, accel, voltage, current. See the api doc
                 of the class for more detail. Each variable is accessible by
                 like this for example:

                     servostate= robot.getActualState().servoState
        '''
        #TODO: Handle AttributeError. Typically when RobotHardware is not found,
        #      AttributeError: 'NoneType' object has no attribute 'getStatus'

        return self.rh_svc.getStatus()

    def isCalibDone(self):
        '''
        Check whether joints have been calibrated.
        @rtype bool
        '''
        if self.simulation_mode:
            return True
        else:
            rstt = self.rh_svc.getStatus()
            for item in rstt.servoState:
                if not item[0] & 1:
                    return False
        return True

    def isServoOn(self, jname='any'):
        '''
        Check whether servo control has been turned on.
        @type jname: str
        @param jname: Name of a link (that can be obtained by "hiro.Groups"
                      as lists of groups).
        @rtype bool
        '''

        if self.simulation_mode:
            return True
        else:
            s_s = self.getActualState().servoState
            if jname.lower() == 'any' or jname.lower() == 'all':
                for s in s_s:
                    # print self.configurator_name, 's = ', s
                    if (s[0] & 2) == 0:
                        return False
            else:
                jid = eval('self.' + jname)
                print self.configurator_name, s_s[jid]
                if s_s[jid][0] & 1 == 0:
                    return False
            return True
        return False

    def flat2Groups(self, flatList):
        '''
        @type flatList: [int]
        @param flatList: single dimension list, with its length depends on
                         'Groups' variable defined within this class. Excessive
                         elements will be dropped (see example below in @return)

                         eg. If the number of joints of the robot is 15,
                             len(flatList) should be 15.
        @rtype: [[]]
        @return: 2-dimensional list that has the same format with
                 'Groups' variable.

                 eg.
                 ipython> flatlist = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
                                      100, 110, 120, 130, 140, 150]
                 ipython> robot.flat2Groups(flatlist)
                 [[0], [10, 20],
                  [30, 40, 50, 60, 70, 80],
                  [90, 100, 110, 120, 130, 140]]

        '''
        retList = []
        index = 0
        for group in self.Groups:
            joint_num = len(group[1])
            retList.append(flatList[index: index + joint_num])
            index += joint_num
        return retList

    def servoOn(self, jname='all', destroy=1, tm=3):
        '''
        Turn on servo motors at joint specified.
        Joints need to be calibrated (otherwise error returns).

        @type jname: str
        @param jname: The value 'all' works iteratively for all servos.
        @param destroy: Not used.
        @type tm: float
        @param tm: Second to complete.
        @rtype: int
        @return: 1 or -1 indicating success or failure, respectively.
        '''
        # check joints are calibrated
        if not self.isCalibDone():
            waitInputConfirm('!! Calibrate Encoders with checkEncoders first !!\n\n')
            return -1

        # check servo state
        if self.isServoOn():
            return 1

        # check jname is acceptable
        if jname == '':
            jname = 'all'

        #self.rh_svc.power('all', SWITCH_ON)  #do not switch on before goActual

        try:
            waitInputConfirm(\
                '!! Robot Motion Warning (SERVO_ON) !!\n\n'
                'Confirm RELAY switched ON\n'
                'Push [OK] to switch servo ON(%s).' % (jname))
        except:  # ths needs to change
            self.rh_svc.power('all', SWITCH_OFF)
            raise

        # Need to reset JointGroups.
        # See https://code.google.com/p/rtm-ros-robotics/issues/detail?id=277
        try:
            # remove jointGroups
            self.seq_svc.removeJointGroup("larm")
            self.seq_svc.removeJointGroup("rarm")
            self.seq_svc.removeJointGroup("head")
            self.seq_svc.removeJointGroup("torso")
        except:
            print(self.configurator_name,
                  'Exception during servoOn while removing JoingGroup. ' +
                  _MSG_ASK_ISSUEREPORT)
        try:
            # setup jointGroups
            self.setSelfGroups()  # restart groups
        except:
            print(self.configurator_name,
                  'Exception during servoOn while removing setSelfGroups. ' +
                  _MSG_ASK_ISSUEREPORT)

        try:
            self.goActual()  # This needs to happen before turning servo on.
            time.sleep(0.1)
            self.rh_svc.servo(jname, SWITCH_ON)
            time.sleep(2)
            # time.sleep(7)
            if not self.isServoOn(jname):
                print self.configurator_name, 'servo on failed.'
                raise
        except:
            print self.configurator_name, 'exception occured'

        try:
            angles = self.flat2Groups(map(numpy.rad2deg,
                                          self.getActualState().angle))
            print 'Move to Actual State, Just a minute.'
            for i in range(len(self.Groups)):
                self.setJointAnglesOfGroup(self.Groups[i][0], angles[i], tm,
                                           wait=False)
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        except:
            print self.configurator_name, 'post servo on motion trouble'

        # turn on hand motors
        print 'Turn on Hand Servo'
        if self.sc_svc:
            self.sc_svc.servoOn()

        return 1

    def servoOff(self, jname='all', wait=True):
        '''
        @type jname: str
        @param jname: The value 'all' works iteratively for all servos.
        @type wait: bool
        @rtype: int
        @return: 1 = all arm servo off. 2 = all servo on arms and hands off.
                -1 = Something wrong happened.
        '''
        # do nothing for simulation
        if self.simulation_mode:
            print self.configurator_name, 'omit servo off'
            return 0

        print 'Turn off Hand Servo'
        if self.sc_svc:
            self.sc_svc.servoOff()
        # if the servos aren't on switch power off
        if not self.isServoOn(jname):
            if jname.lower() == 'all':
                self.rh_svc.power('all', SWITCH_OFF)
            return 1

        # if jname is not set properly set to all -> is this safe?
        if jname == '':
            jname = 'all'

        if wait:
            waitInputConfirm(
                '!! Robot Motion Warning (Servo OFF)!!\n\n'
                'Push [OK] to servo OFF (%s).' % (jname))  # :

        try:
            self.rh_svc.servo('all', SWITCH_OFF)
            time.sleep(0.2)
            if jname == 'all':
                self.rh_svc.power('all', SWITCH_OFF)

            # turn off hand motors
            print 'Turn off Hand Servo'
            if self.sc_svc:
                self.sc_svc.servoOff()

            return 2
        except:
            print self.configurator_name, 'servo off: communication error'
            return -1

    def checkEncoders(self, jname='all', option=''):
        '''
        Run the encoder checking sequence for specified joints,
        run goActual to adjust the direction values, and then turn servos on.

        @type jname: str
        @param jname: The value 'all' works iteratively for all servos.
        @type option: str
        @param option: Possible values are follows (w/o double quote):\
                        "-overwrite": Overwrite calibration value.
        '''
        if self.isServoOn():
            waitInputConfirm('Servo must be off for calibration')
            return
        # do not check encoders twice
        elif self.isCalibDone():
            waitInputConfirm('System has been calibrated')
            return

        self.rh_svc.power('all', SWITCH_ON)
        msg = '!! Robot Motion Warning !!\n'\
              'Turn Relay ON.\n'\
              'Then Push [OK] to '
        if option == '-overwrite':
            msg = msg + 'calibrate(OVERWRITE MODE) '
        else:
            msg = msg + 'check '

        if jname == 'all':
            msg = msg + 'the Encoders of all.'
        else:
            msg = msg + 'the Encoder of the Joint "' + jname + '".'

        try:
            waitInputConfirm(msg)
        except:
            print "If you're connecting to the robot from remote, " + \
                  "make sure tunnel X (eg. -X option with ssh)."
            self.rh_svc.power('all', SWITCH_OFF)
            return 0

        print self.configurator_name, 'calib-joint ' + jname + ' ' + option
        self.rh_svc.initializeJointAngle(jname, option)
        print self.configurator_name, 'done'
        self.rh_svc.power('all', SWITCH_OFF)
        self.goActual()  # This needs to happen before turning servo on.
        time.sleep(0.1)
        self.rh_svc.servo(jname, SWITCH_ON)

        # turn on hand motors
        print 'Turn on Hand Servo'
        if self.sc_svc:
            self.sc_svc.servoOn()

    '''
    **** All methods below here is overridden from super class
    **** to fill in api document that is missing in the upstream
    **** repository. See http://TODO_url_ticket for the detail.
    ****
    **** These methods must NOT implement new functionality.
    ****
    **** TODO: These methods must be moved to somewhere abstract,
    ****       since they are NOT specific to HIRONX.
    '''
    def getCurrentPose(self, jointname):
        '''
        @see: HrpsysConfigurator.getCurrentPose

        Returns the current physical pose of the specified joint.
        cf. getReferencePose that returns commanded value.

        eg.
             IN: robot.getCurrentPose('LARM_JOINT5')
             OUT: [-0.0017702356144599085,
              0.00019034630541264752,
              -0.9999984150158207,
              0.32556275164378523,
              0.00012155879975329215,
              0.9999999745367515,
               0.0001901314142046251,
               0.18236394191140365,
               0.9999984257434246,
               -0.00012122202968358842,
               -0.001770258707652326,
               0.07462472659364472,
               0.0,
               0.0,
               0.0,
               1.0]

        @type jointname: str
        @rtype: List of float
        @return: Rotational matrix and the position of the given joint in
                 1-dimensional list, that is:

                 [a11, a12, a13, x,
                  a21, a22, a23, y,
                  a31, a32, a33, z,
                   0,   0,   0,  1]
        '''
        return HrpsysConfigurator.getCurrentPose(self, jointname)

    def getCurrentPosition(self, jointname):
        '''
        @see: HrpsysConfigurator.getCurrentPosition

        Returns the current physical position of the specified joint.
        cf. getReferencePosition that returns commanded value.

        eg.
            robot.getCurrentPosition('LARM_JOINT5')
            [0.325, 0.182, 0.074]

        @type jointname: str
        @rtype: List of float
        @return: List of x, y, z positions about the specified joint.
        '''
        return HrpsysConfigurator.getCurrentPosition(self, jointname)

    def getCurrentRotation(self, jointname):
        '''
        @see: HrpsysConfigurator.getCurrentRotation

        Returns the current physical rotation of the specified joint.
        cf. getReferenceRotation that returns commanded value.

        @type jointname: str
        @rtype: List of float
        @return: Rotational matrix of the given joint in 2-dimensional list,
                 that is:
                 [[a11, a12, a13],
                  [a21, a22, a23],
                  [a31, a32, a33]]
        '''
        return HrpsysConfigurator.getCurrentRotation(self, jointname)

    def getCurrentRPY(self, jointname):
        '''
        @see: HrpsysConfigurator.getCurrentRPY

        Returns the current physical rotation in RPY of the specified joint.
        cf. getReferenceRPY that returns commanded value.

        @type jointname: str
        @rtype: List of float
        @return: List of orientation in rpy form about the specified joint.
        '''
        return HrpsysConfigurator.getCurrentRPY(self, jointname)

    def getJointAngles(self):
        '''
        @see: HrpsysConfigurator.getJointAngles

        Returns the commanded joint angle values.

        Note that it's not the physical state of the robot's joints, which
        can be obtained by getActualState().angle.

        @rtype: List of float
        @return: List of angles (degree) of all joints, in the order defined
                 in the member variable 'Groups' (eg. chest, head1, head2, ..).
        '''
        return HrpsysConfigurator.getJointAngles(self)

    def getReferencePose(self, jointname):
        '''
        @see: HrpsysConfigurator.getReferencePose

        Returns the current commanded pose of the specified joint.
        cf. getCurrentPose that returns physical pose.

        @rtype: List of float
        @return: Rotational matrix and the position of the given joint in
                 1-dimensional list, that is:

                 [a11, a12, a13, x,
                  a21, a22, a23, y,
                  a31, a32, a33, z,
                   0,   0,   0,  1]
        '''
        return HrpsysConfigurator.getReferencePose(self, jointname)

    def getReferencePosition(self, jointname):
        '''
        @see: HrpsysConfigurator.getReferencePosition

        Returns the current commanded position of the specified joint.
        cf. getCurrentPosition that returns physical value.

        @type jointname: str
        @rtype: List of float
        @return: List of x, y, z positions about the specified joint.
        '''
        return HrpsysConfigurator.getReferencePosition(self, jointname)

    def getReferenceRotation(self, jointname):
        '''
        @see: HrpsysConfigurator.getReferenceRotation

        Returns the current commanded rotation of the specified joint.
        cf. getCurrentRotation that returns physical value.

        @type jointname: str
        @rtype: List of float
        @return: Rotational matrix of the given joint in 2-dimensional list,
                 that is:
                 [[a11, a12, a13],
                  [a21, a22, a23],
                  [a31, a32, a33]]
        '''
        return HrpsysConfigurator.getReferenceRotation(self, jointname)

    def getReferenceRPY(self, jointname):
        '''
        @see: HrpsysConfigurator.getReferenceRPY

        Returns the current commanded rotation in RPY of the specified joint.
        cf. getCurrentRPY that returns physical value.

        @type jointname: str
        @rtype: List of float
        @return: List of orientation in rpy form about the specified joint.
        '''
        return HrpsysConfigurator.getReferenceRPY(self, jointname)

    def goActual(self):
        '''
        @see: HrpsysConfigurator.goActual (that calls StateHolder::goActual)

        Adjust directional values to the angles in the physical state. This
        needs to be run BEFORE servos are turned on.
        '''
        HrpsysConfigurator.goActual(self)

    def loadPattern(self, fname, tm):
        '''
        @see: HrpsysConfigurator.loadPattern

        Load a pattern file that is created offline.

        Format of the pattern file:
        - example format:

          t0 j0 j1 j2...jn
          t1 j0 j1 j2...jn
          :
          tn j0 j1 j2...jn

        - Delimmitted by space
        - Each line consists of an action.
        - Time between each action is defined by tn+1 - tn
          - The time taken for the 1st line is defined by the arg tm.

        @param fname: Name of the pattern file.
        @type tm: double
        @param tm: - The time to take for the 1st line.
        @return: List of 2 oct(string) values.
        '''
        return HrpsysConfigurator.loadPattern(self, fname, tm)

    def readDigitalInput(self):
        '''
        @see: HrpsysConfigurator.readDigitalInput

        Digital input consits of 14 bits. The last 2 bits are lacking
        and compensated, so that the last 4 bits are 0x4 instead of 0x1.

        @rtype: [str]
        @return: List of 2 oct(string) values.
        '''
        return HrpsysConfigurator.readDigitalInput(self)
        #TODO: Catch AttributeError that occurs when RobotHardware not found.
        #      Typically with simulator, erro msg might look like this;
        #      'NoneType' object has no attribute 'readDigitalInput'

    def readDigitaloutput(self):
        '''
        @see: HrpsysConfigurator.readDigitaloutput

        Digital input consits of 14 bits. The last 2 bits are lacking
        and compensated, so that the last 4 bits are 0x4 instead of 0x1.

        @rtype: [int]
        @return: List of 2 oct(string) values.
        '''
        return HrpsysConfigurator.readDigitaloutput(self)
        #TODO: Catch AttributeError that occurs when RobotHardware not found.
        #      Typically with simulator, erro msg might look like this;
        #      'NoneType' object has no attribute 'readDigitaloutput'

    def setJointAngle(self, jname, angle, tm):
        '''
        @see: HrpsysConfigurator.setJointAngle

        Set angle to the given joint.

        NOTE-1: It's known that this method does not do anything after
                some group operation is done.
                TODO: at least need elaborated to warn users.

        NOTE-2: that while this method does not check angle value range,
                any joints could emit position limit over error, which has not
                yet been thrown by hrpsys so that there's no way to catch on
                this client side. Worthwhile opening an enhancement ticket
                for that at hironx' designated issue tracker.

        @type jname: str
        @type angle: float
        @param angle: In degree.
        @type tm: float
        @param tm: Time to complete.
        '''
        return HrpsysConfigurator.setJointAngle(self, jname, angle, tm)

    def setJointAngles(self, angles, tm):
        '''
        @see: HrpsysConfigurator.setJointAngles

        NOTE-1: that while this method does not check angle value range, any
                joints could emit position limit over error, which has not yet
                been thrown by hrpsys so that there's no way to catch on this
                client side. Worthwhile opening an enhancement ticket for that
                at hironx' designated issue tracker.

        @type angles: float
        @param angles: In degree.
        @type tm: float
        @param tm: Time to complete.
        '''
        return HrpsysConfigurator.setJointAngles(self, angles, tm)

    def setJointAnglesOfGroup(self, gname, pose, tm, wait=True):
        '''
        @see: HrpsysConfigurator.setJointAnglesOfGroup

        Set the joint angles to aim. By default it waits interpolation to be
        over.

        Note that while this method does not check angle value range,
        any joints could emit position limit over error, which has not yet
        been handled in hrpsys so that there's no way to catch on this client
        class level. Please consider opening an enhancement ticket for that
        at hironx' designated issue tracker.

        @type gname: str
        @param gname: Name of joint group.
        @type pose: [float]
        @param pose: list of positions and orientations
        @type tm: float
        @param tm: Time to complete.
        @type wait: bool
        @param wait: If true, SequencePlayer.waitInterpolationOfGroup gets run.
        '''
        return HrpsysConfigurator.setJointAnglesOfGroup(self, gname, pose, tm,
                                                        wait)

    def setTargetPose(self, gname, pos, rpy, tm, ref_frame_name=None):
        '''
        @see: HrpsysConfigurator.setTargetPose

        Set absolute pose to a joint.
        All d* arguments are in meter.

        @param gname: Name of the joint group.
        @type pos: float
        @type rpy: TODO: ??
        @rtype: bool
        @type ref_frame_name: str
        @param ref_frame_name: Name of the frame that this particular command
                               reference to.
        '''
        return HrpsysConfigurator.setTargetPose(self, gname, pos, rpy, tm, ref_frame_name)

    def setTargetPoseRelative(self, gname, target_frame, dx=0, dy=0, dz=0,
                              dr=0, dp=0, dw=0, tm=10, wait=True):
        '''
        @see: HrpsysConfigurator.setTargetPoseRelative

        Set angles to a joint group relative to its current pose.
        All d* arguments are in meter.

        Example usage: The following moves RARM_JOINT5 joint 0.1mm forward
                       within 0.1sec.

            robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dx=0.0001,
                                        tm=0.1)

        @param gname: Name of the joint group.
        @param target_frame: Frame that the pose will be calculated against.
        @rtype: bool
        '''
        return HrpsysConfigurator.setTargetPoseRelative(self, gname,
                                                     target_frame, dx, dy, dz,
                                                     dr, dp, dw, tm, wait)

    def waitInterpolationOfGroup(self, groupname):
        '''
        Lets SequencePlayer wait until the movement currently happening to
        finish.

        @see: SequencePlayer.waitInterpolationOfGroup
        @see: http://wiki.ros.org/joint_trajectory_action. This method
              corresponds to JointTrajectoryGoal in ROS.

        @type groupname: str
        '''
        self.seq_svc.waitInterpolationOfGroup(groupname)

    def writeDigitalOutput(self, dout):
        '''
        @see: HrpsysConfigurator.writeDigitalOutput

        Using writeDigitalOutputWithMask is recommended for the less data
        transport.

        @type dout: [int]
        @param dout: List of bits, length of 32 bits where elements are
                     0 or 1.

                     What each element stands for depends on how
                     the robot's imlemented. Consult the hardware manual.

        @rtype: bool
        @return: RobotHardware.writeDigitalOutput returns True if writable. False otherwise.
        '''
        return HrpsysConfigurator.writeDigitalOutput(self, dout)

    def writeDigitalOutputWithMask(self, dout, mask):
        '''
        @see: HrpsysConfigurator.writeDigitalOutputWithMask

        Both dout and mask are lists with length of 32. Only the bit in dout
        that corresponds to the bits in mask that are flagged as 1 will be
        evaluated.

        Example:
         Case-1. Only 18th bit will be evaluated as 1.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

         Case-2. Only 18th bit will be evaluated as 0.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

         Case-3. None will be evaluated since there's no flagged bit in mask.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        @type dout: [int]
        @param dout: List of bits, length of 32 bits where elements are
                     0 or 1.
        @type mask: [int]
        @param mask: List of masking bits, length of 32 bits where elements are
                     0 or 1.
        @rtype: bool
        @return: RobotHardware.writeDigitalOutput returns True if writable. False otherwise.
        '''
        return HrpsysConfigurator.writeDigitalOutputWithMask(self, dout, mask)

    def clear(self):
        '''
        @see HrpsysConfigurator.clear
        Clears the Sequencer's current operation. Works for joint groups too.

        Discussed in https://github.com/fkanehiro/hrpsys-base/issues/158
        Examples is found in a unit test: https://github.com/start-jsk/rtmros_hironx/blob/bb0672be3e03e5366e03fe50520e215302b8419f/hironx_ros_bridge/test/test_hironx.py#L293
        '''
        HrpsysConfigurator.clear(self)
