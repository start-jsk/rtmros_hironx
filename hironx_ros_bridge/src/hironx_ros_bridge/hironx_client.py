#!/usr/bin/env python
# -*- coding: utf-8 -*-


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


class HIRONX(HrpsysConfigurator):
    '''
    This class holds methods that are specific to Kawada Industries' dual-arm
    robot called Hiro.
    '''

    OffPose = [[0], [0, 0],
                   [25, -139, -157, 45, 0, 0],
                   [-25, -139, -157, -45, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]
    InitialPose = [[0], [0, 0],
                   [-0.6, 0, -100, 15.2, 9.4, 3.2],
                   [0.6, 0, -100, -15.2, 9.4, -3.2],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]]

    Groups = [['torso', ['CHEST_JOINT0']],
              ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
              ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']],
              ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]]
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
        Set predefined (as member variable) off pose per each body groups.

        @type tm: int
        @param tm: time
        '''
        for i in range(len(self.Groups)):
            self.setJointAnglesOfGroup(self.Groups[i][0], self.OffPose[i], tm,
                                       wait=False)
        for i in range(len(self.Groups)):
            self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        self.servoOff(wait=False)

    def goInitial(self, tm=7, wait=True):
        '''
        Set predefined initial pose per each body groups. Predefined moves are defined as member variable of this class.

        @type tm: int
        @param tm: Time that takes for the whole process.
        @type wait: bool
        @param wait: If true, SequencePlayer.waitInterpolationOfGroup gets run. (TODO: Elaborate what this means...Even after having taken a look at its source code I can't tell exactly what it means.)
        '''
        ret = True
        for i in range(len(self.Groups)):
            # radangles = [x/180.0*math.pi for x in self.InitialPose[i]]
            print self.configurator_name, 'self.setJointAnglesOfGroup(', \
                  self.Groups[i][0], ',', self.InitialPose[i], ', ', tm, \
                  ',wait=False)'
            ret &= self.setJointAnglesOfGroup(self.Groups[i][0], self.InitialPose[i], tm, wait=False)
        if wait:
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        return ret

    def getRTCList(self):
        '''
        Overwriting HrpsysConfigurator.getRTCList
        Returning predefined list of RT components.
        @rtype [[str]]
        @rerutrn List of available components. Each element consists of a list of abbreviated and full names of the component.
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

    #
    # hand interface
    # effort: 1~100[%]
    # hiro.HandOpen("rhand", effort)
    # hiro.HandOpen()        # for both hand
    # hiro.HandClose("rhand", effort)
    # hiro.HandClose()       # for both hand
    #
    def HandOpen(self, hand=None, effort=None):
        '''
        Set the stretch between two fingers of the specified hand as hardcoded value (100mm).

        @type hand: str
        @type effort: int
        '''
        self.setHandWidth(hand, 100, effort=effort)

    def HandClose(self, hand=None, effort=None):
        '''
        @type hand: str
        @type effort: int
        '''
        self.setHandWidth(hand, 0, effort=effort)

    def setHandJointAngles(self, hand, angles, tm=1):
        '''
        @type hand: str
        @param hand: which hand. (TODO: List the possible values)
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

    def moveHand(self, hand, av, tm=1) :  # direction av : + for open, - for close
        '''
        Negate the angle value for {2, 3, 6, 7}th element in av.

        @type hand: str
        @param hand: Specifies hand. (TODO: List the possible values. Should be listed in setHandJointAngles so just copy from its doc.)
        @type av: [int]
        @param av: angle of each joint of the specified arm (TODO: need verified. Also what's the length of the list?)
        @param tm: Time in second to complete the work.
        '''
        for i in [2, 3, 6, 7]:  # do not change this line if servo is difference, change HandGroups
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
        Set elements of body groups and joing groups that are statically
        defined as member variables within this class.
        '''
        for item in self.Groups:
            self.seq_svc.addJointGroup(item[0], item[1])
        for k, v in self.HandGroups.iteritems():
            self.sc_svc.addJointGroup(k, v)

    def getActualState(self):
        '''
        TODO: needs documented. What state does this return?
        '''
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

    def liftRobotUp(self):
        '''
        TODO: needs documented. Returning always true?
        '''
        return True

    def stOff(self):
        '''
        TODO: needs documented. Returning always false?
        '''
        return False

    def flat2Groups(self, flatList):
        retList = []
        index = 0
        for group in self.Groups:
            joint_num = len(group[1])
            retList.append(flatList[index: index + joint_num])
            index += joint_num
        return retList

    def servoOn(self, jname='all', destroy=1, tm=3):
        '''
        Turn on/off servos. Calibration of joints need to be done already.

        @type jname: str
        @param jname: The value 'all' works iteratively for all servos.
        @param destroy: Not used.
        @type tm: int
        @param tm: Time to complete.
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

        self.liftRobotUp()
        #self.rh_svc.power('all', SWITCH_ON)  # 1/13/14 commented out as debug trial

        try:
            waitInputConfirm(\
                '!! Robot Motion Warning (SERVO_ON) !!\n\n'
                'Confirm RELAY switched ON\n'
                'Push [OK] to switch servo ON(%s).' % (jname))
        except:  # ths needs to change
            self.rh_svc.power('all', SWITCH_OFF)
            raise

        try:
            self.stOff()
            self.goActual()
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
            angles = self.flat2Groups(map(numpy.rad2deg, self.getActualState().angle))
            print 'Move to Actual State, Just a minute.'
            for i in range(len(self.Groups)):
                self.setJointAnglesOfGroup(self.Groups[i][0], angles[i], tm, wait=False)
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        except:
            print self.configurator_name, 'post servo on motion trouble'

        # turn on hand motors
        print 'Turn on Hand Servo'
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
        self.sc_svc.servoOff()
        # if the servos aren't on switch power off
        if not self.isServoOn(jname):
            if jname.lower() == 'all':
                self.rh_svc.power('all', SWITCH_OFF)
            return 1

        # if jname is not set properly set to all -> is this safe?
        if jname == '':
            jname = 'all'

        self.liftRobotUp()

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
            self.sc_svc.servoOff()

            return 2
        except:
            print self.configurator_name, 'servo off: communication error'
            return -1

    def checkEncoders(self, jname='all', option=''):
        '''
        Run the encoder checking sequence for specified joints,
        run goActual and turn on servos.

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
        self.goActual()
        time.sleep(0.1)
        self.rh_svc.servo(jname, SWITCH_ON)

        # turn on hand motors
        print 'Turn on Hand Servo'
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

    def goActual():
        ''' 
        StateHolder::goActual
        TODO: behavior needs documented; looking at the original
              method's code doesn't give enough hint to do so.
        '''
        self.goActual()

    def setJointAngle(self, jname, angle, tm):
        '''
        HrpsysConfigurator.setJointAngle

        @type jname: str
        @param jname: Name of joint.
        @type angle: double
        @type tm: double
        @param tm: Time to complete.
        '''
        self.setJointAngle(jname, angle, tm)

    def setJointAnglesOfGroup(self, gname, pose, tm, wait=True):
        '''
        HrpsysConfigurator.setJointAnglesOfGroup

        @type gname: str
        @param gname: Name of body link group.
        @type pose: [double]
        @param pose: list of positions and orientations
        @type tm: double
        @param tm: Time to complete.
        @type wait: bool
        @param wait: If true, SequencePlayer.waitInterpolationOfGroup gets run. (TODO: Elaborate what this means...Even after having taken a look at its source code I can't tell exactly what it means.)
        '''
        self.setJointAnglesOfGroup(gname, pose, tm, wait)

    def setTargetPoseRelative(self, gname, eename, dx=0, dy=0, dz=0,
dr=0, dp=0, dw=0, tm=10, wait=True):
        '''
        Set angles to a body link group relative to its current pose.
        All d* arguments are in meter.

        @param gname: Name of the link group.
        @param eename: Name of the link.
        '''
        self.setTargetPoseRelative(gname, eename, dx, dy, dz, dr, dp, dw, tm, wait)

    def writeDigitalOutput(self, dout):
        '''
        @type dout: [int]
        @param dout: List of bits. Length might defer depending on robot's implementation.
        @return: What RobotHardware.writeDigitalOutput returns (TODO: document).
        '''
        self.writeDigitalOutput(dout)

    def writeDigitalOutputWithMask(self, dout, mask):
        '''
        @type dout: [int]
        @param dout: List of bits. Length might defer depending on robot's implementation.
        @type mask: [int]
        @param mask: List of masking bits. Length depends on that of dout. 
        @return: What RobotHardware.writeDigitalOutput returns (TODO: document).
        '''
        sel.writeDigitalOutputWithMask(dout, mask)

    def readDigitalInput(self):
        '''
        HrpsysConfigurator.readDigitalInput

        TODO: document
        '''
        self.readDigitalInput()

    ''' TODO: add What Urko asked '''
