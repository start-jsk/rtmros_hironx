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

import roslib
roslib.load_manifest("hrpsys")
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

def delete_module(modname, paranoid=None):
    from sys import modules
    try:
        thismod = modules[modname]
    except KeyError:
        raise ValueError(modname)
    these_symbols = dir(thismod)
    if paranoid:
        try:
            paranoid[:]  # sequence support
        except:
            raise ValueError('must supply a finite list for paranoid')
        else:
            these_symbols = paranoid[:]
    del modules[modname]
    for mod in modules.values():
        try:
            delattr(mod, modname)
        except AttributeError:
            pass
        if paranoid:
            for symbol in these_symbols:
                if symbol[:2] == '__':  # ignore special symbols
                    continue
                try:
                    delattr(mod, symbol)
                except AttributeError:
                    pass

class HIRONX(HrpsysConfigurator):
    '''
    @see: <a href = "https://github.com/fkanehiro/hrpsys-base/blob/master/" +
                    "python/hrpsys_config.py">HrpsysConfigurator</a>

    This class holds methods that are specific to Kawada Industries' dual-arm
    robot called Hiro.

    For the API doc for the derived methods, please see the parent
    class via the link above; nicely formatted api doc web page
    isn't available yet (discussed in
    https://github.com/fkanehiro/hrpsys-base/issues/268).
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
    OffPose = [[0],
               [0, 0],
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

    hrpsys_version = '0.0.0'

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
        self.hrpsys_version = self.fk.ref.get_component_profile().version

        # reload for hrpsys 315.1.8
        if self.hrpsys_version < '315.2.0':
            delete_module('ImpedanceControllerService_idl')
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'hrpsys_315_1_9/hrpsys'))
            import ImpedanceControllerService_idl
            self.ic_svc = narrow(self.ic.service("service0"), "ImpedanceControllerService")

        # connect ic if needed
        for sensor in ['lhsensor' , 'rhsensor']:
            if self.ic and self.ic.port(sensor) and self.ic.port(sensor).get_port_profile() and \
                    not self.ic.port(sensor).get_port_profile().connector_profiles :
                connectPorts(self.rh.port(sensor), self.ic.port(sensor))


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
            print(
                '{}, JntAnglesOfGr={}, INITPOSE[i]={}, tm={}, wait={}'.format(
                    self.configurator_name, self.Groups[i][0], _INITPOSE[i],
                    tm, wait))
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
        rtclist = [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['ic', "ImpedanceController"],
            ['el', "SoftErrorLimiter"],
            # ['co', "CollisionDetector"],
            ['sc', "ServoController"],
            ['log', "DataLogger"],
            ]
        if hasattr(self, 'rmfo'):
            self.ms.load("RemoveForceSensorLinkOffset")
            self.ms.load("AbsoluteForceSensor")
            if "RemoveForceSensorLinkOffset" in self.ms.get_factory_names():
                rtclist.append(['rmfo', "RemoveForceSensorLinkOffset"])
            elif "AbsoluteForceSensor" in self.ms.get_factory_names():
                rtclist.append(['rmfo', "AbsoluteForceSensor"])
            else:
                print "Component rmfo is not loadable."
        return rtclist

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

        @rtype: <a href = "http://hrpsys-base.googlecode.com/svn/doc/df/d17/" +
                          "structOpenHRP_1_1RobotHardwareService_1_1" +
                          "RobotState.html">
                          OpenHRP::RobotHardwareService::RobotState</a>
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
            waitInputConfirm(
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

    def startImpedance_315_1(self, arm,
                       M_p = 100.0,
                       D_p = 100.0,
                       K_p = 100.0,
                       M_r = 100.0,
                       D_r = 2000.0,
                       K_r = 2000.0,
                       ref_force = [0, 0, 0],
                       force_gain = [1, 1, 1],
                       ref_moment = [0, 0, 0],
                       moment_gain = [0, 0, 0],
                       sr_gain = 1.0,
                       avoid_gain = 0.0,
                       reference_gain = 0.0,
                       manipulability_limit = 0.1):
        ic_sensor_name = 'rhsensor'
        ic_target_name = 'RARM_JOINT5'
        if arm == 'rarm':
            ic_sensor_name = 'rhsensor'
            ic_target_name = 'RARM_JOINT5'
        elif arm == 'larm':
            ic_sensor_name = 'lhsensor'
            ic_target_name = 'LARM_JOINT5'
        else:
            print 'startImpedance: argument must be rarm or larm.'
            return

        self.ic_svc.setImpedanceControllerParam(
            OpenHRP.ImpedanceControllerService.impedanceParam(
                name = ic_sensor_name,
                base_name = 'CHEST_JOINT0',
                target_name = ic_target_name,
                M_p = M_p,
                D_p = D_p,
                K_p = K_p,
                M_r = M_r,
                D_r = D_r,
                K_r = K_r,
                ref_force = ref_force,
                force_gain = force_gain,
                ref_moment = ref_moment,
                moment_gain = moment_gain,
                sr_gain = sr_gain,
                avoid_gain = avoid_gain,
                reference_gain = reference_gain,
                manipulability_limit = manipulability_limit))

    def stopImpedance_315_1(self, arm):
        ic_sensor_name = 'rhsensor'
        if arm == 'rarm':
            ic_sensor_name = 'rhsensor'
        elif arm == 'larm':
            ic_sensor_name = 'lhsensor'
        else:
            print 'startImpedance: argument must be rarm or larm.'
            return
        self.ic_svc.deleteImpedanceControllerAndWait(ic_sensor_name)

    def startImpedance_315_2(self, arm,
                       M_p = 100.0,
                       D_p = 100.0,
                       K_p = 100.0,
                       M_r = 100.0,
                       D_r = 2000.0,
                       K_r = 2000.0,
                       force_gain = [1, 1, 1],
                       moment_gain = [0, 0, 0],
                       sr_gain = 1.0,
                       avoid_gain = 0.0,
                       reference_gain = 0.0,
                       manipulability_limit = 0.1):
        self.ic_svc.setImpedanceControllerParam(
            arm,
            OpenHRP.ImpedanceControllerService.impedanceParam(
                M_p = M_p,
                D_p = D_p,
                K_p = K_p,
                M_r = M_r,
                D_r = D_r,
                K_r = K_r,
                force_gain = force_gain,
                moment_gain = moment_gain,
                sr_gain = sr_gain,
                avoid_gain = avoid_gain,
                reference_gain = reference_gain,
                manipulability_limit = manipulability_limit))
        return self.ic_svc.startImpedanceController(arm)

    def startImpedance_315_3(self, arm,
                       M_p = 100.0,
                       D_p = 100.0,
                       K_p = 100.0,
                       M_r = 100.0,
                       D_r = 2000.0,
                       K_r = 2000.0,
                       force_gain = [1, 1, 1],
                       moment_gain = [0, 0, 0],
                       sr_gain = 1.0,
                       avoid_gain = 0.0,
                       reference_gain = 0.0,
                       manipulability_limit = 0.1,
                       controller_mode = OpenHRP.ImpedanceControllerService.MODE_IMP):
        self.ic_svc.setImpedanceControllerParam(
            arm,
            OpenHRP.ImpedanceControllerService.impedanceParam(
                M_p = M_p,
                D_p = D_p,
                K_p = K_p,
                M_r = M_r,
                D_r = D_r,
                K_r = K_r,
                force_gain = force_gain,
                moment_gain = moment_gain,
                sr_gain = sr_gain,
                avoid_gain = avoid_gain,
                reference_gain = reference_gain,
                manipulability_limit = manipulability_limit,
                controller_mode = controller_mode))
        return self.ic_svc.startImpedanceController(arm)

    def stopImpedance_315_2(self, arm):
        return self.ic_svc.stopImpedanceController(arm)

    def stopImpedance_315_3(self, arm):
        return self.ic_svc.stopImpedanceController(arm)

    def startImpedance(self, arm, **kwargs):
        if self.hrpsys_version < '315.2.0':
            self.startImpedance_315_1(arm, **kwargs)
        elif self.hrpsys_version < '315.3.0':
            self.startImpedance_315_2(arm, **kwargs)
        else:
            self.startImpedance_315_3(arm, **kwargs)

    def stopImpedance(self, arm):
        if self.hrpsys_version < '315.2.0':
            self.stopImpedance_315_1(arm)
        elif self.hrpsys_version < '315.3.0':
            self.stopImpedance_315_2(arm)
        else:
            self.stopImpedance_315_3(arm)

    def removeForceSensorOffset(self):
        self.rh_svc.removeForceSensorOffset()

    def getCurrentPose(self, lname=None, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getCurrentPose({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getCurrentPose(self, lname)
        else:
            return HrpsysConfigurator.getCurrentPose(self, lname, frame_name)
    
    def getCurrentPosition(self, lname=None, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None

        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getCurrentPosition({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getCurrentPosition(self, lname)
        else:
            return HrpsysConfigurator.getCurrentPosition(self, lname, frame_name)

    def getCurrentRotation(self, lname=None, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getCurrentRotation({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getCurrentRotation(self, lname)
        else:
            return HrpsysConfigurator.getCurrentRotation(self, lname, frame_name)
    
    def getCurrentRPY(self, lname, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None

        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getCurrentRPY({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getCurrentRPY(self, lname)
        else:
            return HrpsysConfigurator.getCurrentRPY(self, lname, frame_name)

    def getReferencePose(self, lname, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getReferencePose({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getReferencePose(self, lname)
        else:
            return HrpsysConfigurator.getReferencePose(self, lname, frame_name)

    def getReferencePosition(self, lname, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getReferencePosition({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getReferencePosition(self, lname)
        else:
            return HrpsysConfigurator.getReferencePosition(self, lname, frame_name)

    def getReferenceRotation(self, lname, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getReferenceRotation({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getReferenceRotation(self, lname)
        else:
            return HrpsysConfigurator.getReferenceRotation(self, lname, frame_name)

    def getReferenceRPY(self, lname, frame_name='WAIST'):
        if ':' in lname:
            frame_name = None
        
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m getReferenceRPY({}, {}) is not supported on {}\033[0m".format(lname, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.getReferenceRPY(self, lname)
        else:
            return HrpsysConfigurator.getReferenceRPY(self, lname, frame_name)
    
    def setTargetPose(self, gname, pos, rpy, tm, frame_name='WAIST'):
        if self.hrpsys_version <= '315.2.4':
            print "\033[33m setTargetPose({}, {}, {}, {}, {}) is not supported on {}\033[0m".format(gname, pos, rpy, tm, frame_name, self.hrpsys_version)
            return HrpsysConfigurator.setTargetPose(self, gname, pos, rpy, tm)
        else:
            return HrpsysConfigurator.setTargetPose(self, gname, pos, rpy, tm, frame_name)

