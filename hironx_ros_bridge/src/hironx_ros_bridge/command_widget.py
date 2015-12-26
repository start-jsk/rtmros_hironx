#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association (TORK).
#    nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
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

import os

from hironx_ros_bridge.hironx_client import HIRONX
from hrpsys_ros_bridge.hrpsys_dashboard import HrpsysDashboard
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import (QHeaderView, QItemSelectionModel,
                                     QWidget)
from rospkg import RosPack
import rospy
from rospy.exceptions import ROSException
import rosservice
from rqt_robot_dashboard.widgets import MenuDashWidget

PKG_NAME = 'hironx_ros_bridge'


class HironxoCommandPanel(QWidget):
    '''
    Design decisions:

    - joint_state_publisher is dropped, since hrpsys_ros_bridge seems to set
    `robot_description` param using COLLADA (see https://goo.gl/aLpILa)
    instead of URDF, which joint_state_publisher is not capable of.
    '''

    def __init__(self, parent, guicontext):
        '''
        A GUi panel to list common operation commands for Hironx / NEXTAGE Open robot.

        @param guicontext: The plugin context to create the monitor in.
        @type guicontext: qt_gui.plugin_context.PluginContext
        '''
        super(HironxoCommandPanel, self).__init__()
        self._parent = parent
        self._guicontext = guicontext

        # RTM Client
        self._rtm = HIRONX()
        self._rtm.init(robotname='HiroNX(Robot)0', url='')

        rospack = RosPack()
        ui_file = os.path.join(rospack.get_path(PKG_NAME), 'resource',
                               'hironx_commandpanel_main.ui')
        loadUi(ui_file, self, {'HironxoCommandPanel': HironxoCommandPanel})

        # Assign callback methods
        self.pushButton_checkEncoders.clicked[bool].connect(self._check_encoders)
        self.pushButton_goInitial.clicked[bool].connect(self._go_initial)
        self.pushButton_goInitial.clicked[bool].connect(self._go_initial)
        self.pushButton_goInitial_factoryval.clicked[bool].connect(self._go_initial_factoryval)
        self.pushButton_goOffPose.clicked[bool].connect(self._go_offPose)
        self.pushButton_startImpedance.clicked[bool].connect(self._impedance_on)
        self.pushButton_stopImpedance.clicked[bool].connect(self._impedance_off)

    def _get_duration(self):
        '''
        @rtype float
        '''
        return float(self.doubleSpinBox_duration.text())

    def _get_arm_impedance(self):
        '''
        @rtype str
        @return: Returns a name of arm group that is checked on GUI. None by default.
        '''
        checked_arm = None
        if self.radioButton_impedance_left.isChecked():
            checked_arm = 'larm'
        elif self.radioButton_impedance_right.isChecked():
            checked_arm = 'rarm'
        return checked_arm

    def _check_encoders(self):
        self._rtm.checkEncoders()

    def _go_initial(self):
        self._rtm.goInitial(tm=self._get_duration())

    def _go_initial_factoryval(self):
        self._rtm.goInitial(init_pose_type=self._rtm.INITPOS_TYPE_FACTORY,
                            tm=self._get_duration())

    def _go_offPose(self):
        self._rtm.goOffPose(tm=self._get_duration())

    def _impedance_on_off(self, do_on=True):
        '''
        Start/stop impedance control for the specified arm group.
        Arm group to operate impedance control is automatically obtained from
        GUI internally within this method.
        @raise AttributeError: When no arm group is specified.
        @type do_on: bool
        @param do_on: On/off of impedance control
        '''
        armgroup = self._get_arm_impedance()
        if not armgroup:
            raise AttributeError('No arm is specified for impedance control to start.')
        if do_on:
            self._rtm.startImpedance(armgroup)
        elif not do_on:
            self._rtm.stopImpedance(armgroup)

    def _impedance_on(self):
        self._impedance_on_off()

    def _impedance_off(self):
        self._impedance_on_off(do_on=False)
