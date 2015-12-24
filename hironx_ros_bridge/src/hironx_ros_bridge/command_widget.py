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
        self.pushButton_servoOn.clicked[bool].connect(self._servo_on)

    def _get_duration(self):
        '''
        @rtype float
        '''
        return float(self.doubleSpinBox_duration.text())

    def _check_encoders(self):
        self._rtm.checkEncoders()

    def _go_initial(self):
        self._rtm.goInitial(tm=self._get_duration())

    def _go_initial_factoryval(self):
        self._rtm.goInitial(init_pose_type=self._rtm.INITPOS_TYPE_FACTORY,
                            tm=self._get_duration())

    def _go_offPose(self):
        self._rtm.goOffPose(tm=self._get_duration())

    def _servo_on(self):
        self._rtm.servoOn()

    def _servo_off(self):
        self._rtm.servoOff()

    def _impedance_on(self):
        self._rtm.startImpedance()

    def _imdedance_off(self):
        self._rtm.stopImpedance()
