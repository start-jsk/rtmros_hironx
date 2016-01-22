#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, JSK Lab, University of Tokyo
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

import os

from python_qt_binding.QtGui import QMessageBox, QLabel, QPalette

from hrpsys_ros_bridge.hrpsys_dashboard import HrpsysDashboard
from rqt_robot_dashboard.widgets import MenuDashWidget

from hironx_ros_bridge.command_widget import HironxoCommandPanel


class HiroNXNameLabel(QLabel):
    def __init__(self, name):
        super(HiroNXNameLabel, self).__init__()
        palette = QPalette()
        self.setStyleSheet('font-size: larger; font-weight: bold; color: #ffffff; background-color: darkgreen;')
        self.setText(name)


class HiroNXDashboard(HrpsysDashboard):
    def setup(self, context):
        super(HiroNXDashboard, self).setup(context)
        self.name = "HiroNX dashboard"
        self._imp_button = None
        self._pose_button = None
        self._name_label = HiroNXNameLabel("HiroNX " + os.environ["ROS_MASTER_URI"] + " ")
        self._command_panel = HironxoCommandPanel(self, self.context)
        context.add_widget(self._command_panel)

    def get_widgets(self):
        widgets = super(HiroNXDashboard, self).get_widgets()
        widgets.append([self._name_label])
        return widgets
