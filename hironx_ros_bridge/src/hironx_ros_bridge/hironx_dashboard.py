#!/usr/bin/env python

from hrpsys_ros_bridge.hrpsys_dashboard import HrpsysDashboard
from rqt_robot_dashboard.widgets import MenuDashWidget

from python_qt_binding.QtGui import QMessageBox, QLabel, QPalette

import os
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
        self._name_label = HiroNXNameLabel("HiroNX "+os.environ["ROS_MASTER_URI"]+" ")

    def get_widgets(self):
        widgets = super(HiroNXDashboard, self).get_widgets()
        widgets.append([self._name_label])
        return widgets
