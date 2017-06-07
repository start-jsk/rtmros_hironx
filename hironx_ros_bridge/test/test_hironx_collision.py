#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import numpy

from hironx_ros_bridge import hironx_client as hironx
from hrpsys import rtm

_ARMGROUP_TESTED = 'larm'
_LINK_TESTED = 'LARM_JOINT5'
_GOINITIAL_TIME_MIDSPEED = 3  # second
_NUM_CARTESIAN_ITERATION = 300
_PKG = 'nextage_ros_bridge'


class TestNextageopen(unittest.TestCase):
    '''
    Test NextageClient with rostest.
    '''

    @classmethod
    def setUpClass(self):

        #modelfile = rospy.get_param("hironx/collada_model_filepath")
        #rtm.nshost = 'hiro024'
        #robotname = "RobotHardware0"

        self._robot = hironx.HIRONX()
        #self._robot.init(robotname=robotname, url=modelfile)
        self._robot.init()

        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def _set_collide_y(self):
        self._robot.seq_svc.setMaxIKError(0.00001, 0.01)
        init_pose = self._robot.getCurrentPosition(_LINK_TESTED)
        self._robot.setTargetPoseRelative(_ARMGROUP_TESTED, _LINK_TESTED, dy = -0.3, tm = 1)


    def test_collision_dx(self):
        assert(self._set_collide_x())
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def test_collision_dy(self):
        assert(self._set_collide_y())
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def test_collision_dz(self):
        assert(self._set_collide_z())
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_nxopen', TestNextageopen)
