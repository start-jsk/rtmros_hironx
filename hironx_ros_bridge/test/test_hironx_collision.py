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
_DIFF_THRESHOLD = 0.001
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

    def _set_collide(self, dx = 0, dy = 0, dz = 0):
        self._robot.seq_svc.setMaxIKError(0.00001, 0.01)
        init_pose = self._robot.getCurrentPosition(_LINK_TESTED)
        self._robot.setTargetPoseRelative(_ARMGROUP_TESTED, _LINK_TESTED, dx, dy, dz, tm = 1)
        curr_pose = self._robot.getCurrentPosition(_LINK_TESTED)

        if (abs(init_pose[0] - curr_pose[0] + dx) > _DIFF_THRESHOLD) or (abs(init_pose[1] - curr_pose[1] + dy) > _DIFF_THRESHOLD) or (abs(init_pose[2] - curr_pose[2] + dz) > _DIFF_THRESHOLD):
            print('Collision Detector enabled\n')
            return True
        else:
            print('Collision Detector NOT enabled\n')
            return False

    # def test_collision_dx(self):
    #     assert(self._set_collide(dx = -0.3))
    #     self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    def test_collision_dy(self):
        assert(self._set_collide(dy = -0.4))
        self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

    # def test_collision_dz(self):
    #     assert(self._set_collide(dz = -0.3))
    #     self._robot.goInitial(_GOINITIAL_TIME_MIDSPEED)

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKG, 'test_collide', TestNextageopen)
