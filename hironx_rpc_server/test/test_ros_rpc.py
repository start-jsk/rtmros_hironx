#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy

from hironx_ros_bridge.rpc import HironxRPC
from hironx_ros_bridge.sample_rpc import SampleClientHironxRPC
from test_hironx_ros_bridge import *


class TestHiroROSBridgeRPC(TestHiroROSBridge):

    def setUp(self):
        '''
        Overriding because we need to instantiate ROS action server using
        hironx_ros_bridge.rpc.HironxRPC class.
        '''
        # Start an action server that handles various ROS Actions.
        self.sample_rpc = SampleClientHironxRPC(robot="HiroNX(Robot)0")

        # Not sure if 3 sec is safe enough but this should not be run on,
        # real robot so should be ok.
        self.sample_rpc.goInitial(tm=3)

    def tearDown(self):
        True  # TODO Run practical teardown.

    def test_rpc_checkEncoders(self):
        '''
        The targeted method requires interaction on a popup window so it's
        possible that a test hangs with no input. Thus no action for this.
        '''
        pass

    def test_rpc_getCurrentPose(self):
        '''
        pose:
          tm:
            sec: 1488517639
            nsec: 519280000
          data: [-0.0017702356144599085, 0.00019034630541264752, -0.9999984150158207, 0.32556275164378523, 0.00012155879975329215, 0.9999999745367515, 0.0001901314142046251, 0.18236394191140365, 0.9999984257434246, -0.00012122202968358842, -0.001770258707652326, 0.07462472659364472, 0.0, 0.0, 0.0, 1.0]
        '''
        pose_initpose = [-0.0017702356144599085, 0.00019034630541264752, -0.9999984150158207, 0.32556275164378523, 0.00012155879975329215, 0.9999999745367515, 0.0001901314142046251, 0.18236394191140365, 0.9999984257434246, -0.00012122202968358842, -0.001770258707652326, 0.07462472659364472, 0.0, 0.0, 0.0, 1.0]
        ret_srv = self.sample_rpc.sample_getCurrentPose()
        self.assertTrue(ret_srv.operation_return)
        numpy.testing.assert_almost_equal(pose_initpose, ret_srv.pose.data, 3)

    def test_rpc_getReferencePose(self):
        '''
        Expected value (after goIntial):
            [-0.0017695671419776793, 0.00019009187609556055, -0.9999984162473501,
             0.3255627368715471, 0.00012151860756957206, 0.99999997458995,
             0.00018987713664875283, 0.1823638733778268, 0.9999984269314416,
             -0.0001211824147721042, -0.001769590196713035, 0.07462449717662004,
             0.0, 0.0, 0.0, 1.0]
        '''
        pose_initpose = [
            -0.0017695671419776793, 0.00019009187609556055, -0.9999984162473501,
            0.3255627368715471, 0.00012151860756957206, 0.99999997458995,
            0.00018987713664875283, 0.1823638733778268, 0.9999984269314416,
            -0.0001211824147721042, -0.001769590196713035, 0.07462449717662004,
            0.0, 0.0, 0.0, 1.0]
        ret_srv = self.sample_rpc.sample_getReferencePose()
        self.assertTrue(ret_srv.operation_return)
        numpy.testing.assert_almost_equal(pose_initpose, ret_srv.pose, 3)

    def _test_rpc_getJointAngles(self):
        '''Stashed as of 20170201'''
        ret_srv = self.sample_rpc.sample_getJointAngles()

        self.assertTrue(ret_srv.operation_return)
        self.assertEqual(24, len(ret_srv.jvs),
                         '(as of Mar 2017) this returns 24 joints for Hironx.')

    def test_rpc_goActual(self):
        self.assertTrue(self.sample_rpc.goActual())

    def test_rpc_goInitial(self):
        self.assertTrue(self.sample_rpc.goInitial())

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hironx_rpc', TestHiroROSBridgeRPC)
