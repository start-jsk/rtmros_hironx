#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy

from hironx_rpc_server.sample_rpc import SampleClientHironxRPC
from hironx_ros_bridge.testutil.test_hironx_ros_bridge import TestHiroROSBridge

PKG = 'hironx_rpc_server'


class TestHiroROSBridgeRPC(TestHiroROSBridge):
    _JOINT_TARGET = 'LARM_JOINT5'
    _POSITION_LJOINT5_INIT = (0.3255627368715471, 0.1823638733778268, 0.07462449717662004)
    _ROTATION_LJOINT5_INIT = [[-0.0017695671419776793, 0.00019009187609556055, -0.9999984162473501],
         [0.00012151860756957206, 0.99999997458995, 0.00018987713664875283],
         [0.9999984269314416, -0.0001211824147721042, -0.001769590196713035]]
    _RPY_LJOINT5_INIT = (-3.0732189053889805, -1.5690225912054285, 3.0730289207320203)

    # Start an action server that handles various ROS Actions.
    _sample_rpc = SampleClientHironxRPC()

    def setUp(self):
        '''
        Overriding because we need to instantiate ROS action server using
        hironx_ros_bridge.rpc.HironxRPC class.
        '''

        # Not sure if 3 sec is safe enough but this should not be run on,
        # real robot so should be ok.
        self._sample_rpc.sample_goInitial(tm=3)

    def tearDown(self):
        True  # TODO Run practical teardown.

    def test_rpc_checkEncoders(self):
        '''
        The targeted method requires interaction on a popup window so it's
        possible that a test hangs with no input. Thus no action for this.
        '''
        pass

    def _create_multidimensional_list(self, source):
        '''
        @type source: Either tork_rpc_util/Int8List[] or
                      tork_rpc_util/Float32List[]
        '''
        list_servoState = []
        for each_servo in source:
            list_val_each_servo = []
            for each_val in each_servo.data:
                list_val_each_servo.append([each_val])
            list_servoState.extend(list_val_each_servo)
        return list_servoState

    def test_rpc_getActualState(self):
        '''
        In [2]: robot.getActualState()
        Out[2]: OpenHRP.RobotHardwareService.RobotState(angle=[0.0, 0.0, 0.0, 0.0, -0.010471975511965976, 0.0, -1.7453292519943295, 0.26529004630313807, 0.16406094968746698, 0.05585053606381855, 0.0, 0.0, 0.0, 0.0, 0.010471975511965976, 0.0, -1.7453292519943295, -0.26529004630313807, 0.16406094968746698, -0.05585053606381855, 0.0, 0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0, 0.0, -0.010471975511965976, 0.0, -1.7453292519943295, 0.26529004630313807, 0.16406094968746698, 0.05585053606381855, 0.0, 0.0, 0.0, 0.0, 0.010471975511965976, 0.0, -1.7453292519943295, -0.26529004630313807, 0.16406094968746698, -0.05585053606381855, 0.0, 0.0, 0.0, 0.0], torque=[], servoState=[[7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7]], force=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], rateGyro=[], accel=[], voltage=0.0, current=0.0)
        '''
        expected_value = dict(
            angle=[0.0, 0.0, 0.0, 0.0, -0.010471975511965976, 0.0,
                   -1.7453292519943295, 0.26529004630313807,
                   0.16406094968746698, 0.05585053606381855, 0.0, 0.0, 0.0,
                   0.0, 0.010471975511965976, 0.0, -1.7453292519943295,
                   -0.26529004630313807, 0.16406094968746698,
                   -0.05585053606381855, 0.0, 0.0, 0.0, 0.0],
            command=[0.0, 0.0, 0.0, 0.0, -0.010471975511965976, 0.0,
                     -1.7453292519943295, 0.26529004630313807,
                     0.16406094968746698, 0.05585053606381855, 0.0, 0.0, 0.0,
                     0.0, 0.010471975511965976, 0.0, -1.7453292519943295,
                     -0.26529004630313807, 0.16406094968746698,
                     -0.05585053606381855, 0.0, 0.0, 0.0, 0.0],
            torque=[],
            servoState=[[7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7],
                        [7], [7], [7], [7], [7], [7], [7], [7], [7], [7], [7],
                        [7], [7]],
            force=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            rateGyro=[], accel=[], voltage=0.0, current=0.0)
        ret_srv = self._sample_rpc.sample_getActualState()
        # Manually workaround the output format difference of multi-dimensional
        # entries in RobotaState.srv.
        list_servoState = self._create_multidimensional_list(ret_srv.servoState)
        list_force = self._create_multidimensional_list(ret_srv.force)
        numpy.testing.assert_almost_equal(expected_value['angle'], ret_srv.angle, 3)
        numpy.testing.assert_almost_equal(expected_value['command'], ret_srv.command, 3)
        numpy.testing.assert_almost_equal(expected_value['torque'], ret_srv.torque, 3)
        numpy.testing.assert_almost_equal(expected_value['servoState'], list_servoState, 3)
        numpy.testing.assert_almost_equal(expected_value['rateGyro'], ret_srv.rateGyro, 3)
        numpy.testing.assert_almost_equal(expected_value['accel'], ret_srv.accel, 3)
        numpy.testing.assert_almost_equal(expected_value['voltage'], ret_srv.voltage, 3)
        numpy.testing.assert_almost_equal(expected_value['current'], ret_srv.current, 3)

    def test_rpc_getCurrentPose(self):
        '''
        pose:
          tm:
            sec: 1488517639
            nsec: 519280000
          data: [-0.0017702356144599085, 0.00019034630541264752, -0.9999984150158207, 0.32556275164378523, 0.00012155879975329215, 0.9999999745367515, 0.0001901314142046251, 0.18236394191140365, 0.9999984257434246, -0.00012122202968358842, -0.001770258707652326, 0.07462472659364472, 0.0, 0.0, 0.0, 1.0]
        '''
        pose_initpose = [-0.0017702356144599085, 0.00019034630541264752, -0.9999984150158207, 0.32556275164378523, 0.00012155879975329215, 0.9999999745367515, 0.0001901314142046251, 0.18236394191140365, 0.9999984257434246, -0.00012122202968358842, -0.001770258707652326, 0.07462472659364472, 0.0, 0.0, 0.0, 1.0]
        ret_srv = self._sample_rpc.sample_getCurrentPose()
        self.assertTrue(ret_srv.operation_return)
        numpy.testing.assert_almost_equal(pose_initpose, ret_srv.pose.data, 3)

    def test_rpc_getCurrentPosition(self):
        ret = self._sample_rpc.sample_getCurrentPosition(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._POSITION_LJOINT5_INIT,
                                          ret, 3)

    def test_rpc_getCurrentRotation(self):
        ret = self._sample_rpc.sample_getCurrentRotation(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._ROTATION_LJOINT5_INIT,
                                          ret, 3)

    def test_rpc_getCurrentRPY(self):
        ret = self._sample_rpc.sample_getCurrentRPY(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._RPY_LJOINT5_INIT,
                                          ret, 3)

    def test_rpc_groups(self):
        expected_value = [
            ['torso', ['CHEST_JOINT0']],
            ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
            ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']],
            ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]]
        ret = self._sample_rpc.sample_groups()
        self.assertListEqual(expected_value, ret)

    def test_rpc_getJointAngles(self):
        expected_value = [0.0, 0.0, 0.0, 0.0, -0.6, 0.0, -100.0, 15.2, 9.4,
                          3.2, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0, -100.0, -15.2,
                          9.4, -3.2, 0.0, 0.0, 0.0, 0.0]  # Initial pose.
        ret = self._sample_rpc.sample_getJointAngles()
        numpy.testing.assert_almost_equal(expected_value, ret, 3)

    def _test_rpc_getJointAngles(self):
        '''Stashed as of 20170201'''
        ret_srv = self._sample_rpc.sample_getJointAngles()

        self.assertTrue(ret_srv.operation_return)
        self.assertEqual(24, len(ret_srv.jvs),
                         '(as of Mar 2017) this returns 24 joints for Hironx.')

    def test_rpc_getReferencePosition(self):
        ret = self._sample_rpc.sample_getReferencePosition(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._POSITION_LJOINT5_INIT,
                                          ret, 3)

    def test_rpc_getReferenceRotation(self):
        ret = self._sample_rpc.sample_getReferenceRotation(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._ROTATION_LJOINT5_INIT,
                                          ret, 3)

    def test_rpc_getReferenceRPY(self):
        ret = self._sample_rpc.sample_getReferenceRPY(self._JOINT_TARGET)
        numpy.testing.assert_almost_equal(self._RPY_LJOINT5_INIT,
                                          ret, 3)

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
        ret_srv = self._sample_rpc.sample_getReferencePose()
        self.assertTrue(ret_srv.operation_return)
        numpy.testing.assert_almost_equal(pose_initpose, ret_srv.pose.data, 3)

    def _test_rpc_getRTCList(self):
        expected_value = [
            ['seq', 'SequencePlayer'],
            ['sh', 'StateHolder'],
            ['fk', 'ForwardKinematics'],
            ['ic', 'ImpedanceController'],
            ['el', 'SoftErrorLimiter'],
            ['sc', 'ServoController'],
            ['log', 'DataLogger'],
            ['rmfo', 'RemoveForceSensorLinkOffset']]
        ret = self._sample_rpc.sample_getRTCList()
        self.assertListEqual(expected_value, ret)

    def _test_rpc_getSensors(self):
        expected_value = [
            "OpenHRP.SensorInfo(type='Vision', name='CAMERA_HEAD_R', id=0, translation=[0.059959, -0.069754, 0.095], rotation=[0.5575793136150866, -0.4975428112921299, -0.6644971481955756, 1.906030089009999], specValues=[0.05000000074505806, 10.0, 0.5777000188827515, 1.0, 640.0, 480.0, 30.00029945373535], specFile='', shapeIndices=[], inlinedShapeTransformMatrices=[])",
            "OpenHRP.SensorInfo(type='Vision', name='CAMERA_HEAD_L', id=1, translation=[0.059959, 0.069754, 0.095], rotation=[0.4824954118724479, -0.5407162028626804, -0.6890748620315513, 1.9967351576399999], specValues=[0.05000000074505806, 10.0, 0.5777000188827515, 1.0, 640.0, 480.0, 30.00029945373535], specFile='', shapeIndices=[], inlinedShapeTransformMatrices=[])",
            "OpenHRP.SensorInfo(type='Force', name='rhsensor', id=1, translation=[0.0, 0.0, 0.0], rotation=[0.5773636022982372, 0.5773436025198526, 0.5773436025198526, 0.03655417585376924], specValues=[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], specFile='', shapeIndices=[], inlinedShapeTransformMatrices=[])",
            "OpenHRP.SensorInfo(type='Force', name='lhsensor', id=0, translation=[0.0, 0.0, 0.0], rotation=[0.5773636022982372, 0.5773436025198526, 0.5773436025198526, 0.03655417585376924], specValues=[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], specFile='', shapeIndices=[], inlinedShapeTransformMatrices=[])"]
        ret = self._sample_rpc.sample_getSensors()
        self.assertListEqual(expected_value, ret)

    def test_rpc_goActual(self):
        self.assertTrue(self._sample_rpc.sample_goActual())

    def test_rpc_goInitial(self):
        self.assertTrue(self._sample_rpc.sample_goInitial())

    def test_rpc_isServoOn(self):
        ''''''
        self.assertEqual(1, self._sample_rpc.sample_isServoOn())

    # No pattern file for HiroNXO is found, so we can't yet test this func.
    #def test_loadPatter(self):
    #    pass

    def test_rpc_servoOff(self):
        '''
        On simulation, all servos are always on, so no tests running here.
        See https://github.com/start-jsk/rtmros_hironx/blob/f523a54e9cd96af54a29d5613730b91600eb95a1/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py#L739
        '''
        pass

    def test_rpc_servoOn(self):
        ''''''
        self.assertEqual(1, self._sample_rpc.sample_servoOn())

    def test_rpc_setJointAngle(self):
        self._sample_rpc.sample_setJointAngle()
        expected_pos = [0.0, 0.0, 0.5695]
        # TODO getCurrentPosition('HEAD_JOINT1')
        #self.assertAlmostEqual(first, second, places, msg, delta)
        pass

    def test_rpc_setJointAngles(self):
        expected_value = [0.0, 0.0, 0.0, 10.0, -0.600001403060998,
                          0.0, -100.00004285756798, 15.19999734702561,
                          9.4000028826958, 3.2000265815851607, 0.0, 0.0, 0.0,
                          0.0, 0.600001403060998, 0.0, -100.00004285756798,
                          -15.19999734702561, 9.4000028826958, -3.2000265815851607,
                          0.0, 0.0, 0.0, 0.0]
        self._sample_rpc.sample_setJointAngles(angles=expected_value)
        # TODO getCurrentPosition('HEAD_JOINT1')
        #self.assertAlmostEqual(first, second, places, msg, delta)
        pass

    def test_rpc_setJointAnglesOfGroup(self):
        expected_value = [0, 0.0, -130.0, 0, 0.0, 0.0]  # init factory pose

        self._sample_rpc.sample_setJointAnglesOfGroup()
        # TODO getCurrentPosition('HEAD_JOINT1')
        #self.assertAlmostEqual(first, second, places, msg, delta)
        pass

    def test_rpc_setTargetPose(self):
        expected_value = [0.0, 0.0, 0.0, 0.0, -0.6, 0.0, -100.0, 15.2, 9.4,
                          3.2, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0, -100.0, -15.2,
                          9.4, -3.2, 0.0, 0.0, 0.0, 0.0]  # init factory pose
        self._sample_rpc.sample_setTargetPose()
        ret = self._sample_rpc.sample_getJointAngles()
        numpy.testing.assert_almost_equal(expected_value, ret, 3)

    def test_rpc_waitInerpolation(self):
        #TODO Move left arm.
        #TODO getCurrentPose of right arm.
        self._sample_rpc.sample_waitInerpolation()
        #TODO getCurrentPose of right arm again
        # self.assertTrue() if the right arm stays at the same pose.
        #TODO move R arm.
        pass

    def test_rpc_waitInerpolationOfGroups(self):
        #TODO Move left arm.
        #TODO getCurrentPose of right arm.
        self._sample_rpc.sample_waitInerpolationOfGroups()
        #TODO getCurrentPose of right arm again
        # self.assertTrue() if the right arm stays at the same pose.
        #TODO move R arm.
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hironx_rpc', TestHiroROSBridgeRPC)
