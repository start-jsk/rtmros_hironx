#!/usr/bin/env python
# -*- coding: utf-8 -*-

from test_hironx_ros_bridge import *

class TestHiroROSBridgeSend(TestHiroROSBridge):

    def test_send_goal_and_wait(self):
        #self.rarm.send_goal(self.setup_Positions(self.goal_LArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]])) this should returns error
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.joint_states = []
        time.sleep(1.0);
        tm0 = rospy.Time.now()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
        self.rarm.wait_for_result()
        tm1 = rospy.Time.now()
        data_time = (tm1 - tm0).to_sec()
        filename = self.filename_base + "-wait"
        data = self.check_q_data(filename)
        min_data = min([d[1] for d in data])
        max_data = max([d[1] for d in data])
        print "check setJointAnglesOfGroup(wait=True),  tm = ", data_time, ", ok?", abs(data_time - 15.0) < 0.1
        self.assertTrue(abs(data_time - 15.0) < 1.0)
        print "                                        min = ", min_data, ", ok?", abs(min_data - -140) < 5
        self.assertTrue(abs(min_data - -140) < 5)
        print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
        self.assertTrue(abs(max_data - -100) < 5)

    def test_send_goal_and_nowait(self):
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -120, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.joint_states = []
            rospy.sleep(1.0)
            tm0 = rospy.Time.now()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
            rospy.sleep(clear_time[i]);
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            tm1 = rospy.Time.now()
            rospy.sleep(1.0)
            filename = self.filename_base + "-no-wait-"+str(clear_time[i])
            data = self.check_q_data(filename)
            data_time = (tm1 - tm0).to_sec()
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(wait=False), tm = ", data_time, ", ok?", abs(data_time - (10.0 - (5 - clear_time[i]))) < 1.5, " ", (10.0 - (5 - clear_time[i]))
            self.assertTrue(abs(data_time - (10.0 - (5 - clear_time[i]))) < 1.5)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+i*40/len(clear_time))) < 20, " ", -140+i*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+i*40/len(clear_time))) < 20)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

    def test_send_goal_and_clear(self):
        clear_time = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5]
        clear_time = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
        for i in range(len(clear_time)):
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.joint_states = []
            rospy.sleep(1.0)
            tm0 = rospy.Time.now()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -100, 15.2, 9.4, 3.2]], 5))
            self.rarm.wait_for_result()
            self.rarm.send_goal(self.setup_Positions(self.goal_RArm(), [[-0.6, 0, -140, 15.2, 9.4, 3.2]], 5))
            rospy.sleep(clear_time[i])
            self.rarm.cancel_goal()
            self.rarm.wait_for_result()
            tm1 = rospy.Time.now()
            rospy.sleep(1.0)
            filename = self.filename_base + "-clear-"+str(clear_time[i])
            data = self.check_q_data(filename)
            data_time = (tm1 - tm0).to_sec()
            min_data = min([d[1] for d in data])
            max_data = max([d[1] for d in data])
            print "check setJointAnglesOfGroup(clear "+str(clear_time[i])+"),  tm = ", data_time, ", ok?", abs(data_time - (10 - (5 - clear_time[i]))) < 0.5, " ", (10 - (5 - clear_time[i]))
            self.assertTrue(abs(data_time - (10 - (5 - clear_time[i]))) < 0.5)
            print "                                        min = ", min_data, ", ok?", abs(min_data - (-140+(i+1)*40/len(clear_time))) < 35, " ", -140+(i+1)*40/len(clear_time)
            self.assertTrue(abs(min_data - (-140+(i+1)*40/len(clear_time))) < 35)
            print "                                        max = ", max_data, ", ok?", abs(max_data - -100) < 5
            self.assertTrue(abs(max_data - -100) < 5)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge_send', TestHiroROSBridgeSend)



