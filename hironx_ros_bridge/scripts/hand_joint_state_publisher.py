#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def joint_state_publisher():
    rospy.init_node('hand_joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    joint_list = ['LHAND_JOINT0', 'LHAND_JOINT1', 'LHAND_JOINT2', 'LHAND_JOINT3', 'RHAND_JOINT0', 'RHAND_JOINT1', 'RHAND_JOINT2', 'RHAND_JOINT3']
    rospy.loginfo("{} publishes fake joint states of {}".format(rospy.get_name(), joint_list))
    rate = rospy.Rate(3) # 3hz
    while not rospy.is_shutdown():
        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = []
        joint_states.position = []
        for joint in joint_list:
            joint_states.name.append(joint)
            joint_states.position.append(0)
        rospy.logdebug(joint_states)
        pub.publish(joint_states)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
