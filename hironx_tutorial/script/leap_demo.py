#!/usr/bin/env python
__author__ = 'flier'

import argparse
import sys
import time

import rospy

from nextage_ros_bridge.nextage_client import NextageClient
from leap_motion.msg import leap
from leap_motion.msg import leaprosWY

# Native datatypes, I've heard this is bad practice, use the geometry messages instead.
# def callback(data):
#    rospy.loginfo(rospy.get_name() + ": Leap Raw Data %s" % data)

counter = 0
prev_sig = 0
STATE_FIRST = 0
STATE_SECOND = 1
STATE_FINAL = 2
state_curr = STATE_FIRST

# Callback of the ROS subscriber, just print the received data.
def callback_ros(data):
    global counter
    global state_curr

    rospy.logdebug(rospy.get_name() + ": Leap ROS Data %s" % data)
    sig_palm = data.palm_dir_on_off
    if state_curr == STATE_FIRST:
        rospy.loginfo('state 1 COUNT: {}'.format(counter))
        if sig_palm == -1:
            counter = counter + 1
        else:
            counter = 0
            rospy.loginfo('state 1 counter RESET')
        if 50 < counter:
            state_curr = STATE_SECOND
            rospy.loginfo('state 1 reached')

    elif state_curr == STATE_SECOND:
        if sig_palm == 1:
            counter += 1
        else:
            counter = 0
            rospy.loginfo('state 222222 counter RESET')

        if 50 < counter:
            state_curr = STATE_FINAL
            rospy.loginfo('state 2 reached')

    elif state_curr == STATE_FINAL:
        if sig_palm == -1:
            counter += 1
        else:
            counter = 0
            rospy.loginfo('state ****** 333 ****** counter RESET')

        if 50 < counter:
            #TODO: send sig to robot
            rospy.loginfo('REACHED GOAL. Exiting.')
            rospy.signal_shutdown('REACHED GOAL. Exiting.')
            sys.exit(1)


# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    rospy.init_node('leap_sub', anonymous=True)
    # rospy.Subscriber("leapmotion/raw", leap, callback)
    rospy.Subscriber("leapmotion/data", leaprosWY, callback_ros)
    rospy.spin()


if __name__ == '__main__':
    listener()
