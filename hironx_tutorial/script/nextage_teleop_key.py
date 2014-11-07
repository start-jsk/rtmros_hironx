#!/usr/bin/env python

#############################################################################################
 # Copyright (c) 2014 Daiki Maekawa and ROS JAPAN Users Group All Rights Reserved.         #
 #                                                                                         #
 # @file nextage_teleop_key.py                                                             #
 # @brief Reading from the keyboard and Publishing to Pose.                                #
 # @author Daiki Maekawa                                                                   #
 # @date 2014-08-26                                                                        #
#############################################################################################

import rospy, geometry_msgs.msg
import sys, select, termios, tty


class TeleopNextage:
    MOVE_BINDINGS = {
        'h' : (1, 0.1),
        'j' : (0, -0.1),
        'k' : (0, 0.1),
        'l' : (1, -0.1),
    }

    MSG = """
        **********
        h : y += 0.1
        j : x += -0.1
        k : x += 0.1
        l : y += -0.1
        **********
    """
    
    def __init__(self):
        self._settings = termios.tcgetattr(sys.stdin)
        self._delta_pose_pub = rospy.Publisher('right_arm/delta_pose', geometry_msgs.msg.Pose)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        
        return key

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.get_key()
            print key
            if key in TeleopNextage.MOVE_BINDINGS.keys():
                move = TeleopNextage.MOVE_BINDINGS[key]
                diff_list = [move[1] if index is move[0] else 0 for index in range(7)]
            
                delta_pose = geometry_msgs.msg.Pose()
                delta_pose.position.x = diff_list[0]
                delta_pose.position.y = diff_list[1]
                delta_pose.position.z = diff_list[2]
                delta_pose.orientation.x = diff_list[3]
                delta_pose.orientation.y = diff_list[4]
                delta_pose.orientation.z = diff_list[5]
                delta_pose.orientation.w = diff_list[6]
                
                self._delta_pose_pub.publish(delta_pose)
            elif key == '\x03':
                print "Quit"
                break
            else:
                print "Unknown type"

            r.sleep()

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)

def main():
    rospy.init_node('teleop_pose_key')
    teleop_nextage = TeleopNextage()
    
    print TeleopNextage.MSG
    
    teleop_nextage.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

