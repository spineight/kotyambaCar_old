#! /usr/bin/env python

import rospy
from kotyambaCar.msg import movement_command 

def callback(data):
    rospy.loginfo("movement_command: speed_dc: {}, steer_dc: {}, active_time_sec: {}".format(speed_dc, steer_dc, active_time_sec))

def listener():
    rospy.init_node("command_listener",anonymous=True)
    rospy.Subscriber("command_center", movement_command, movement_command)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

