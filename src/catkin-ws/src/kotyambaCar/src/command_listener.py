#! /usr/bin/env python

import rospy
from kotyambaCar.msg import movement_command 

def callback(data):
    print "got data"
def callback(data):
    msg = "movement_command: speed_dc: {}, steer_dc: {}, active_time_sec: {}".format(data.speed_dc, data.steer_dc, data.active_time_sec)
    rospy.loginfo(msg)
    print msg

def listener():
    print "starting command_listener node"
    rospy.init_node("command_listener",anonymous=True)
    rospy.Subscriber("command_center", movement_command, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

