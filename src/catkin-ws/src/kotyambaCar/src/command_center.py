#!/usr/bin/env python
import rospy
from kotyambaCar.msg import movement_command 

def command_center():
    pub = rospy.Publisher('command_center_commands', movement_command)
    rospy.init_node('command_center', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = movement_command()
    msg.speed_dc = 90
    msg.steer_dc = 90
    msg.active_time_sec = 1

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        command_center()
    except rospy.ROSInterruptException: pass