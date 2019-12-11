#!/usr/bin/env python
import rospy
from kotyambaCar.msg import movement_command 

def command_center():
    ''' STUB while we don't have any logic, move in circle '''

    ## TODO Choosing a good queue_size:
    ## http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    pub = rospy.Publisher('command_center_commands', movement_command, queue_size = 3)
    rospy.init_node('command_center', anonymous=True)

    # http://wiki.ros.org/rospy/Overview/Time
    rateHz = 0.1
    r = rospy.Rate(rateHz)
    msg = movement_command()
    msg.speed_dc = 80
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