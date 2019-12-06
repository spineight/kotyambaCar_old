#!/usr/bin/env python
import rospy
from kotyambaCar.msg import ultrasonic_distance 

def ultrasonic_sensor():
    pub = rospy.Publisher('ultrasonic_distance', ultrasonic_distance)
    rospy.init_node('ultrasonic_distance', anonymous=True)
    frequency = rospy.get_param('~frequency')
    r = rospy.Rate(frequency) #hz
    msg = ultrasonic_distance()
    msg.distance_cm = 80

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        ultrasonic_sensor()
    except rospy.ROSInterruptException: 
      print "Execution was interrupted"