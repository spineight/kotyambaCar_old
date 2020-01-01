#!/usr/bin/env python
import rospy
from kotyambaCar.msg import ultrasonic_distance 

import RPi.GPIO as GPIO
import time

class Ultrasonic_sensor():
    def __init__(self):
        print "##ROS##\n. Starting ultrasonic_sensor_node. Publishing to ultrasonic_distance topic\n##ROS##"
        pub = rospy.Publisher('ultrasonic_distance', ultrasonic_distance)
        rospy.init_node('ultrasonic_sensor_node', anonymous=True)
        frequency = 10#rospy.get_param('/ultrasonic_sensor/frequency')
        r = rospy.Rate(frequency) #hz
        msg = ultrasonic_distance()

        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        #set GPIO Pins
        self.GPIO_TRIGGER = 4
        self.GPIO_ECHO = 17

        #set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        while not rospy.is_shutdown():
            msg.distance_cm = self.distance()
            rospy.loginfo(msg)
            pub.publish(msg)
            r.sleep()

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)
    
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
    
        StartTime = time.time()
        StopTime = time.time()
    
        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()
    
        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()
    
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2.
    
        return distance

if __name__ == '__main__':
    try:
        ultrasonic_sensor = Ultrasonic_sensor()
    except rospy.ROSInterruptException: 
      print "Execution was interrupted"