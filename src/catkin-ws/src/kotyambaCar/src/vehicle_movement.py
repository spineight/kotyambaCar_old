#! /usr/bin/env python

import os
import sys
# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
sys.path.append("{}/src/control_motors".format(os.environ["KOTYAMBA_REPO_RASPBERRY"].rstrip()))
print "added paths to *.py files to PYTHON_PATH:"
print(sys.path)

from motor_pwm import Motor
from vehicle import Vehicle

import rospy
from geometry_msgs.msg import Twist
    
class Vehicle_movement_node:
    def __init__(self):
        SpeedControlMotor = Motor(7,8,1,100)
        SteerControlMotor = Motor(10,9,11,2)
        self.car = Vehicle(SpeedControlMotor, SteerControlMotor)
        print "##ROS##\n. Starting movement_node. Subscribed to movement_command topic\n##ROS##"
        rospy.init_node("Vehicle_movement_node") # removed ,anonymous=True to be able to kill it by name
        rospy.Subscriber("kotyamba/cmd_vel", Twist, self.on_twist_command)
    
        rospy.spin()
    def __del__(self):
        self.car.stop()

    def on_twist_command(self,twist):
        '''  Up/Down Axis stick left twist.linear.x
            Left/Right Axis stick right twist.angular.z
        '''
        msg = "twist cmd: twist.linear.x: {}, twist.angular.z: {}".format(twist.linear.x, twist.angular.z)

        rospy.loginfo(twist)
        print twist
        if(twist.linear.x == 0 and twist.angular.z == 0):
            car.on_stop()
        break
        elif(twist.linear.x > 0):
            car.on_speed_change(speed_dc_step)
        elif(twist.linear.x < 0):
            car.on_speed_change(-speed_dc_step)
        elif(twist.angular.z < 0):
            car.on_steering_change(-steering_dc_step)
        elif(twist.angular.z > 0):
            car.on_steering_change(steering_dc_step)

if __name__ == '__main__':
    try:
        listener = Movement_node()
    except rospy.ROSInterruptException:
        print "Command listener was interrupted"
