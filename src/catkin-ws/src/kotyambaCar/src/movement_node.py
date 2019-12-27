#! /usr/bin/env python

import os
import sys
# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
sys.path.append("{}/src/control_motors".format(os.environ["KOTYAMBA_REPO_RASPBERRY"].rstrip()))
print "added paths to *.py files to PYTHON_PATH:"
print(sys.path)

from motor_pwm import Motor
from vehicle import vehicle

import rospy
from kotyambaCar.msg import movement_command
    
class Movement_node:
    def __init__(self):
        self.car = vehicle
        print "##ROS##\n. Starting movement_node. Subscribed to movement_command topic\n##ROS##"
        rospy.init_node("movement_node") # removed ,anonymous=True to be able to kill it by name
        rospy.Subscriber("movement_command", movement_command, self.on_movement_command)
    
        rospy.spin()
    def __del__(self):
        self.car.stop()

    def on_movement_command(self,cmd):
        msg = "movement_command: direction: {}, steer_dc: {}".format(cmd.direction, cmd.speed)
        rospy.loginfo(msg)
        print msg
        self.car.on_movement_command(cmd.direction, cmd.speed)

if __name__ == '__main__':
    try:
        listener = Movement_node()
    except rospy.ROSInterruptException:
        print "Command listener was interrupted"
