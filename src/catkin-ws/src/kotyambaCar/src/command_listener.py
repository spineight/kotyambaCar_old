#! /usr/bin/env python

# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
import os
import sys
print os.environ['KOTYAMBA_REPO_RASPBERRY']
sys.path.append("{}/src/control_motors".format(os.environ['KOTYAMBA_REPO_RASPBERRY'])) # Adds higher directory to python modules path.
from motor_pwm import Motor
from vehicle import Vehicle

import rospy
from kotyambaCar.msg import movement_command 
    
class Command_listener:
    def __init__(self):
        pwm_frequency = 200
        SpeedControlMotor = Motor(7,8,1, pwm_frequency)
        SteerControlMotor = Motor(9,10,11, pwm_frequency)
        self.car = Vehicle(SpeedControlMotor, SteerControlMotor)
        print "starting command_listener node"
        rospy.init_node("command_listener") # removed ,anonymous=True to be able to kill it by name
        rospy.Subscriber("command_center_commands", movement_command, self.on_movement_command)
    
        rospy.spin()

    def on_movement_command(self,data):
        msg = "movement_command: speed_dc: {}, steer_dc: {}, active_time_sec: {}".format(data.speed_dc, data.steer_dc, data.active_time_sec)
        rospy.loginfo(msg)
        print msg
        if(data.speed_dc > 0):
            self.car.moveForwardAsync(data.speed_dc, data.steer_dc, data.active_time_sec)
        elif(data.speed_dc <0):
            self.car.moveBackwardAsync(data.speed_dc, data.steer_dc, data.active_time_sec)

if __name__ == '__main__':
    listener = Command_listener()
