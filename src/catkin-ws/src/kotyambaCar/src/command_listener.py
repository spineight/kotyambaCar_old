#! /usr/bin/env python

# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
import sys
sys.path.append("/home/pi/kotyambaCar/src/control_motors") # Adds higher directory to python modules path.
from motor_pwm import Motor
from vehicle import Vehicle

import rospy
from kotyambaCar.msg import movement_command 
    
class Command_listener:
    def __init__(self):
        SpeedControlMotor = Motor(7,8,1)
        SteerControlMotor = Motor(9,10,11)
        self.car = Vehicle(SpeedControlMotor, SteerControlMotor)
        print "starting command_listener node"
        rospy.init_node("command_listener",anonymous=True)
        rospy.Subscriber("command_center_commands", movement_command, self.on_movement_command)
    
        rospy.spin()

    def on_movement_command(data):
        msg = "movement_command: speed_dc: {}, steer_dc: {}, active_time_sec: {}".format(data.speed_dc, data.steer_dc, data.active_time_sec)
        rospy.loginfo(msg)
        print msg
        if(data.speed_dc > 0):
            self.car.moveForwardAsync(speed_dc, steer_dc, data.active_time_sec)
        elif(data.speed_dc <0):
            self.car.moveBackwardAsync(speed_dc, steer_dc, data.active_time_sec)

if __name__ == '__main__':
    listener = Command_listener()

# /Users/mac/remotePiMount/src/catkin-ws/src/kotyambaCar/src/command_listener.py
# /Users/mac/remotePiMount/src/control_motors