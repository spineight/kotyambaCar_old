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
from kotyambaCar.msg import movement_command
    
class Movement_node:
    def __init__(self):
        SpeedControlMotor = Motor("/home/pi/kotyambaCar/src/control_motors/speed_motor.yaml")
        SteerControlMotor = Motor("/home/pi/kotyambaCar/src/control_motors/steering_motor.yaml")
        self.car = Vehicle(SpeedControlMotor, SteerControlMotor)
        print "starting movement_node"
        rospy.init_node("movement_node") # removed ,anonymous=True to be able to kill it by name
        rospy.Subscriber("movement_command", movement_command, self.on_movement_command)
    
        rospy.spin()
    def __del__(self):
        self.car.stop()

    def on_movement_command(self,cmd):
        msg = "movement_command: speed_dc: {}, steer_dc: {}".format(cmd.speed_dc, cmd.steer_dc)
        rospy.loginfo(msg)
        print msg
        if(cmd.is_emergency_stop):
            print "emergency_stop"
            self.car.hardBreak_()
        elif(cmd.speed_dc > 0):
            self.car.moveForward_(cmd.speed_dc, cmd.steer_dc)
        elif(cmd.speed_dc <0):
            self.car.moveBackward_(abs(cmd.speed_dc), cmd.steer_dc)

if __name__ == '__main__':
    try:
        listener = Movement_node()
    except rospy.ROSInterruptException:
        print "Command listener was interrupted"
