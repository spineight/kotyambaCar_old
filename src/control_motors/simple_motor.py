#!/usr/bin/python

# use GPIO library to work with Raspberry Pi GPIO
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

# Assign pins ids to variables with descriptive names
## Motor A (Speed control)
MotorA_direction_pin0 = 7
MotorA_direction_pin1 = 8
MotorA_EnA_pin = 1

## Motor B (Steering control)
MotorB_steering_pin0 = 9
MotorB_steering_pin1 = 10
MotorB_EnB_pin = 11

## Setup pins mode to output
## We will control motors by setting output voltage (Logical 1 / 0) on these pins
GPIO.setup(MotorA_direction_pin0, GPIO.OUT)
GPIO.setup(MotorA_direction_pin1,GPIO.OUT)
GPIO.setup(MotorA_EnA_pin,GPIO.OUT)

GPIO.setup(MotorB_steering_pin0,GPIO.OUT)
GPIO.setup(MotorB_steering_pin1,GPIO.OUT)
GPIO.setup(MotorB_EnB_pin,GPIO.OUT)

## You can change isForward, isLeft to false and observe how behaviour changes

print "Turning motor A. Speed control"
isForward = True
## For motor to rotate on pin0 and pin1 should be different voltages
## On one shoulde be 1 on another 0
GPIO.output(MotorA_direction_pin0, isForward)
GPIO.output(MotorA_direction_pin1, not isForward)
GPIO.output(MotorA_EnA_pin, GPIO.HIGH)
sleep(5)
print "Stopping motor A"
GPIO.output(MotorA_direction_pin0, not isForward)

print "Turning motor B. Steering control"
isLeft = True
## For motor to rotate on pin0 and pin1 should be different voltages
## On one shoulde be 1 on another 0
GPIO.output(MotorB_steering_pin0, isLeft)
GPIO.output(MotorB_steering_pin1, not isLeft)
GPIO.output(MotorB_EnB_pin, GPIO.HIGH)
sleep(2)
print "Stopping motor B"
GPIO.output(MotorB_steering_pin0, not isLeft)

GPIO.cleanup()
