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

## TODO: play with values of:
# 1. recommendedFreq
# 2. DC (should be between 0 and 100)
# 3. GPIO.HIGH & GPIO.LOW values for pins
try :
  print "Turning motor A. Speed control"
  ## For motor to rotate on MotorA_direction_pin0 and MotorA_direction_pin1 should be opposite voltages (HIGH vs LOW)
  ## On one shoulde be 1 on another 0
  GPIO.output(MotorA_direction_pin0, GPIO.HIGH)
  GPIO.output(MotorA_direction_pin1, GPIO.LOW)
  recommendedFreq = 400
  DC = 90
  action_time_sec = 5
  speedPWM = GPIO.PWM(MotorA_EnA_pin, recommendedFreq)
  speedPWM.start(0)
  speedPWM.ChangeDutyCycle(DC)
  sleep(action_time_sec)

  print "Stopping motor A"
  GPIO.output(MotorA_direction_pin0, GPIO.LOW)

  print "Turning motor B. Steering control"
  GPIO.output(MotorB_steering_pin0, GPIO.HIGH)
  GPIO.output(MotorB_steering_pin1, GPIO.LOW)
  recommendedFreq = 400
  DC = 80
  action_time_sec = 5
  directionPWM = GPIO.PWM(MotorB_EnB_pin, recommendedFreq)
  directionPWM.start(0)
  directionPWM.ChangeDutyCycle(DC)
  sleep(action_time_sec)

  print "Stopping motor B"
  GPIO.output(MotorB_steering_pin0, GPIO.LOW)
except Exception as e:
  print e
finally:
  GPIO.cleanup()


