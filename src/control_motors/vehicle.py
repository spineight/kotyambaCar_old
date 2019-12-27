#!/usr/bin/python
import threading
from threading import Thread
from time import sleep

from motor_pwm import Motor

import os

class Vehicle:
  ''' speed in 0..100% '''
  def __init__(self,speed_control_motor, steering_control_motor):
    self.speed_control_motor = speed_control_motor
    self.steering_control_motor = steering_control_motor
    self.cv = threading.Condition()

    self.steering_angle = 0

  # Asynch (to be able to interrupt commands by new commands 
  # (for ex. for collision avoidance, when new object suddenly appears in front of us)
  def stopAnotherCommands(self):
    self.cv.acquire()
    self.cv.notifyAll()
    self.cv.release()

  def on_movement_command(self,steer_cmd_str, speed_dc):
    active_time_sec = 1
    self.stopAnotherCommands()
    steer_dc = 0
    if(steer_cmd_str in "left_small"):
      steer_dc=-5
    elif(steer_cmd_str in "right_small"):
      steer_dc=5
    elif(steer_cmd_str in "left_medium"):
      steer_dc=-10
    elif(steer_cmd_str in "right_medium"):
      steer_dc=10
    elif(steer_cmd_str in "left_large"):
      steer_dc=-15
    elif(steer_cmd_str in "right_large"):
      steer_dc=15
    elif(steer_cmd_str in "straight"):
      steer_dc = -self.steering_angle
      self.steering_angle = 0
    elif(steer_cmd_str in "emergency_stop"):
      self.hardBreak_()
    self.steering_angle = self.steering_angle + steer_dc
    cmdThread = threading.Thread(target=self.steer_thread, args=(float(steer_dc), float(active_time_sec)))
    cmdThread.daemon = True
    self.last_thread_name = cmdThread.getName()
    cmdThread.start()

    if(speed_dc > 0):
      self.speed_control_motor.rotate(speed_dc,True)
    elif (speed_dc < 0):
      self.speed_control_motor.rotate(abs(speed_dc),False)


  def steer_thread(self, steer_dc, active_time_sec):
    ''' steering should be done within given time,
      after that when given angle achevied we should stop it
    '''
    try:
      if(steer_dc < 0):
        self.steering_control_motor.rotate(steer_dc,True) # left
      elif(steer_dc > 0):
        self.steering_control_motor.rotate(steer_dc,False) # right
      self.cv.acquire()
      self.cv.wait(active_time_sec)

      # if thread was interrupted don't stop the motor
      # for smooth commands chaining
      if(threading.current_thread().getName() == self.last_thread_name):
        self.speed_control_motor.stop()
        self.steering_control_motor.stop()
      self.cv.release() 
    except Exception as e:
      print "steer"
      print str(e)

  # def moveForward_(self, speed_dc, steer_dc):
  #   '''
  #     if steer_dc > 0 - move right, else - left 
  #   '''
  #   self.speed_control_motor.rotate(speed_dc,True)
  #   if(steer_dc >= 0):
  #     self.steering_control_motor.rotate(steer_dc,False) # right
  #   else:
  #     self.steering_control_motor.rotate(abs(steer_dc),True) # left

  # def moveBackward_(self, speed_dc, steer_dc):
  #   '''
  #     if steer_dc > 0 - move right, else - left 
  #   '''
  #   self.speed_control_motor.rotate(speed_dc,False)
  #   if(steer_dc >= 0):
  #     self.steering_control_motor.rotate(steer_dc,False) # right
  #   else:
  #     self.steering_control_motor.rotate(abs(steer_dc),True) # left

  def stop_(self):
    ''' soft stop, wheels can move'''
    self.speed_control_motor.stop()
    self.steering_control_motor.stop()

  def hardBreak_(self):
    ''' hard stop - wheels are blocked'''
    self.speed_control_motor.emergency_stop()
    self.steering_control_motor.emergency_stop()

  # def moveForward(self,speed,active_time_sec):
  #   ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
  #   self.speed_control_motor.rotate(speed,True)
  #   sleep(active_time_sec)
  #   self.speed_control_motor.stop()

  # def moveBackward(self,speed,active_time_sec):
  #   ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
  #   self.speed_control_motor.rotate(speed,False)
  #   sleep(active_time_sec)
  #   self.speed_control_motor.stop()

  # def turnLeft(self,speed, active_time_sec):
  #   ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
  #   self.speed_control_motor.rotate(speed,True)
  #   self.steering_control_motor.rotate(speed,True)
  #   sleep(active_time_sec)
  #   self.speed_control_motor.stop()
  #   self.steering_control_motor.stop()

  # def turnRight(self,speed, active_time_sec):
  #   ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
  #   self.speed_control_motor.rotate(speed,True)
  #   self.steering_control_motor.rotate(speed,False)
  #   sleep(active_time_sec)
  #   self.speed_control_motor.stop()
  #   self.steering_control_motor.stop()

  # def stop(self):
  #   ''' soft stop, wheels can move'''
  #   self.speed_control_motor.stop()
  #   self.steering_control_motor.stop()
  #   self.stopAnotherCommands()

  # def hardBreak(self):
  #   ''' hard stop - wheels are blocked'''
  #   self.speed_control_motor.emergency_stop()
  #   self.steering_control_motor.emergency_stop()
  #   self.stopAnotherCommands()

  # ## for more precise control 
  # def moveForwardPrecise(self, speed_dc, steer_dc, active_time_sec):
  #   '''
  #     use this methods for understanding vehicle dynamics
  #     if steer_dc > 0 - move right, else - left 
  #   '''
  #   try:
  #     self.speed_control_motor.rotate(speed_dc,True)
  #     if(steer_dc >= 0):
  #       self.steering_control_motor.rotate(steer_dc,False) # right
  #     else:
  #       self.steering_control_motor.rotate(abs(steer_dc),True) # left

  #     self.cv.acquire()
  #     self.cv.wait(active_time_sec)

  #     # if thread was interrupted don't stop the motor
  #     # for smooth commands chaining
  #     if(threading.current_thread().getName() == self.last_thread_name):
  #       self.speed_control_motor.stop()
  #       self.steering_control_motor.stop()
  #     self.cv.release() 
  #   except Exception as e:
  #     print "Forward"
  #     print str(e)

  # def moveBackwardPrecise(self, speed_dc, steer_dc, active_time_sec):
  #   '''
  #     use this methods for understanding vehicle dynamics
  #     if steer_dc > 0 - move right, else - left 
  #   '''
  #   try:
  #     self.speed_control_motor.rotate(speed_dc,False)
  #     if(steer_dc >= 0):
  #       self.steering_control_motor.rotate(steer_dc,False) # right
  #     else:
  #       self.steering_control_motor.rotate(abs(steer_dc),True) # left

  #     self.cv.acquire()
  #     self.cv.wait(active_time_sec)

  #     # if thread was interrupted don't stop the motor
  #     # for smooth commands chaining
  #     if(threading.current_thread().getName() == self.last_thread_name):
  #       self.speed_control_motor.stop()
  #       self.steering_control_motor.stop()
  #     self.cv.release()
  #   except Exception as e:
  #     print "Backward"
  #     print str(e)


  # # Asynch (to be able to interrupt commands by new commands 
  # # (for ex. for collision avoidance, when new object suddenly appears in front of us)
  # def stopAnotherCommands(self):
  #   self.cv.acquire()
  #   self.cv.notifyAll()
  #   self.cv.release()

  # def moveForwardAsync(self, speed_dc, steer_dc, active_time_sec):
  #   self.stopAnotherCommands()
  #   cmdThread = threading.Thread(target=self.moveForwardPrecise, args=(float(speed_dc), float(steer_dc), float(active_time_sec)))
  #   cmdThread.daemon = True
  #   self.last_thread_name = cmdThread.getName()
    
  #   cmdThread.start()
    
 
  # def moveBackwardAsync(self, speed_dc, steer_dc, active_time_sec):
  #   self.stopAnotherCommands()
  #   cmdThread = threading.Thread(target=self.moveBackwardPrecise, args=(speed_dc, steer_dc, active_time_sec))
  #   cmdThread.daemon = True
  #   self.last_thread_name = cmdThread.getName()
  #   cmdThread.start()

  # def left_medium(self):



vehicle = Vehicle(
  Motor("{}/src/control_motors/speed_motor.yaml".format(os.environ["KOTYAMBA_REPO_RASPBERRY"])), 
  Motor("{}/src/control_motors/steering_motor.yaml".format(os.environ["KOTYAMBA_REPO_RASPBERRY"]))
  )