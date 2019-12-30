#!/usr/bin/python
import threading
from threading import Thread
from time import sleep

from motor_pwm import Motor

import os

class Vehicle:


  def moveForward_(self, speed_dc, steer_dc):
    '''
      if steer_dc > 0 - move right, else - left 
    '''
    self.speed_control_motor.rotate(speed_dc,True)
    if(steer_dc >= 0):
      self.steering_control_motor.rotate(steer_dc,False) # right
    else:
      self.steering_control_motor.rotate(abs(steer_dc),True) # left

  def moveBackward_(self, speed_dc, steer_dc):
    '''
      if steer_dc > 0 - move right, else - left 
    '''
    self.speed_control_motor.rotate(speed_dc,False)
    if(steer_dc >= 0):
      self.steering_control_motor.rotate(steer_dc,False) # right
    else:
      self.steering_control_motor.rotate(abs(steer_dc),True) # left

  def stop_(self):
    ''' soft stop, wheels can move'''
    self.speed_control_motor.stop()
    self.steering_control_motor.stop()

  def hardBreak_(self):
    ''' hard stop - wheels are blocked'''
    self.speed_control_motor.emergency_stop()
    self.steering_control_motor.emergency_stop()

  def moveForward(self,speed,active_time_sec):
    ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
    self.speed_control_motor.rotate(speed,True)
    sleep(active_time_sec)
    self.speed_control_motor.stop()

  def moveBackward(self,speed,active_time_sec):
    ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
    self.speed_control_motor.rotate(speed,False)
    sleep(active_time_sec)
    self.speed_control_motor.stop()

  def turnLeft(self,speed, active_time_sec):
    ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
    self.speed_control_motor.rotate(speed,True)
    self.steering_control_motor.rotate(speed,True)
    sleep(active_time_sec)
    self.speed_control_motor.stop()
    self.steering_control_motor.stop()

  def turnRight(self,speed, active_time_sec):
    ''' simple method to check the wiring of Raspberry Pi, motors and H-bridge '''
    self.speed_control_motor.rotate(speed,True)
    self.steering_control_motor.rotate(speed,False)
    sleep(active_time_sec)
    self.speed_control_motor.stop()
    self.steering_control_motor.stop()

  def stop(self):
    ''' soft stop, wheels can move'''
    self.speed_control_motor.stop()
    self.steering_control_motor.stop()
    self.stopAnotherCommands()

  def hardBreak(self):
    ''' hard stop - wheels are blocked'''
    self.speed_control_motor.emergency_stop()
    self.steering_control_motor.emergency_stop()
    self.stopAnotherCommands()

  ## for more precise control 
  def moveForwardPrecise(self, speed_dc, steer_dc, active_time_sec):
    '''
      use this methods for understanding vehicle dynamics
      if steer_dc > 0 - move right, else - left 
    '''
    try:
      self.speed_control_motor.rotate(speed_dc,True)
      if(steer_dc >= 0):
        self.steering_control_motor.rotate(steer_dc,False) # right
      else:
        self.steering_control_motor.rotate(abs(steer_dc),True) # left

      self.cv.acquire()
      self.cv.wait(active_time_sec)

      # if thread was interrupted don't stop the motor
      # for smooth commands chaining
      if(threading.current_thread().getName() == self.last_thread_name):
        self.speed_control_motor.stop()
        self.steering_control_motor.stop()
      self.cv.release() 
    except Exception as e:
      print "Forward"
      print str(e)

  def moveBackwardPrecise(self, speed_dc, steer_dc, active_time_sec):
    '''
      use this methods for understanding vehicle dynamics
      if steer_dc > 0 - move right, else - left 
    '''
    try:
      self.speed_control_motor.rotate(speed_dc,False)
      if(steer_dc >= 0):
        self.steering_control_motor.rotate(steer_dc,False) # right
      else:
        self.steering_control_motor.rotate(abs(steer_dc),True) # left

      self.cv.acquire()
      self.cv.wait(active_time_sec)

      # if thread was interrupted don't stop the motor
      # for smooth commands chaining
      if(threading.current_thread().getName() == self.last_thread_name):
        self.speed_control_motor.stop()
        self.steering_control_motor.stop()
      self.cv.release()
    except Exception as e:
      print "Backward"
      print str(e)


  # Asynch (to be able to interrupt commands by new commands 
  # (for ex. for collision avoidance, when new object suddenly appears in front of us)
  def stopAnotherCommands(self):
    self.cv.acquire()
    self.cv.notifyAll()
    self.cv.release()

  def moveForwardAsync(self, speed_dc, steer_dc, active_time_sec):
    self.stopAnotherCommands()
    cmdThread = threading.Thread(target=self.moveForwardPrecise, args=(float(speed_dc), float(steer_dc), float(active_time_sec)))
    cmdThread.daemon = True
    self.last_thread_name = cmdThread.getName()
    
    cmdThread.start()
    
 
  def moveBackwardAsync(self, speed_dc, steer_dc, active_time_sec):
    self.stopAnotherCommands()
    cmdThread = threading.Thread(target=self.moveBackwardPrecise, args=(speed_dc, steer_dc, active_time_sec))
    cmdThread.daemon = True
    self.last_thread_name = cmdThread.getName()
    cmdThread.start()


  def __init__(self,speed_control_motor, steering_control_motor):
    self.speed_control_motor = speed_control_motor
    self.steering_control_motor = steering_control_motor

    self.steering_condition_variable = threading.Condition()
    self.speed_condition_variable = threading.Condition()

    self.steering_dc = 0
    self.speed_dc = 0

    self.is_engines_on = False
  
  def steering_thread(self):
    while (self.is_engines_on):
      try:
        if(this.steering_dc >= 0):
          self.steering_control_motor.rotate(this.steering_dc,False) # right
        else:
          self.steering_control_motor.rotate(this.steering_dc,True) # left
        self.steering_condition_variable.acquire()
        self.steering_condition_variable.wait()
        self.steering_condition_variable.release()
      except Exception as e:
        print "steering_thread"
        print str(e)

  def speed_thread(self):
    while (self.is_engines_on):
      try:
        if(this.speed_dc >= 0):
          self.steering_control_motor.rotate(this.speed_dc,False) # right
        else:
          self.steering_control_motor.rotate(this.speed_dc,True) # left
        self.speed_condition_variable.acquire()
        self.speed_condition_variable.wait()
        self.speed_condition_variable.release()
      except Exception as e:
        print "speed_thread"
        print str(e)

  def start_engines(self):
    self.is_engines_on = True
    steering_thread = threading.Thread(target=self.steering_thread)
    steering_thread.daemon = True
    speed_thread = threading.Thread(target=self.speed_thread)
    speed_thread.daemon = True

  def stop_engines(self):
    self.is_engines_on = False
    
    self.steering_condition_variable.acquire()
    self.steering_condition_variable.notifyAll()
    self.steering_condition_variable.release()

    self.speed_condition_variable.acquire()
    self.speed_condition_variable.notifyAll()
    self.speed_condition_variable.release()
  
  def on_speed_change(self, speed_dc_change):
    self.speed_dc = self.speed_dc + speed_dc_change
    print "self.speed_dc:{}".format(self.speed_dc)

  def on_steering_change(self, steering_dc_change):
    self.steering_dc = self.steering_dc + steering_dc_change
    print "self.steering_dc:{}".format(self.steering_dc)

  def on_stop(self):
    self.stop_engines()
# vehicle = Vehicle(
#   Motor("{}/src/control_motors/speed_motor.yaml".format(os.environ["KOTYAMBA_REPO_RASPBERRY"])), 
#   Motor("{}/src/control_motors/steering_motor.yaml".format(os.environ["KOTYAMBA_REPO_RASPBERRY"]))
#   )