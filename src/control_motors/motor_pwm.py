#!/usr/bin/python

import RPi.GPIO as GPIO
import yaml

class Motor:
  ''' do not use the start() and stop() methods in a loop, 
      use the ChangeDutyCycle() method instead to set the duty cycle to zero to stop PWM 
      https://raspberrypi.stackexchange.com/questions/68386/pwm-stop-respond-after-hundreds-of-start-stop
  '''
  def __init__(self, *args, **kwargs):
    if len(args) == 1:
      self.init_from_config_file(args[0])
    elif len(args) == 4:
      self.init_with_values(args[0], args[1], args[2], args[3])

  def init_from_config_file(self, path_to_config_file):
    try:
      with open(path_to_config_file) as config_file:
        print "using {} for motor configuration".format(path_to_config_file)
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        params = yaml.load(config_file, Loader=yaml.FullLoader)
        self.init_with_values(params["directionPinFirst_GPIO_id"], params["directionPinSecond_GPIO_id"], params["speedPin_GPIO_id"], params["pwm_frequency"])
    except IOError as e:
      print "config file {} not available".format(path_to_config_file)
      print e

  def init_with_values(self, directionPinFirst_GPIO_id, directionPinSecond_GPIO_id, speedPin_GPIO_id, pwm_frequency):
    ''' if signal on pins:  
        directionPinFirst_GPIO_id - TRUE, directionPinSecond_GPIO_id - FALSE - motor rotates in one direction
        directionPinFirst_GPIO_id - FALSE, directionPinSecond_GPIO_id - TRUE - rotates in oposite direction
        when signals are TRUE & TRUE or FALSE & FALSE - motor is stoped
        when jumper from enB or enA is removed - we can specify voltage and use PWM for speed control,
        otherwise motor will rotate with max speed 

        dc is the duty cycle (0.0% <= dc <= 100.0%)
    '''
    # Use GPIO numbers not pin numbers
    GPIO.setmode(GPIO.BCM)
    # setup gpio channels to output mode
    GPIO.setup(directionPinFirst_GPIO_id, GPIO.OUT)   
    GPIO.setup(directionPinSecond_GPIO_id, GPIO.OUT)   
    GPIO.setup(speedPin_GPIO_id, GPIO.OUT)   

    self.first_gpio_id = directionPinFirst_GPIO_id
    self.second_gpio_id = directionPinSecond_GPIO_id

    self.speedPWM = GPIO.PWM(speedPin_GPIO_id, pwm_frequency)
    self.speedPWM.start(0)

  def __del__(self):
    ''' put GPIO pins to input mode, more safe mode,
        see this: 
        https://raspi.tv/2013/rpi-gpio-basics-3-how-to-exit-gpio-programs-cleanly-avoid-warnings-and-protect-your-pi 
    '''
    GPIO.cleanup()


  def stop(self):
    ''' free motor - not blocked, continue movement by inertia'''
    self.speedPWM.ChangeDutyCycle(0)

  def emergency_stop(self):
    ''' Motor is blocked
        from 298 datasheet three (IN1, IN2, EN) pins for each motor channel.
        EN=0 => freewheel
        else EN=1
        IN1=1, IN2=0 => power forward
        IN1=0, IN2=1 => power backward
        IN1=IN2=1, or IN1=IN2=0 => power braking (motor is shorted through either the high-side or low-side transistors)
        https://forum.arduino.cc/index.php?topic=235720.0
    '''
    self.speedPWM.ChangeDutyCycle(0)
    GPIO.output(self.first_gpio_id,False)
    GPIO.output(self.second_gpio_id,False)

  def rotate(self, dutyCycle, isForward):
    ''' 0 ... 100% 
        the duty cycle (0.0 <= dc <= 100.0) is used in GPIO.PWM
        for refs: https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
    '''
    # enable forward rotation on motor
    GPIO.output(self.first_gpio_id, isForward)
    GPIO.output(self.second_gpio_id,not isForward)

    # rotation speed
    self.speedPWM.ChangeDutyCycle(dutyCycle)
