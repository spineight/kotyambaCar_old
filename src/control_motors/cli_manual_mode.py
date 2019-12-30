#!/usr/bin/python

from motor_pwm import Motor
from vehicle import Vehicle

def main():
  SpeedControlMotor = Motor(7,8,1,100)
  SteerControlMotor = Motor(10,9,11,2)
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  car.start_engines()

  print "Manual mode\n Used for understanding vechicle dynamics" 
  print "By providing different commands for precise movement you understand how vechicle behaves"

  try:
    while(True):
      cmdStr = raw_input("---->")
      if(cmdStr in "e"):
        car.on_stop()
        break
      elif(cmdStr in "w"):
        car.on_speed_change(10)
      elif(cmdStr in "s"):
        car.on_speed_change(-10)
      elif(cmdStr in "a"):
        car.on_steering_change(-10)
      elif(cmdStr in "d"):
        car.on_steering_change(10)
        
      
  except KeyboardInterrupt as e: 
    print (str(e))
  except Exception as e:
    print (str(e))

main()
    
