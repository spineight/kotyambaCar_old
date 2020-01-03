#!/usr/bin/python

from motor_pwm import Motor
from vehicle import Vehicle

def main():
  SpeedControlMotor = Motor(8,7,1,100)
  SteerControlMotor = Motor(9,10,11,2)
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  car.start_engines()

  print "Manual mode\n Used for understanding vechicle dynamics" 
  print "By providing different commands for precise movement you understand how vechicle behaves"

  # TODO move to yaml file
  speed_dc_step = 100
  steering_dc_step = 5
  try:
    while(True):
      cmdStr = raw_input("---->")
      if(cmdStr in "q"):
        car.on_stop()
        break
      elif(cmdStr in "w"):
        car.on_speed_change(speed_dc_step)
      elif(cmdStr in "s"):
        car.on_speed_change(-speed_dc_step)
      elif(cmdStr in "a"):
        car.on_steering_change(-steering_dc_step)
      elif(cmdStr in "d"):
        car.on_steering_change(steering_dc_step)
      elif(cmdStr in "e"):
        car.on_steering_change(0)
        car.on_speed_change(0)
        
      
  except KeyboardInterrupt as e: 
    print (str(e))
  except Exception as e:
    print (str(e))

main()
    
