#!/usr/bin/python

from motor_pwm import Motor
from vehicle import Vehicle

def main():
  SpeedControlMotor = Motor(7,8,1)
  SteerControlMotor = Motor(9,10,11)
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  print "Manual mode\n Used for understanding vechicle dynamics" 
  print "By providing different commands for precise movement you understand how vechicle behaves"

  try:
    while(True):
      print "Cmd format:****** %SPEED,%STEER,TIME_SEC******"
      print "speed in [-100..100] backward..forward"
      print "steer in [-100..100] left..right"
      print "\n\n\nFor ex.: '80,90,3' - move forward 80% of max power, right 90% of max power for 3 seconds"
      print "\n\n\n for exit: 'e'"
      cmdStr = raw_input("---->")
      if(cmdStr in "e"):
        car.stop()
        break

      speed_dc, steer_dc, active_time = (float(v) for v in cmdStr.split(','))

      if(speed_dc >= 0):
        car.moveForwardAsync(speed_dc, steer_dc, active_time)
      else:
        car.moveBackwardAsync(abs(speed_dc), steer_dc, active_time)
  except KeyboardInterrupt as e: 
    print (str(e))
  except Exception as e:
    print (str(e))

main()
    
