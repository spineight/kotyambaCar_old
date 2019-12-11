import os # os.system("cmd")
import subprocess 

import tornado.httpserver
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket

# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
import sys
sys.path.append("../control_motors") # Adds higher directory to python modules path.
from motor_pwm import Motor
from vehicle import Vehicle
 
from tornado.options import define, options
define("port", default=8085, help="run on the given port", type=int)

# ROS launch files
# http://wiki.ros.org/roslaunch/API%20Usage
import roslaunch
class roslaunch_info:
  def __init__(self, name, launch):
    self.name = name
    self.launch = launch

import subprocess

def setup_env_vars_for_ROS_IP():
  print "#### setting env vars for ROS ip ####"
  raspberry_pi_ip = subprocess.check_output(["getent", "ahosts", "raspberrypi.local"]).split()[0]

  os.environ["ROS_IP"] = raspberry_pi_ip
  print "ROS_IP:{}".format(os.environ["ROS_IP"])
  os.environ["ROS_MASTER_IP"] = raspberry_pi_ip
  print "ROS_MASTER_IP:{}".format(os.environ["ROS_MASTER_IP"])
  os.environ["ROS_MASTER_URI"] = "http://{}:11311".format(raspberry_pi_ip)
  print "ROS_MASTER_URI:{}".format(os.environ["ROS_MASTER_URI"])


### handles requests: http://kotyambacar.local:8084/
class IndexHandler(tornado.web.RequestHandler):
  def get(self):
    self.render('index.html')

###  Docs for WebSocketHandler:
### https://www.tornadoweb.org/en/stable/websocket.html
class WebSocketHandler(tornado.websocket.WebSocketHandler):
  # WebSocketHandler
  def initialize(self, car):
    print "initializing {}".format(self.__class__.__name__)
    self.car = car
    self.active_launch_file = None
  
  # overridden
  # Invoked when a new WebSocket is opened.
  # The arguments to open are extracted from the tornado.web.URLSpec regular expression
  def open(self):
    print 'new connection'
    # Sends the given message to the client of this Web Socket.
    # The message may be either a string or a dict (which will be encoded as json). 
    # If the binary argument is false, the message will be sent as utf8; 
    # in binary mode any byte string is allowed.
    # If the connection is already closed, raises WebSocketClosedError. 
    # Returns a Future which can be used for flow control.
    self.write_message("connected")
 

  # overridden
  # Handle incoming messages on the WebSocket
  def on_message(self, message):
    try:
      print "message received {}".format(message)
      # write message to the client
      self.write_message("message received {}".format(message))
      message_list = message.split()
      msg_type = message_list[0]
      if(msg_type in "control_command"):
        xOffset, yOffset, viewWidth, viewHeight, slider_value = map(float, message_list[1:])
        print "xOffset: {0}; yOffset {1}".format(xOffset, yOffset) 
        assert(viewWidth == viewHeight)
        circleRadius = viewWidth / 2. # we expect that viewWidth == viewHeight

        steer_dc = xOffset / circleRadius * 100. # 0 <= steer_dc <= 100
        speed_dc = yOffset / circleRadius * 100. # 0 <= speed_dc <= 100
        ## move car
        print "setting steer_dc:{} speed_dc:{}".format(steer_dc,speed_dc)
        if(speed_dc > 0):
          speed_dc = 100
          car.moveForwardAsync(speed_dc, steer_dc, slider_value)
        elif(speed_dc <0):
          speed_dc = 100
          car.moveBackwardAsync(abs(speed_dc), steer_dc, slider_value)
      elif(msg_type in "mode_command"):
        if(message_list[1] in "manual"):
          self.enable_manual_mode();
        elif(message_list[1] in "training"):
          self.enable_training_mode();
        elif(message_list[1] in "autonomous"):
          self.enable_autonomous_mode();
      elif (msg_type in "status_info"):
        print message_list[1]
      else: print "ERROR: unnown command type"

    except Exception as e:
      print str(e)

  def close_all_running_modes(self):
    if(self.active_launch_file is not None):
      print "terminating active launch file: {}".format(self.active_launch_file.name)
      self.active_launch_file.launch.shutdown()
      self.active_launch_file = None

  def enable_manual_mode(self):
    self.close_all_running_modes()
    print "manual mode"
    print "starting Motion module, streaming on port 8081"
    os.system("sudo killall motion")
    path_to_config_file = './motion.conf'
    os.system("sudo motion -m  -c {}".format(path_to_config_file))
    os.system("rosnode kill command_listener.py")
    print "enable_manual_mode"

  def enable_training_mode(self):
    self.close_all_running_modes()
    
    print "stopping Motion module"
    print "enable_training_mode"
    os.system("sudo killall motion")

  def enable_autonomous_mode(self):
    self.close_all_running_modes()
    # check that Command center is up
    try:
      command_center_ip = subprocess.check_output(["getent", "ahosts", "commandCenter.local"]).split()[0]
      print "Found commandCenter machine: {}".format(command_center_ip)
    except Exception as e:
      print "Error! commandCenter machine is not available \n For self-driving mode to work: commandCenter should be up and running"
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(os.path.abspath("../"), "../catkin-ws/src/kotyambaCar/launch/self_driving_mode.launch")])
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["../catkin-ws/src/kotyambaCar/launch/self_driving_mode.launch"])
    self.active_launch_file = roslaunch_info("training mode",launch)
    launch.start()
    rospy.loginfo("training mode launch file started")
    print "enable_autonomous_mode"
    print "stopping Motion module"
    os.system("sudo killall motion")
    # print subprocess.check_output(['source', '../catkin-ws/src/kotyambaCar/scripts/setup_slave_node.sh'])
    # print subprocess.check_output(['rosrun','kotyambaCar', 'command_listener.py'])

 
  def on_close(self):
    print 'connection closed'


## use syntax {"car":car} - to pass argument to the WebSocketHandler initialize method
def make_app(car):
    return tornado.web.Application(
    handlers=[
      (r"/", IndexHandler), ## will handle all requests to http://kotyambacar.local:8085/
      (r"/websocket", WebSocketHandler, {"car":car}) # maps handler to http://kotyambacar.local:8085/websocket request
      ]
  )
 
if __name__ == "__main__":
  tornado.options.parse_command_line()

  # SpeedControlMotor = Motor(7,8,1,100)
  # SteerControlMotor = Motor(9,10,11,100)

  SpeedControlMotor = Motor("../control_motors/speed_motor.yaml")
  SteerControlMotor = Motor("../control_motors/steering_motor.yaml")
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  setup_env_vars_for_ROS_IP()

  app = make_app(car)
  
  httpServer = tornado.httpserver.HTTPServer(app)
  try:
    httpServer.listen(options.port)
    print "Listening on port:", options.port
  except Exception as e:
    print e
    print "Unblocking port"
    os.system("sudo lsof -t -i tcp:{0} | xargs kill -9".format(options.port))
    httpServer.listen(options.port)
    print "Listening on port:", options.port
  tornado.ioloop.IOLoop.instance().start()
