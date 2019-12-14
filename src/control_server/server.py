import os # os.system("cmd")


from network_status import get_command_center_ip
from environment import setup_distributed_ROS_environment

import sys
# https://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
sys.path.append("{}/src/control_motors".format(os.environ["KOTYAMBA_REPO_RASPBERRY"].rstrip()))
print "added paths to *.py files to PYTHON_PATH:"
print(sys.path)

import tornado.httpserver
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket

from motor_pwm import Motor
from vehicle import Vehicle

from driving_modes_manager import DrivingModesManager
 
from tornado.options import define, options
define("port", default=8085, help="run on the given port", type=int)

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
    self.driving_modes_manager = DrivingModesManager()
  
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
 
  def on_close(self):
    print 'connection closed'

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
        self.on_control_command(message_list[1:])
      elif(msg_type in "mode_command"):
        self.on_mode_command(message_list[1])
      elif (msg_type in "status_info"):
        print message_list[1]
      else: print "ERROR: unnown command type"
    except Exception as e:
      print str(e)

  def on_control_command(self, cmd):
    xOffset, yOffset, viewWidth, viewHeight, slider_value = map(float, cmd)
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
  
  def on_mode_command(self, mode):
    if(mode in "manual"):
      self.enable_manual_mode();
    elif(mode in "training"):
      self.enable_training_mode();
    elif(mode in "autonomous"):
      self.enable_self_driving_mode();

  def enable_manual_mode(self):
    self.driving_modes_manager.terminate_active_mode()

  def enable_training_mode(self):
    try:
      command_center_ip = get_command_center_ip()
      print "Found commandCenter machine: {}".format(command_center_ip)
    except Exception as e:
      print "Error cannot start training mode!"
      print e
    else:
      self.driving_modes_manager.start_training_mode()

  def enable_self_driving_mode(self):
    # check that Command center is up
    try:
      command_center_ip = get_command_center_ip()
      print "Found commandCenter machine: {}".format(command_center_ip)
    except Exception as e:
      print "Error cannot start self-driving mode!"
      print e
    else:
      self.driving_modes_manager.start_self_driving_mode()


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

  SpeedControlMotor = Motor("../control_motors/speed_motor.yaml")
  SteerControlMotor = Motor("../control_motors/steering_motor.yaml")
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  setup_distributed_ROS_environment()

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
