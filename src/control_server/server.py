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

### handles requests: http://kotyambacar.local:8084/
class IndexHandler(tornado.web.RequestHandler):
  def get(self):
    self.render('index.html')

###  Docs for WebSocketHandler:
### https://www.tornadoweb.org/en/stable/websocket.html
class WebSocketHandler(tornado.websocket.WebSocketHandler):
  # WebSocketHandler
  def initialize(self, car):
    self.car = car
  
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
        if(speed_dc > 0):
          car.moveForwardAsync(speed_dc, steer_dc, slider_value)
        elif(speed_dc <0):
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

  def enable_manual_mode(self):
    print "manual mode"
    print "starting Motion module, streaming on port 8081"
    os.system("sudo killall motion")
    path_to_config_file = './motion.conf'
    os.system("sudo motion -m  -c {}".format(path_to_config_file))
    os.system("rosnode kill command_listener.py")
    print "enable_manual_mode"

  def enable_training_mode(self):
    print "stopping Motion module"
    print "enable_training_mode"
    os.system("sudo killall motion")

  def enable_autonomous_mode(self):
    print "enable_autonomous_mode"
    print "stopping Motion module"
    os.system("sudo killall motion")
    # print subprocess.check_output(['source', '../catkin-ws/src/kotyambaCar/scripts/setup_slave_node.sh'])
    print subprocess.check_output(['rosrun','kotyambaCar', 'command_listener.py'])

 
  def on_close(self):
    print 'connection closed'


## use syntax {"car":car} - to pass argument to the WebSocketHandler initialize method
def make_app(car):
    return tornado.web.Application(
    handlers=[
      (r"/", IndexHandler), ## will handle all requests to http://kotyambacar.local:8084/
      (r"/websocket", WebSocketHandler, {"car":car}) # maps handler to http://kotyambacar.local:8084/websocket request
      ]
  )
 
if __name__ == "__main__":
  tornado.options.parse_command_line()

  SpeedControlMotor = Motor(7,8,1,200)
  SteerControlMotor = Motor(9,10,11,200)
  car = Vehicle(SpeedControlMotor, SteerControlMotor)

  app = make_app(car)

  
  httpServer = tornado.httpserver.HTTPServer(app)
  httpServer.listen(options.port)
  print "Listening on port:", options.port
  tornado.ioloop.IOLoop.instance().start()
