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
define("port", default=8084, help="run on the given port", type=int)
 
class IndexHandler(tornado.web.RequestHandler):
  def get(self):
    self.render('index.html')
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
  def initialize(self, car):
    self.car = car
  def open(self):
    print 'new connection'
    # write msg to the client
    self.write_message("connected")
 
  def on_message(self, message):
    try:
      print "message received {}".format(message)
      # write message to the client
      self.write_message("message received {}".format(message))
      xOffset, yOffset = message.split()
      print "xOffset: {0}; yOffset {1}".format(xOffset, yOffset) 
      
      ## move car
      if(yOffset > 0):
        car.moveForwardAsync(speed_dc, steer_dc, active_time)
      elif(yOffset <0)
        car.moveBackwardAsync(speed_dc, steer_dc, active_time)

    except Exception as e:
      print str(e)
 
  def on_close(self):
    print 'connection closed'

def make_app(car):
    return tornado.web.Application(
    handlers=[
      (r"/", IndexHandler),
      (r"/websocket", WebSocketHandler, {"car":car}) # maps handler to http://kotyambacar.local:8080/websocket request
      ]
  )
 
if __name__ == "__main__":
  tornado.options.parse_command_line()

  SpeedControlMotor = Motor(7,8,1)
  SteerControlMotor = Motor(9,10,11)
  car = Vechicle(SpeedControlMotor, SteerControlMotor)

  app = make_app(car)

  
  httpServer = tornado.httpserver.HTTPServer(app)
  httpServer.listen(options.port)
  print "Listening on port:", options.port
  tornado.ioloop.IOLoop.instance().start()
