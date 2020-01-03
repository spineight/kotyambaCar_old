#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
class Teleoperation_node:
    def __init__(self):
        rospy.init_node('Joy2Kotyamba')
        # publishing to "turtle1/cmd_vel" to control turtle1
        self.publisher = rospy.Publisher('kotyamba/cmd_vel', Twist)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.on_joy_data)
        print "init"


        rospy.spin()
    def on_joy_data(self, data):
        twist = Twist()
        # Up/Down Axis stick left (data.axes[1])
        twist.linear.x = data.axes[1]
        print "twist.linear.x {}".format(twist.linear.x)
        
        # Left/Right Axis stick right (data.axes[2])
        twist.angular.z = data.axes[2]
        
        stop_button = data.buttons[0]
        if stop_button == 1:
            twist.linear.x = 0
            twist.angular.z = 0

        self.publisher.publish(twist)

# rosrun turtlesim turtlesim_node
if __name__ == '__main__':
    tn = Teleoperation_node()