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
        # publishing to "turtle1/cmd_vel" to control turtle1
        self.publisher = rospy.Publisher('turtle1/cmd_vel', Twist)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.on_joy_data)

        rospy.init_node('Joy2Kotyamba')
        rospy.spin()
    def on_joy_data(self, data):
        twist = Twist()
        # Up/Down Axis stick left (data.axes[1])
        twist.linear.x = data.axes[1]
        
        # Left/Right Axis stick right (data.axes[2])
        twist.angular.z = data.axes[2]
        self.publisher.publish(twist)

# rosrun turtlesim turtlesim_node
if __name__ == '__main__':
    tn = Teleoperation_node()