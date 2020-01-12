#!/usr/bin/env python
import rospy


import std_msgs.msg
# Twist message doesn't have time info, so I can't synch it with cam_img
# introducing TwistStamped instead brings more overheads,
# see this for details: https://answers.ros.org/question/29842/why-is-twist-message-without-timestamp/
from geometry_msgs.msg import TwistStamped


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
        self.publisher = rospy.Publisher('kotyamba/cmd_vel', TwistStamped, queue_size = None)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.on_joy_data)
        print "init"


        rospy.spin()
    def on_joy_data(self, data):
        twist_stamped = TwistStamped()
        # Up/Down Axis stick left (data.axes[1])
        twist_stamped.twist.linear.x = data.axes[1]
        print "twist.linear.x {}".format(twist_stamped.twist.linear.x)
        
        # Left/Right Axis stick right (data.axes[2])
        twist_stamped.twist.angular.z = data.axes[2]
        print "twist.angular.z {}".format(twist_stamped.twist.angular.z)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        twist_stamped.header = h

        print "twist.header: {}".format(twist_stamped.header)
        
        stop_button = data.buttons[0]
        if stop_button == 1:
            twist_stamped.twist.linear.x = 0
            twist_stamped.twist.angular.z = 0

        self.publisher.publish(twist_stamped)

# rosrun turtlesim turtlesim_node
if __name__ == '__main__':
    tn = Teleoperation_node()