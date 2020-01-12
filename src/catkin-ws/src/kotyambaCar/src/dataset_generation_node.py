#!/usr/bin/env python

import rospy

# import cv2
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from message_filters import ApproximateTimeSynchronizer, Subscriber

class Data_set_generation:
    def on_frame_callback(img, cmd_vel):
        print "on_frame_callback"
    # def on_image(self,img):
    #     print "image"
    # def on_cmd_velocity(self,cmd_vel):
    #     print "cmd_vel"
    def __init__(self):
        print "init"
        rospy.init_node("dataset_generation_node") # removed ,anonymous=True to be able to kill it by name
        self.bridge = CvBridge()
        self.image_sub = Subscriber("/usb_cam/image_raw", Image)
        self.cmd_vel_sub = Subscriber("kotyamba/cmd_vel", Twist)
        self.ats = ApproximateTimeSynchronizer([self.image_sub, self.cmd_vel_sub], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.on_frame_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        dsg = Data_set_generation()
    except rospy.ROSInterruptException:
        print "Data_set_generation listener was interrupted"
        # cv2.destroyAllWindows()