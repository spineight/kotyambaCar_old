#!/usr/bin/env python

import rospy

import cv2
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from message_filters import ApproximateTimeSynchronizer, Subscriber



# https://stackoverflow.com/questions/32490629/getting-todays-date-in-yyyy-mm-dd-in-python
from datetime import date

import os # mkdir

class Data_set_generation:
    def on_frame_callback(self,image, twist_stamped):
        print "on_frame_callback"
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite('{}/{}_{:.4f}_{:.4f}.jpg'.format(self.path_to_dataset, self.n_of_frames_dumped, twist_stamped.twist.linear.x, twist_stamped.twist.angular.z),cv_image)
            

            # print "image rows:{} cols:{} channels:{}".format(cv_image.shape[0], cv_image.shape[1], cv_image.shape[2])

            # print twist_stamped.twist
            print "{:.4f},{:.4f}".format(twist_stamped.twist.linear.x,twist_stamped.twist.angular.z)
             

            # This function should be followed by waitKey function which displays 
            # the image for specified milliseconds. 
            # Otherwise, it won not display the image. 
            # For example, waitKey(0) will display the window infinitely 
            # until any keypress (it is suitable for image display). 
            # waitKey(25) will display a frame for 25 ms, 
            # after which display will be automatically closed. 
            # (If you put it in a loop to read videos, 
            # it will display the video frame-by-frame)
            # https://docs.opencv.org/2.4/modules/highgui/doc/user_interface.html?highlight=waitkey
            
            #show every 20th image
            if(self.n_of_frames_dumped % 20 == 0):
                pass
                # cv2.imshow("Image. bgr8", cv_image)
                # cv2.waitKey(100)
                # print "[INFO] {} frames saved to dataset".format(self.n_of_frames_dumped)
            self.n_of_frames_dumped = self.n_of_frames_dumped + 1
        except CvBridgeError as e:
            print(e)

        

        

    def __init__(self, path_to_dataset):
        print "init"
        rospy.init_node("dataset_generation_node") # removed ,anonymous=True to be able to kill it by name
        self.bridge = CvBridge()
        self.image_sub = Subscriber("/usb_cam/image_raw", Image)
        self.cmd_vel_sub = Subscriber("kotyamba/cmd_vel", TwistStamped)
        self.ats = ApproximateTimeSynchronizer([self.image_sub, self.cmd_vel_sub], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.on_frame_callback)

        self.path_to_dataset = path_to_dataset
        self.n_of_frames_dumped = 0
        rospy.spin()

if __name__ == '__main__':
    try:
        today = str(date.today())
        # print(today)   # '2017-12-26'
        path_to_dataset = "/home/oleg/dataset/{}".format(today)
        print "[INFO] path to dataset:{}".format(path_to_dataset)
        os.mkdir(path_to_dataset)
        dsg = Data_set_generation(path_to_dataset)
    except rospy.ROSInterruptException:
        print "Data_set_generation listener was interrupted"
        cv2.destroyAllWindows()