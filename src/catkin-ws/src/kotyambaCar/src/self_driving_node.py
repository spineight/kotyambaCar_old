#! /usr/bin/env python

import rospy

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math

import numpy as np


class SelfDriving:
    def __init__(self):
        rospy.init_node("self_driving_node") # removed ,anonymous=True to be able to kill it by name
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.on_image)
        rospy.spin()

    def on_image(self,image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        print "image rows:{} cols:{} channels:{}".format(cv_image.shape[0], cv_image.shape[1], cv_image.shape[2])
        
        color_image = cv_image
        cv2.imshow("Image. bgr8", color_image)

        edges_img = get_edges_img(color_image)
        cv2.imshow("Image edges", edges_img)

        lines = get_lines(edges_img)
        if lines is None:
            print "NO LINES FOUND"
        else:
            detected_lines_img = edges_img.copy()
            print "n of lines found: {}".format(len(lines))

            for line in lines:
                line_color = [255,255,255] #BGR
                show_line_verbose(detected_lines_img, line, line_color)
            candidate_lines = get_candidate_lines(lines)
            cv2.imshow("Detected lines",detected_lines_img)

            print "n of candidate lines: {}".format(len(candidate_lines))
            for line in candidate_lines:
                show_line_verbose(color_image, line, [0,255,0])
            cv2.imshow("Image with lines", color_image)     
            cv2.waitKey(3)

        # https://www.colorspire.com/rgb-color-wheel/
        ego_lanes_color = [0,145,255]
        left_lane, right_lane = get_ego_lane_boundaries(candidate_lines, cv_image.shape[0])
        if left_lane is not None:
            print left_lane
            show_line_verbose(color_image, left_lane, ego_lanes_color)
        if right_lane is not None:
            print right_lane
            show_line_verbose(color_image, right_lane, ego_lanes_color)
        cv2.imshow("Ego lane boundaries", color_image)   

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

if __name__ == '__main__':
    try:
        slf = Simple_lane_finder()
    except rospy.ROSInterruptException:
        print "Simple_lane_finder listener was interrupted"
        cv2.destroyAllWindows()