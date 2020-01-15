#! /usr/bin/env python

import rospy

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math

import numpy as np

from model_config import *

import tensorflow as tf

models_folder_path = "/home/oleg/dev/kotyambaCar/src/catkin-ws/src/kotyambaCar/src/"

# https://github.com/keras-team/keras/issues/2397



class SelfDriving:
    def __init__(self):
        rospy.init_node("self_driving_node") # removed ,anonymous=True to be able to kill it by name
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.on_image)
        
        self.steering_model = tf.compat.v1.keras.models.load_model(models_folder_path + "steering.model")
        self.steering_model._make_predict_function()

        self.throttle_model = tf.compat.v1.keras.models.load_model(models_folder_path + "throttle.model")
        self.throttle_model._make_predict_function()
        
        rospy.spin()

    def on_image(self,image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            output = cv_image.copy()
            image = cv2.resize(cv_image, (NVIDIA_W, NVIDIA_H))

            # scale the pixel values to [0, 1]
            image = image.astype("float") / 255.0

            # we with a CNN add the batch dimension
            image = image.reshape((1, image.shape[0], image.shape[1],
                image.shape[2]))

            steering_predicted = self.steering_model.predict(image)
            throttle_predicted = self.throttle_model.predict(image)

            # draw the class label + probability on the output image
            steering_text = "steering:{:.2f}".format(float(steering_predicted))
            throttle_text = "throttle:{:.2f}".format(float(throttle_predicted))
            cv2.putText(output, steering_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
	            (0, 255, 255), 2)
            cv2.putText(output, throttle_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
	            (0, 255, 255), 2)

            # show the output image
            cv2.imshow("Image", output)
            cv2.waitKey(0)
        except CvBridgeError as e:
            print(e)
        

if __name__ == '__main__':
    try:
    except rospy.ROSInterruptException:
        print "self_driving_node was interrupted"
        cv2.destroyAllWindows()