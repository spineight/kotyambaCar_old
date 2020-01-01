#! /usr/bin/env python

import rospy

import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math

import numpy as np

def get_slope_rad(line):
    ''' input line: (x1, y1, x2, y2) '''
    x_1 = np.float32(line[0])
    y_1 = np.float32(line[1])
    x_2 = np.float32(line[2])
    y_2 = np.float32(line[3])
    ### take care that x_2 - x_1 not 0 : '+ np.finfo(float).eps'
    k = (y_2 - y_1) / (x_2 - x_1 + np.finfo(float).eps)
    return k
def get_bias(line):
    ''' input line: (x1, y1, x2, y2) 
        y = kx + b
        b = y -kx
        returns b
    '''
    x_1 = np.float32(line[0])
    y_1 = np.float32(line[1])

    k = get_slope_rad(line)
    b = y_1 - k * x_1
    return b


def show_line_verbose(img, line, color):
    slope_deg = get_slope_rad(line) * 180. / np.pi

    x_1 = line[0]
    y_1 = line[1]
    x_2 = line[2]
    y_2 = line[3]

    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (int((x_2+x_1)/2.), int((y_2+y_1)/2.))
    fontScale = 1
    fontColor = (255,255,255)
    lineType = 2
    cv2.putText(img,"{:.2f}".format(slope_deg),
    bottomLeftCornerOfText, 
    font, 
    fontScale,
    fontColor,
    lineType)

    
    thickness = 5
    cv2.line(img, (line[0], line[1]), (line[2], line[3]), color, thickness)

def get_edges_img(color_img):
    ''' input img is in BGR format (BlueGreenRed)
        returns image of edges of color_img
    '''
    # convert to grayscale
    img_gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

    # perform gaussian blur
    img_blur = cv2.GaussianBlur(img_gray, (17, 17), 0)

    # perform edge detection
    edges_img = cv2.Canny(img_blur, threshold1=50, threshold2=80)
    return edges_img

def get_lines(edges_img):
    ''' perform hough transform
        returns [(x1, y1, x2, y2), (x1, y1, x2, y2)]
    ''' 
    ## Accumulator
    rho=1 # Distance resolution of the accumulator in pixels
    theta=np.pi / 180 # Angle resolution of the accumulator in radians
    # threshold - min number of intersections to detect a line
    threshold=4 # Accumulator threshold parameter. Only those lines are returned that get enough votes > threshold )
    ## Line 
    img_height = edges_img.shape[1]
    # The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    minLineLength=img_height/4. # Minimum line length. Line segments shorter than that are rejected.
    # The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap=5 # Maximum allowed gap between points on the same line to link them
    
    lines = cv2.HoughLinesP(edges_img,
                                        rho = rho,
                                        theta = theta,
                                        threshold = threshold,
                                        minLineLength = minLineLength,
                                        maxLineGap = maxLineGap)
    # HoughLinesP returns in format [ [line0], [line1] ]
    # convert return result to more convenient format
    result = []
    if lines is not None:
        result = [line[0] for line in lines]
    return result
def get_candidate_lines(lines):
    candidate_lines = []
    for line in lines:
        # in get_slope_degrees division by zero is possible
        slope = get_slope_rad(line)
        slope_deg = slope * 180. / np.pi
        if (40 <= abs(slope_deg) and abs(slope_deg) <= 100):
            print "candidate {}".format(line)
            candidate_lines.append(line)
    return candidate_lines

def get_ego_lane_boundaries(candidate_lines, img_height):
    ''' find line equation y = kx + b for left and right lanes,
        steps:
        1. separate lines into groups:left/right based on slope
        2. avarage lines within their group
        3. find 2 points: (0,b) and (-b/k,0) for each line
        4. given 2 points on the line extend it to the screen limits '''
    left_lanes = []
    right_lanes = []
    for l in candidate_lines:
        if get_slope_rad(l) < 0:
            left_lanes.append(l)
        else:
            right_lanes.append(l)
    left_lane = None
    if len(left_lanes) > 0:
        left_lane_b = np.median([get_bias(l) for l in left_lanes])
        left_lane_k = np.median( [get_slope_rad(l) for l in left_lanes] )
        left_lane_x0, left_lane_y0 = 0,left_lane_b
        left_lane_x1, left_lane_y1 = -left_lane_b/left_lane_k, 0
        left_lane = map(int,(left_lane_x0, left_lane_y0, left_lane_x1, left_lane_y1))

    right_lane = None
    if len(right_lanes) > 0:
        right_lane_b = np.median([get_bias(l) for l in right_lanes])
        right_lane_k = np.median( [get_slope_rad(l) for l in right_lanes] )
        right_lane_x0, right_lane_y0 = 0,right_lane_b
        right_lane_x1, right_lane_y1 = (img_height-right_lane_b)/right_lane_k, img_height
        right_lane = map(int,(right_lane_x0, right_lane_y0, right_lane_x1, right_lane_y1))

    return (left_lane, right_lane)

class Simple_lane_finder:
    def __init__(self):
        rospy.init_node("simple_lane_finder") # removed ,anonymous=True to be able to kill it by name
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