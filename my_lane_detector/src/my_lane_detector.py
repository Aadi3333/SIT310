#!/usr/bin/env python3

# Python Libs
import sys
import time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Subscribing to the image topic
        self.image_sub = rospy.Subscriber('/akandb/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        
        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert compressed image message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Get image dimensions
        height, width, _ = img.shape

        # Define the cropping parameters
        top = int(height / 2)
        bottom = height
        left = 0
        right = width

        # Crop the image
        cropped_img = img[top:bottom, left:right]

        # Convert cropped image to HSV color space
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Define range for white color in HSV
        lower_white = np.array([0, 0, 200])  # Lower bound for white color
        upper_white = np.array([255, 50, 255])  # Upper bound for white color

        # Define range for yellow color in HSV
        lower_yellow = np.array([20, 100, 100])  # Lower bound for yellow color
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow color

        # Threshold the HSV image to get only white pixels
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)

        # Threshold the HSV image to get only yellow pixels
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        # Apply Canny Edge Detection to both masks
        edges_white = cv2.Canny(white_mask, 50, 150)
        edges_yellow = cv2.Canny(yellow_mask, 50, 150)

        # Apply Hough Transform to the edge-detected images
        white_lines = self.apply_hough_transform(edges_white)
        yellow_lines = self.apply_hough_transform(edges_yellow)

        # Create black background images for drawing lines
        black_img_white_lines = np.zeros_like(cropped_img)
        black_img_yellow_lines = np.zeros_like(cropped_img)

        # Draw lines found on both Hough Transforms on the black background images
        self.draw_lines(black_img_white_lines, white_lines, color=(255, 255, 255))  # white lines
        self.draw_lines(black_img_yellow_lines, yellow_lines, color=(0, 255, 255))  # yellow lines

        # Convert back to RGB for visualization
        white_filtered_img = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        yellow_filtered_img = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)

        # Display the images
        cv2.imshow('White Filtered Image', white_filtered_img)
        cv2.imshow('Yellow Filtered Image', yellow_filtered_img)
        cv2.imshow('Detected Lines - White', black_img_white_lines)
        cv2.imshow('Detected Lines - Yellow', black_img_yellow_lines)

        cv2.waitKey(1)

    def apply_hough_transform(self, img):
        # Apply Hough Transform
        lines = cv2.HoughLinesP(img, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=50)
        return lines

    def draw_lines(self, img, lines, color=(0, 255, 0)):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), color, 2)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

