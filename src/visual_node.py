#!/usr/bin/env python

#############################################################################
# imports
#############################################################################

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from emarus.msg import camera

VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)

        # topic where we publish the position of the ball, the goal and the distance from the ball
        self.recognition_pub = rospy.Publisher("/camera_node/visual_recognition",camera, queue_size=1)

        # definition of the message
        self.visual_msg = camera()

        # Variable necessary to compute the distance with the ball
        self.known_distance = 25
        self.know_width_ball = 6
        self.focal = (self.known_distance * 84)/self.know_width_ball

        # subscribed Topic to get the camera image
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /raspicam_node/image/compressed"


    def callback(self, ros_data):
        #Callback function of subscribed topic Here images get converted and features detected

        #### direct conversion from ROS to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Definiton of red color for the mask
        redLower = (161, 155, 84)
        redUpper = (179, 255, 255)

        # Definiton of green color for the mask
        greenLower = (40, 40, 40)
        greenUpper = (70, 255, 255)

        #Recuperation des informations de l'image
        h, w, d = image_np.shape

        #Conversion of the RGB image into HSV
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        #Application of the red mask to the image
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        #Application of the green mask to the image
        mask_goal = cv2.inRange(hsv, greenLower, greenUpper)
        mask_goal = cv2.erode(mask_goal, None, iterations=2)
        mask_goal = cv2.dilate(mask_goal, None, iterations=2)

        #Find the ball contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center_ball = None

        #Find the goal contour
        cnts_goal = cv2.findContours(mask_goal.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts_goal = imutils.grab_contours(cnts_goal)
        center_goal = None
        radius = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
			# find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center_ball = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 10:
                    # compute the error between the center of the ball and the center of the camera and the distance
					self.visual_msg.error_ball = center_ball[0] - w/2
					self.visual_msg.ball_distance =  self.know_width_ball * self.focal / (radius * 2)
					self.visual_msg.ball_seen = True

                    # Draw a circle in the center of the ball and around it
					cv2.circle(image_np, (int(x), int(y)), int(radius),(0, 255, 255), 2)
					cv2.circle(image_np, center_ball, 5, (0, 0, 255), -1)
			else:
					self.visual_msg.error_ball = 0
					self.visual_msg.ball_distance = 0
					self.visual_msg.ball_seen = False

    # only proceed if at least one contour was found
	if len(cnts_goal) > 0:
		# find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
		c = max(cnts_goal, key=cv2.contourArea)
		((x, y), radius_goal) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center_goal = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		if radius_goal > 10:
            # compute the error between the center of the goal and the center of the camera
			self.visual_msg.error_goal = center_goal[0] - w/2
			self.visual_msg.goal_seen = True
			# draw the circle and centroid on the frame
			cv2.circle(image_np, (int(x), int(y)), int(radius_goal),(0, 255, 255), 2)
			cv2.circle(image_np, center_goal, 5, (0, 0, 255), -1)
		else:
			self.visual_msg.error_goal = 0
			self.visual_msg.goal_seen = False

    #publish on the visual recognition topic
	self.recognition_pub.publish(self.visual_msg)

    #show the result
	cv2.imshow('window',image_np)
	cv2.waitKey(2)




def main(args):
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    def callback(self, ros_data):
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format
