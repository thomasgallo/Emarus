
"""
This script implements a simple state machine that defines the various possible states of the robot, 
the state transition rules and the actions to be performed in each state.

"""


#!/usr/bin/env python

import rospy
import sys
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from emarus.msg import camera

# Definition of the state machine class

class StateMachine:

    def __init__(self):

        # Subscriber and publisher needed
        self.sub_ = rospy.Subscriber("/camera_node/visual_recognition",camera,self.callback) # Subscribe to the camera's topic
        self.speed_ = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)    # Publish the velocity to send to the motor driver
        self.nh_ = rospy.init_node('simplified_sm', anonymous=True)

        ## IMPORTANT ##
        # List of the possible states, write here to add one
        self.states_ = ['ALIGNING','FINDINGBALL','TARGETINGBALL','STOPPING','KICKINGBALL']
        self.currentState_ = self.states_.index("FINDINGBALL") # Cursor on the current state (Initially FINDINGBALL)

        # Constant coefficients of linear and angular speed, threshold distances and angles
        self.amplitude_ = 2 # amplitude of the rotation around the z-axis
        self.threshold_angle_ = 25  # minimal accepted angle between the robot and the ball
        self.threshold_distance_ = 25 # minimal accepted distance between the robot and the ball
        self.amp_z_ = 0.1
        self.coefficient_ = 0.02    # Coefficient corresponding to the amplitude of the movement along the y axis
        self.kicking_ = 3   # Amplitude of the kicking movement along the y-axis

	# Initialization of variables
        self.ball_distance_ = 0 # Distance between the ball and the robot in m
        self.error_ball_ = 0    # Difference in pixel between the robot and the ball
        self.error_goal_ = 0    # Difference in pixel between the robot and the goal
        self.isBallVisible_ = False
        self.isGoalVisible_ = False
        self.vel_msg_ = Twist() # Msg that will be published to the Arduino


    # Camera message callback function
    def callback(self,data):
        # Get the data from the camera node
        self.isBallVisible_ = data.ball_seen
        self.isGoalVisible_ = data.goal_seen
        self.ball_distance_ = data.ball_distance
        self.error_ball_ = data.error_ball
        self.error_goal_ = data.error_goal

    # Function that decides the state, transitions and corresponding actions
    def decision(self):
       rospy.loginfo("decision making")
       #rospy.loginfo("currentState_ avant = %d",self.currentState_)

       # If the current state is "ALIGNING", then rotate and orbit until the ball and the goal are in the same line of sight.
       if(self.currentState_ == self.states_.index("ALIGNING")):
           rospy.loginfo("ALIGNING")

           if self.isGoalVisible_ == False:
               if self.isBallVisible_ == True:
		   # If the goal is not visible, but the ball is visible, orbit the ball until you see the goal
           # These coefficients have been defined experimentaly and correspond to the orbital trajectory
                   self.vel_msg_.linear.x = 1
                   self.vel_msg_.angular.z = -3
               # else, go to the next state FINDINGBALL
               else: # Look for the ball first
                   self.currentState_ = self.states_.index("FINDINGBALL")
           else: 
	       # If the goal and the ball are visible, kick the ball
               if self.isBallVisible_ == True:
                   self.currentState_ = self.states_.index("KICKINGBALL")
               else: # If the ball is not visible, the first find the ball
                   self.currentState_ = self.states_.index("FINDINGBALL")

       # If the current state is "KICKINGBALL", then approach the ball with the set linear velocity for kicking
       elif(self.currentState_ == self.states_.index("KICKINGBALL")):
           rospy.loginfo("KICKINGBALL")
	   # If the pixel difference between the robot and the goal (alignment) is less than a tolerance threshold (50), then approach.
           if abs(self.error_goal_) < 50:
               self.vel_msg_.linear.y = self.kicking_ 
           else: # Orbit the ball until aligned by going in the direction in which the goal is seen
               self.vel_msg_.linear.x = 1*self.error_goal_/abs(self.error_goal_)
               self.vel_msg_.angular.z = -3*self.error_goal_/abs(self.error_goal_)
               self.currentState_ = self.states_.index("ALIGNING")

       # Find the ball by rotating about the z-axis of the robot frame
       elif(self.currentState_ == self.states_.index("FINDINGBALL")):
           rospy.loginfo("FINDINGBALL")
           # If the ball is not visible, rotate until finding it
           if self.isBallVisible_ == False:
               self.vel_msg_.angular.z = self.amplitude_
           # else, go to the next state TARGETINGBALL
           else:
               self.currentState_ = self.states_.index("TARGETINGBALL")

       # Now that the ball is found, turn to face it directly and approach
       elif(self.currentState_ == self.states_.index("TARGETINGBALL")):
           rospy.loginfo("TARGETINGBALL")
           # If the ball is visible, modify the orientation to face the ball and move towards the ball
           if self.isBallVisible_ == True:
               # Define the orientation of the robot if it is sufficiently misaligned
               if abs(self.error_ball_) > self.threshold_angle_:
                   self.vel_msg_.angular.z = -0.01*(self.error_ball_)   # Rotation proportionnal to the misalignement of the robot with respect to the ball
                   if abs(self.ball_distance_) > self.threshold_distance_:
                        self.vel_msg_.linear.y = 100/(4.0*abs(self.error_ball_)) # Robot go faster to the ball as it is more aligned with it
               # If the robot is already sufficiently aligned
               else:
                   # But it is still far from the ball
                    if abs(self.ball_distance_) > self.threshold_distance_:
                        self.vel_msg_.linear.y = 0.5    # Go straight to the ball with constant speed
                    else:
                        self.vel_msg_.linear.y = 0  # if close enough, stop the movement
                    self.vel_msg_.angular.z = 0
                # If the robot is close enough and in direction of the ball
               if  self.ball_distance_ < self.threshold_distance_:
                   self.currentState_ = self.states_.index("ALIGNING")  # Align the ball to the goal by orbiting
               # else, go to the state FINDINGBALL
           else:
               self.currentState_ = self.states_.index("FINDINGBALL")

       elif(self.currentState_ == self.states_.index("STOPPING")):
           rospy.loginfo("STOPPING")
           # promptly go to the ball

       # In every state, send the velocity to the Arduino and reinitialize the component of the Twist()
       self.speed_.publish(self.vel_msg_)
       self.vel_msg_.linear.y = 0
       self.vel_msg_.linear.x = 0
       self.vel_msg_.angular.z = 0
       #rospy.loginfo("currentState_ apres = %d",self.currentState_)

    def loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
                    self.decision()
                    rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    sm = StateMachine()   # Initialize the state machine
    try:
        sm.loop()    # Spin the decision making code
    except:
        rospy.logerr("Error while executing state machine please try again")
