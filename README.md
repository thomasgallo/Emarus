# Emarus
ROS Workspace for experimental project (rocket league)

## The Project
This Project has been developed for the experimental course of the master degree program in Robotics Engineering at University of Genoa.

### The Objective
The aim of the project is to develop a rocket league vehicule

### Description of the Nodes

* **visual_node.py**: this node handles the image ball and the goal recognition using openCV librairi. It publish on the */camera/visual_recognition* topic the error between the ball and the center of the camera(*error_ball*), between the goal and the center of the camera (*error_goal*), finaly the distance between the ball and the camera(*distance_ball*) and two boolean to know if somethings is seen(*ball_seen* and *goal_seen*). To obtain the distance between the ball and the camera, a Triangle Similarity for object to Camera Distance need to be achieve.

* **simplified_sm.py**: This node is responsible for the decision-making process. The node subscribe to the */camera/visual_recognition* topic and thus get the data from the camera. From there, the node compute the transition that should be make from the current node to the next one. There is four states in total : 
1. FINDINGGOAL : In this state, if the goal is not visible, the robot should rotate on itself (around the z-axis) in order to have the goal in its visual field. If the goal is already visible, the transition will lead to the next state : FINDINGBALL.
2. FINDINGBALL : Here the principle is the same but with the ball. When the ball is seen, the trainsition leads to the state TARGETINGBALL.
3. TARGETINGBALL : In this state, the robot is supposed to align with the ball and to get close enough to kick it. This correction in distance and in alignment is done simoultaniously considering that the robot is holonomic. While the robot is not sufficiently close and aligned, the next state will be TARGETINGBALL. If in the targeting process, the ball is lost, the next state will be FINDINGBALL. Finally, when the robot is correctly positionned to kick, the trainsition will lead to KICKINGBALL.
4. KICKINGBALL : assuming that the robot is ideally positionned with respect to the ball, this state will only give a strong impulse to the robot in the direction of the ball before going to the state FINDINGBALL to be ready to kick the ball again as soon as possible.

![scheme of the organisation of the states](https://raw.githubusercontent.com/thomasgallo/emarus/thomas/sm_scheme.png)


### Strategies
During a preliminary phase, we have discussed the different strategies that could be implemented on the robot. Depending on the performance and the precision of the robot, especially regarding the self localisation, we thought about :
- The "orbiteur": Once the ball is targetting and the robot is aligned with it, it will "orbitate" around the ball until being also align and in front of the goal. Then the robot can kick with a higher success rate. This technique rely on this formula:
![orbital_strategy](https://raw.githubusercontent.com/thomasgallo/emarus/thomas/Experimental Lab.png)
# Gettin Started

## Prerequisites

### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### ROS vision openCV

In order to achieve the visual recognition we use a repository that provide packaging of the popular OpenCV library for ROS.

Clone the packages in the src folder

```
$ git clone https://github.com/ros-perception/vision_opencv.git

```

## Run the Project

Open a new terminal and launch on your laptop

```
$ roscore
```


Connect the robot to the ROS Master

```
$ ssh pi@raspberrypi.local 
```

In the robot launch the camera node

```
$ roslaunch raspicam_node camerav2_410x308_30fps.launch enable_raw:=true
```

Run the visual recognition 

```
$ rosrun emarus visual_node.py 
```
