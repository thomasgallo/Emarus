# exp_proj_ws
ROS Workspace for experimental project (rocket league)

## The Project
This Project has been developed for the experimental course of the master degree program in Robotics Engineering at University of Genoa.

### The Objective
The aim of the project is to develop a rocket league vehicule

### Description of the Nodes

* **visual_node.py**: this node handles the image ball and the goal recognition using openCV librairi. It publish on the */camera/visual_recognition* topic the error between the ball and the center of the camera(*error_ball*), between the goal and the center of the camera (*error_goal*), finaly the distance between the ball and the camera(*distance_ball*) and two boolean to know if somethings is seen(*ball_seen* and *goal_seen*). To obtain the distance between the ball and the camera, a Triangle Similarity for object to Camera Distance need to be achieve.

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
$ rosrun visual_recognition visual_node.py 
```
