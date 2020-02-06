# Emarus
ROS Workspace for experimental project (rocket league)

## The Project
This Project has been developed for the Experimental Robotics Laboratory course under the masters degree program in Robotics Engineering at University of Genoa.

### The Objective
The aim of the project is to develop a rocket league vehicle.

### Description of the Nodes

* **visual_node.py**: This node handles the image ball and the goal recognition using openCV library. It publish on the */camera/visual_recognition* topic the error between the ball and the center of the camera(*error_ball*), between the goal and the center of the camera (*error_goal*), finally the distance between the ball and the camera(*distance_ball*) and two boolean to know if somethings is seen(*ball_seen* and *goal_seen*). A personalize message has been created to achieve that. Add to this, to obtain the distance between the ball and the camera, a Triangle Similarity for object to Camera Distance need to be achieve.

* **simplified_sm.py**: This node is responsible for the decision-making process. The node subscribes to the */camera/visual_recognition* topic and thus gets the data from the camera. From there, the node computes the transition that should be made from the current node to the next one. There are five states in total : 
1. FINDINGBALL : If the robot cannot see the ball at the moment, it rotate on itself with a current speed until it can see it. When the ball is seen, the transition leads to the state TARGETINGBALL.

2. TARGETINGBALL : In this state, the robot is supposed to align with the ball and to get close enough to kick it. This correction in distance and in alignment is done simultaneously (if both are needed) considering that the robot is holonomic. Nevertheless, the priority is given to the realignment by including the inverse of the misalignment with the ball in the computation of the linear speed along the y-axis. To have a smooth realignment, the angular speed around the z-axis is proportionnal to the misalignment like so: 


                                          W = T1 x angular_misalignment_ball
                                          V = T2 / abs(angular_misalignment_ball)

  Where W is the angular speed, V the linear speed, T1 and T2 two coefficients empirically computed.
  If not realignment is needed, then the angular speed is null and the linear velocity is a constant.
  While the robot is not sufficiently close and aligned, the next state will be TARGETINGBALL. If in the targeting process, the ball is lost, the next state will be FINDINGBALL. Finally, when the robot is correctly positioned to kick, the transition will lead to ALIGNING in order to align with the goal.

3. ALIGNING : In this state, the robot has already found the ball and is close to the ball. Now it orbits around the ball until it finds the goal. To orbit, the robot is given a linear velocity along the x-axis as well as a negative angular velocity around the z-axis. 
Once the goal is found, we try to align the ball and the goal in the same line of sight so that the ball can be kicked in the right direction. Here the alignment speed of the robot should take into acount the direction of the ball to guarantee that the orbital is done in the right direction of rotation:


                                          W = -A1 x sign(angular_misalignment_goal)
                                          V = A2 x sign(angular_misalignment_goal)

  Where W is the angular speed, V the linear speed, A1 and A2 two coefficients empirically computed.
  Once aligned, the next state will be KICKINGBALL. If in the process of orbiting, the ball goes out of the field of view, then we go back to the FINDINGBALL state.

4. KICKINGBALL : Assuming that the robot is ideally positioned with respect to the ball, this state will only give a strong impulse to the robot to kick the ball in the direction of the goal. If the ball has not reached the goal, then go to the state FINDINGBALL to be ready to kick the ball again as soon as possible.

![scheme of the organisation of the states](https://raw.githubusercontent.com/thomasgallo/emarus/master/sm_scheme.png)

Please note that in this state machine architecture, no stopping condition has been given. We have just assume that the robot will keep playing forever. Nevertheless, if the position of the robot could be known (using a motion Arena for instance) then a stopping condition would be implemented and would be as following: "if the position and the orientation of the robot indicates that it is in front of the goal and that it can see the ball and the goal, then the ball must be in the goal".

### How to compute the distance with an object
In order to determine the distance from our camera to a known object, we have use triangle similarity.

To achieve the triangle similarity we have an object with a known width W. We then place this object at a known distance D from our camera. We take a picture of our object using our camera and then measure the apparent width in pixels P. This allows us to derive the perceived focal length F of our camera:

                                          F = (P x  D) / W

Now trough image processing we are able in real time to compute the apparent width at each moment of the object. Therefor we can apply the triangle similarity to determine the distance of the object to the camera:

                                          D’ = (W x F) / P

This will be use to compute the distance with the ball.

### Strategies
During a preliminary phase, we have discussed the different strategies that could be implemented on the robot. Depending on the performance and the precision of the robot, especially regarding the self localisation, we thought about :
- The "orbiteur": Once the ball is targetting and the robot is aligned with it, it will "orbit" around the ball until being also align and in front of the goal. Then the robot can kick with a higher success rate. This technique rely on this idea:
![orbital_strategy](https://raw.githubusercontent.com/thomasgallo/emarus/master/orbital.png)
- The "Disha" (the One that knows the direction): Here, the robot has an a priori knowledge of the position of the goal. When it finishes the targeting phase, it will turn from an angle corresponding to the direction to the unseen goal. This algorithm rely on the assumption that the odometry is sufficiently good to update the direction. In addition, we need to integrate a correction algorithm which reinitialize the direction of the goal every time the goal is seen by the robot. 

# Getting Started

## Prerequisites

### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### ROS vision openCV and Imutils

In order to achieve the visual recognition we use a repository that provide packaging of the popular OpenCV library for ROS.

Clone the packages in the src folder

```
$ git clone https://github.com/ros-perception/vision_opencv.git
```
Function from the imutils library also as been used and need to be install:
```
$ sudo pip install imutils
```

## Run the Project
### Connecting the raspberry to the wifi

Make sure that the Raspberry is correctly connected to the right wifi. To do so, plug the SD card to a computer with linux. Once it is done, go to *rootfs/etc/wpa_supplicant*. Then, execute ``` sudo nano wpa_supplicant.conf``` The configuration file of the Raspberry is now accessible. You can then change the network like so:
```
network={
  ssid="name_of_wifi"
  psk="corresponding_password"
}
```
Then save by doing CTRL+x .
If the configuration of the wifi does not work properly, make sure that no spaces has been added to the wpa_supplicant.conf file as it is not well interpreted.
Also, make sure that ssh has been enabled in the Raspberry.
You can now put the card back in the Raspberry and turn it on.
A IP address has been given to the Raspberry. To find it, execute ```ifconfig``` on the terminal of your computer which must be connected to the same wifi network. You need to give this IP address to your Raspberry. Go to the Raspberry's terminal and execute ``` sudo nano ~/.bashrc```. At the end of the file, you should replace or create:
``` 
export ROS_MASTER_URI=htpp://IP_adress_from_above:11311
```
11311 is the port on your computer dedicated to ROS. This line allows the Raspberry to know that it master will be your computer. In our case, only one Raspberry is used. 
In the scenario where more than one Raspberry is used, you should do an ```ifconfig``` to get the IP address of the Raspberry and define it as a ROS_HOSTNAME to make sure that the computer knows which Raspberry it is communicating with.

Finally, save the nano and execute ``` source ~/.bashrc```

### Launching the code

Open a new terminal and launch on your laptop

```
$ roscore
```


Connect the robot to the ROS Master

```
$ ssh pi@raspberrypi.local 
```
The password is asked, it is "raspberry".

In the robot launch the camera node and the rosserial node (for arduino)

```
$ roslaunch raspicam_node camerav2_410x308_30fps.launch enable_raw:=true
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Run the visual recognition and the state machine on your computer

```
$ roslaunch emarus emarus.launch
```

### Result
The robot has not yet been tested on a real field but on a temporary one we created for testing purposes. The result can be observed in the following video.

![](20200205_234115.gif)

The robot is perfectly achieving the strategy put in place. The fake ball created does not roll so is not able to reach easily the goal but this will not happened with a real ball.
