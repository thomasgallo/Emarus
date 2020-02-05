# Emarus
ROS Workspace for experimental project (rocket league)

## The Project
This Project has been developed for the experimental course of the master degree program in Robotics Engineering at University of Genoa.

### The Objective
The aim of the project is to develop a rocket league vehicule

### Description of the Nodes

* **visual_node.py**: this node handles the image ball and the goal recognition using openCV librairy. It publish on the */camera/visual_recognition* topic the error between the ball and the center of the camera(*error_ball*), between the goal and the center of the camera (*error_goal*), finaly the distance between the ball and the camera(*distance_ball*) and two boolean to know if somethings is seen(*ball_seen* and *goal_seen*). A personalize message has been created to achieve that. Add to this, to obtain the distance between the ball and the camera, a Triangle Similarity for object to Camera Distance need to be achieve.

* **simplified_sm.py**: This node is responsible for the decision-making process. The node subscribe to the */camera/visual_recognition* topic and thus get the data from the camera. From there, the node compute the transition that should be make from the current node to the next one. There is four states in total : 
1. FINDINGGOAL : In this state, if the goal is not visible, the robot should rotate on itself (around the z-axis) in order to have the goal in its visual field. If the goal is already visible, the transition will lead to the next state : FINDINGBALL.
2. FINDINGBALL : Here the principle is the same but with the ball. When the ball is seen, the trainsition leads to the state TARGETINGBALL.
3. TARGETINGBALL : In this state, the robot is supposed to align with the ball and to get close enough to kick it. This correction in distance and in alignment is done simoultaniously considering that the robot is holonomic. While the robot is not sufficiently close and aligned, the next state will be TARGETINGBALL. If in the targeting process, the ball is lost, the next state will be FINDINGBALL. Finally, when the robot is correctly positionned to kick, the trainsition will lead to KICKINGBALL.
4. KICKINGBALL : assuming that the robot is ideally positionned with respect to the ball, this state will only give a strong impulse to the robot in the direction of the ball before going to the state FINDINGBALL to be ready to kick the ball again as soon as possible.

![scheme of the organisation of the states](https://raw.githubusercontent.com/thomasgallo/emarus/thomas/sm_scheme.png)

### How to compute the distance with an object
In order to determine the distance from our camera to a known object, we have use triangle similarity.

To achieve the triangle similarity we have an object with a known width W. We then place this object at a known distance D from our camera. We take a picture of our object using our camera and then measure the apparent width in pixels P. This allows us to derive the perceived focal length F of our camera:

                                          F = (P x  D) / W

Now trough image processing we are able in real time to compute the apparent width at each moment of the object. Therefor we can apply the triangle similarity to determine the distance of the object to the camera:

                                          D’ = (W x F) / P

This will be use to compute the distance with the ball.

### Strategies
During a preliminary phase, we have discussed the different strategies that could be implemented on the robot. Depending on the performance and the precision of the robot, especially regarding the self localisation, we thought about :
- The "orbiteur": Once the ball is targetting and the robot is aligned with it, it will "orbitate" around the ball until being also align and in front of the goal. Then the robot can kick with a higher success rate. This technique rely on this formula:
![orbital_strategy](https://raw.githubusercontent.com/thomasgallo/emarus/thomas/orbital.png)
- The "Disha" (the One that knows the direction): Here, the robot has an a priori knwoledge of the position of the goal. When it finishes the targeting phase, it will turn from an angle corresponding to the direction to the unseen goal. This algorithm rely on the assomption that the odometry is sufficiently good to update the direction. In addition, we need to integrate a correction algorithm which reinitialize the direction of the goal every time the goal is seen by the robot. 

# Gettin Started

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
  psk="corrsponding_password"
}
```
Then save by doing CTRL+x .
If the configuration of the wifi does not work properly, make sure that no spaces has been added to the wpa_supplicant.conf file as it is not well interpretetd.
Also, make sure that ssh has been enabled in the Raspberry.
You can now put the card back in the Raspberry and turn it on.
A IP adress has been given to the Raspberry. To find it, execute ```ifconfig``` on the terminal of your computer which must be connected to the same wifi network. You need to give this IP adresse to your Raspberry. Go to the Raspberry's terminal and execute ``` sudo nano ~/.bashrc```. At the end of the file, you should replace or create:
``` 
export ROS_MASTER_URI=htpp://IP_adress_from_above:11311
```
11311 is the port on your computer dedicated to ROS. This line allows the Raspberry to know that it master will be your computer. In our case, only one Raspberry is used. 
In the scenario where more than one Raspberry is used, you should do an ```ifconfig``` to get the IP adress of the Raspberry and define it as a ROS_HOSTNAME to make sure that the computer knows which Raspberry it is communicating with.

Finally, save the nano and execute ``` source ~/.basrhc```

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
The robot has not yet been tested on a real field but on one created with the what wobsereve could used. The result can observed in the following video.

![](20200205_234115.gif)

The robot is perfectly achieving the strategie put in place. The fake ball created does not roll so is not able to reach easily the goal but this will not happend with a real ball.
