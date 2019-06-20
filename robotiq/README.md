# Robotiq 3-finger Gripper 

## Overview

This is a Robotiq 3-finger Gripper package for RAIN hub project. 

This pakcage combines the followings with necessary modifications:

- Robotiq's h/w interface package(indigo version) (https://github.com/ros-industrial/robotiq)
- A gazebo pakcage of Robotiq 3-finger gripper from Wei Cheah's repository (https://bitbucket.org/wilsonz91/robotiq/src/master/) 
- Debugged Individual Control Mode (use of rICF)

## Installation

##### Clone the package: 
	
	git clone https://inmojang@bitbucket.org/inmojang/robotiq.git

##### Things required as in http://wiki.ros.org/robotiq :

Once cloned, 
	
    rosdep install robotiq_modbus_tcp
    sudo apt-get install ros-kinetic-soem
	
##### Then, catkin_make




