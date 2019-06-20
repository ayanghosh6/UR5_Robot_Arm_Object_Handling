# Universal Robot Package for RAIN

## Version Information

Version 1.1 uses effort_controllers for joint control of UR5. (Ver 1.0 uses position_controllers). 
This update enables the gazebo model to pick and grasp an object in the gazebo environment. 
In Ver 1.0, this was not the case. 
Instead, it becomes necessary to set PID gains, which were just set by trial and errors in this version.

## Overview

Addition/modification of some files into the original Universal Robot package (https://github.com/ros-industrial/universal_robot), in order to be used for a teleorator in RAIN hub (https://rainhub.org.uk/). 

## Installation

Install the required dependencies as follows:

##### Things required as in http://wiki.ros.org/universal_robot
	
    sudo apt-get install ros-kinetic-universal-robot
    sudo apt-get install ros-kinetic-moveit
	
##### Then, install

	git clone https://inmojang@bitbucket.org/rain_epsrc/universal_robot.git
	
	
	
## Changes/Additions

### (1) Addition of Gazebo UR model controller

#### Background
For some reasons, it would be desirable to test and evaluate a python/c++ controller (e.g., a passivity-based teleoepration controller) for a real hardware robot, by using the corresponding gazebo model in simulation. 
To this end, the gazebo model should have the same interface as that of the real robot. If you want to control the robot using Movit, then using the existing universal robot package seems fine. 
However, for haptic teleoperation research, it is required to allow the gazebo model to be controlled via a python controller that we are going to develop for the real robot. 
Although the existing package does not have such an interface explicitly, it is quite straightforward to do so by modifying one line in test_move.py, which was originally developed to control the real hardware. 
The modified file is named here "gazebo_move.py", and its modified line is as follows. 
	
	
		# client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
		
#### Usage

How to use "gazebo_move.py" is identical to that of "test_move.py". After launching the gazebo model, 

		rosrun ur_driver gazebo_move.py
	

### (2) Addition of forward/inverse kinematics (using a python wrapper based on boost numpy)
Some files are extracted from http://wiki.ros.org/ur_kin_py and https://github.com/ndarray/Boost.NumPy

#### Usage
Please refer to http://wiki.ros.org/ur_kin_py

		from ur_kinematics import Kinematics
		kin = Kinematics('ur5') # or ur10
		a = kin.forward([0]*6)
		print a

		b = kin.inverse(a)
		print b

### (3) End effector position control via LEAP motion (15 Oct 2018)

		rosrun ur_driver gazebo_teleop_leap.py

### (4) UR modern driver added (22 Oct 2018)

The new driver (https://github.com/ros-industrial/ur_modern_driver) was added as shown in Section 3.5 in http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial

		
		
		
---

The following is the original description for Universal Robot package (https://github.com/ros-industrial/universal_robot)

----------


# Universal Robot

[![Build Status](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](http://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot


__Installation from Source__  
There are releases available for ROS Hydro and ROS Indigo. However, for the latest features and developments you might want to install from source.

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/universal_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.  

---

__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```

A simple test script that moves the robot to predefined positions can be executed like this:

```rosrun ur_driver test_move.py```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


__MoveIt! with real Hardware__  
Additionally, you can use MoveIt! to control the robot.  
There exist MoveIt! configuration packages for both robots.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


---

__Usage with Gazebo Simulation__  
There are launch files available to bringup a simulated robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur5.launch```


__MoveIt! with a simulated robot__  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


