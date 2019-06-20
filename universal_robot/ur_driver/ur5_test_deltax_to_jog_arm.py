#!/usr/bin/env python

############################
# 2018 Nov 2019 by Inmo Jang
# This code tests if jog_arm package can work well even if I send delta[x,y,z,r,p,y] to it via sin/cos functions (and ultimately via LEAP). 
# If it is suceeeded then it implies that making delta[x,y,z,r,p,y] from LEAP in the existing code was wrong. 
############################

import time
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *

# For teleoperation
import sys, select, termios, tty 
import numpy as np

from sensor_msgs.msg import JointState # To receive the current state

reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

# For forward/inverse kinematics
from ur_kinematics import Kinematics
import math

# For Debugging: Should be deleted when committed
reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0,0,0,0,0,0]
Q2 = [3.14,0,0,0,0,0]
Q3 = [3.14,1.5,0,0,0,0]
Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

global kin, X_pre, rate, ur_status_local
ur_status_local = None
kin = Kinematics('ur5')

pi = math.pi
X_pre = np.transpose([[0, 1 ,0],[1, 0, 0],[0,0,-1]])    


def move_joint1_sine():   
    global ur_status_local
    pub = rospy.Publisher('/leap_motion/twist_input',TwistStamped,queue_size=1)
    pub_xyz = rospy.Publisher('/leap_motion/twist_xyzrpy',TwistStamped,queue_size=1)

    freq = 200
    rate = rospy.Rate(freq)


    twist_ = TwistStamped()
    twist_xyzrpy_ = TwistStamped()
     
    
    vel_xyzrpy = [0, 0, 0, 0, 0.3, 0.0]    # (m/s m/s m/s rad/s rad/s rad/s)
    period = 4     # sec
    end_time = 100 # sec
    time_step = np.linspace(0,end_time,freq*end_time+1)
    time_step_back = [0,0.5,1,1.5,2,2.5,3] # sec
    x = vel_xyzrpy[0]*np.sin(2*np.pi*(time_step-time_step_back[0])/period)
    y = vel_xyzrpy[1]*np.sin(2*np.pi*(time_step-time_step_back[1])/period)
    z = vel_xyzrpy[2]*np.sin(2*np.pi*(time_step-time_step_back[2])/period)
    roll = vel_xyzrpy[3]*np.sin(2*np.pi*(time_step-time_step_back[3])/period)
    pitch = vel_xyzrpy[4]*np.sin(2*np.pi*(time_step-time_step_back[4])/period)
    yaw = vel_xyzrpy[5]*np.sin(2*np.pi*(time_step-time_step_back[5])/period)

    scale = 9
    for i in range(len(time_step)):



        ## Similated Twist Angle Velocity
        twist_.twist.linear.x = 0 # x[i]
        twist_.twist.linear.y = 0 # y[i]
        twist_.twist.linear.z = 0 # z[i]
        twist_.twist.angular.x = 0 # roll[i]
        twist_.twist.angular.y = pitch[i]*scale
        twist_.twist.angular.z = 0 # yaw[i]

        time_now = rospy.get_rostime()
        twist_.header.stamp = time_now   
        pub.publish(twist_)
    
        ## 
        twist_xyzrpy_.twist.linear.x += twist_.twist.linear.x*1/freq
        twist_xyzrpy_.twist.linear.y += twist_.twist.linear.y*1/freq
        twist_xyzrpy_.twist.linear.z += twist_.twist.linear.z*1/freq
        twist_xyzrpy_.twist.angular.x += twist_.twist.angular.x*1/freq
        twist_xyzrpy_.twist.angular.y += twist_.twist.angular.y*1/freq/scale
        twist_xyzrpy_.twist.angular.z += twist_.twist.angular.z*1/freq   
        twist_xyzrpy_.header.stamp = time_now 
        pub_xyz.publish(twist_xyzrpy_)
        rate.sleep()



def main():
    try:   
        rospy.init_node("Test_delta_x_generator", anonymous=True ,disable_signals=True)    
 
        move_joint1_sine()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
