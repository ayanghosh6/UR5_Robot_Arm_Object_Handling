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

global kin, X_ref, ur_status_local
ur_status_local = None
kin = Kinematics('ur5')
pi = math.pi
X_ref = np.transpose([[1, 0 ,0],[0, 1, 0],[0,0,1]])    # Reference Frame (Body Frame)



def callback_UR(status):
    global ur_status_local
    ur_status_local = status

def pub_xyzrpy_status():   
    global X_ref, ur_status_local

    pub_status = rospy.Publisher('/ur_driver/status',TwistStamped,queue_size=1)
    twist_current = TwistStamped()

    freq = 500
    rate = rospy.Rate(freq)

    

    while not rospy.is_shutdown():
        # Get Current UR Status 
        ur_status = ur_status_local    
        if ur_status is not None:
            gimbal_lock_flag = 0
            pose_current = ur_status.position
            pose_current_xyz = kin.forward(pose_current) 
            X_current = pose_current_xyz[0:3,0:3]
            # Calculate r p y
            X_current_X_ref_inv = np.matmul(X_current, np.linalg.inv(X_ref))


            # Roll => Output range: -2pi to 2pi
            m_cos_p_sin_r = X_current_X_ref_inv[1,2]
            cos_p_cos_r = X_current_X_ref_inv[2,2]
            if m_cos_p_sin_r == 0 and cos_p_cos_r == 0: # Gimbal lock case
                print("Gimbal lock happens at Roll! \n")
                gimbal_lock_flag = 1
            else:
                roll = math.atan2(-m_cos_p_sin_r,cos_p_cos_r)


            
            # Pitch => -pi to pi. because it is not easy to get minus cos_p) 
            sin_p = X_current_X_ref_inv[0,2]
            sin_r = math.sin(roll)
            cos_r = math.cos(roll)
            cos_p = cos_p_cos_r/cos_r
            # cos_p = -m_cos_p_sin_r/sin_r
            # cos_p = math.sqrt(m_cos_p_sin_r**2 + cos_p_cos_r**2)
            pitch = math.atan2(sin_p,cos_p)
            
            # Yaw => -2pi to 2pi
            m_cos_p_sin_y = X_current_X_ref_inv[0,1]
            cos_p_cos_y = X_current_X_ref_inv[0,0]
            if m_cos_p_sin_y == 0 and cos_p_cos_y == 0: # Gimbal lock case
                print("Gimbal lock happens at Yaw! \n")
                gimbal_lock_flag = 1
            else:
                yaw = math.atan2(-m_cos_p_sin_y,cos_p_cos_y)

            # Publish
            if gimbal_lock_flag == 0:  
                twist_current.twist.linear.x = pose_current_xyz[0,3]
                twist_current.twist.linear.y = pose_current_xyz[1,3]
                twist_current.twist.linear.z = pose_current_xyz[2,3]
                twist_current.twist.angular.x = roll
                twist_current.twist.angular.y = pitch
                twist_current.twist.angular.z = yaw
                time_now = rospy.get_rostime()
                twist_current.header.stamp = time_now   
                pub_status.publish(twist_current)    
                rate.sleep()
            

  


def main():
    try:   
        rospy.init_node("ur_xyzrpy_status_publisher", anonymous=True ,disable_signals=True)    
       
        rospy.Subscriber("/joint_states",JointState, callback_UR, queue_size=1)
        
        pub_xyzrpy_status()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
