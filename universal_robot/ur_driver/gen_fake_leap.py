#!/usr/bin/env python
import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from leap_motion.msg import Human_orion  as LeapMsg # for LEAP in Unity
from sensor_msgs.msg import JointState # To receive the current state

import time
import numpy as np



def fake_leap_generator():   

    pub = rospy.Publisher('/leap_motion/leap_filtered',LeapMsg,queue_size=1)
    
 
    pub_rate = 500
    rate = rospy.Rate(pub_rate)

    Human_operator = LeapMsg()

    palm_normal_0 = [[1], [0], [0]] # Initial Desired Palm Normal (based on Base frame)
    palm_direction_0 = [[0], [1], [0]] # Initial Desired Palm Directional (based on Base frame)

    # Generate X,Y,Z and Euler Angle Variation
    xyzrpy = [0.1, 0.1, 0.1, 0.3, 0.1, 0.2]    # (m/s) XYZ velocity // (rad) Amplitudes of Roll Pitch Yaw in End Effector Frame
    period = 2     # Repetetive period (sec)
    end_time = 1000 # sec
    time_step = np.linspace(0,end_time,pub_rate*end_time+1)
    time_step_back = [0, 0, 0, 0 ,0 , 0] # sec

    # # Method 1
    # x = xyzrpy[0]*np.sin(2*np.pi*(time_step-time_step_back[0])/period)
    # y = xyzrpy[1]*np.sin(2*np.pi*(time_step-time_step_back[1])/period)
    # z = xyzrpy[2]*np.sin(2*np.pi*(time_step-time_step_back[2])/period)

    # roll = xyzrpy[3]*np.sin(2*np.pi*(time_step-time_step_back[0])/period)
    # pitch = xyzrpy[4]*np.sin(2*np.pi*(time_step-time_step_back[1])/period)
    # yaw = xyzrpy[5]*np.sin(2*np.pi*(time_step-time_step_back[2])/period)


    # for i in range(len(time_step)):
    

    #     a11 = np.cos(pitch[i])*np.cos(yaw[i])
    #     a12 = -np.cos(pitch[i])*np.sin(yaw[i])
    #     a13 = np.sin(pitch[i])
    #     a21 = np.cos(roll[i])*np.sin(yaw[i])+np.cos(yaw[i])*np.sin(roll[i])*np.sin(pitch[i])
    #     a22 = np.cos(roll[i])*np.cos(yaw[i])-np.sin(roll[i])*np.sin(pitch[i])*np.sin(yaw[i])
    #     a23 = -np.cos(pitch[i])*np.sin(roll[i])
    #     a31 = np.sin(roll[i])*np.sin(yaw[i])-np.cos(roll[i])*np.cos(yaw[i])*np.sin(pitch[i])
    #     a32 = np.cos(yaw[i])*np.sin(roll[i])+np.cos(roll[i])*np.sin(pitch[i])*np.sin(yaw[i])
    #     a33 = np.cos(roll[i])*np.cos(pitch[i])         


    #     T = np.matrix([[a11, a12, a13],[a21, a22, a23],[a31, a32, a33]])
    
    #     palm_normal = np.matmul(T, palm_normal_0)
    #     palm_direction = np.matmul(T, palm_direction_0)
    #     ## Palm Normal 
    #     Human_operator.left_hand.palm_normal.x = palm_normal[0]
    #     Human_operator.left_hand.palm_normal.y = palm_normal[1]
    #     Human_operator.left_hand.palm_normal.z = palm_normal[2]

    #     ## Palm Direction
    #     Human_operator.left_hand.palm_direction.x = palm_direction[0]
    #     Human_operator.left_hand.palm_direction.y = palm_direction[1]
    #     Human_operator.left_hand.palm_direction.z = palm_direction[2]

    #     ## Current Time
    #     time_now = rospy.get_rostime()
    #     Human_operator.header.stamp = time_now
        

    #     ## Palm Center Velocity
    #     Human_operator.left_hand.palm_velocity = [x[i], y[i], z[i]]

    #     ## etc info
    #     Human_operator.left_hand.is_present = True

    #     pub.publish(Human_operator)
    #     rate.sleep()



    # Method 2
    while rospy.get_time() == 0:
        time_0 = rospy.get_rostime()

    while not rospy.is_shutdown():
        
        time_now = rospy.get_rostime()

        time_step = time_now.to_sec() - time_0.to_sec()

        x = xyzrpy[0]*np.sin(2*np.pi*(time_step-time_step_back[0])/period)
        y = xyzrpy[1]*np.sin(2*np.pi*(time_step-time_step_back[1])/period)
        z = xyzrpy[2]*np.sin(2*np.pi*(time_step-time_step_back[2])/period)

        roll = xyzrpy[3]*np.sin(2*np.pi*(time_step-time_step_back[0])/period)
        pitch = xyzrpy[4]*np.sin(2*np.pi*(time_step-time_step_back[1])/period)
        yaw = xyzrpy[5]*np.sin(2*np.pi*(time_step-time_step_back[2])/period)

        a11 = np.cos(pitch)*np.cos(yaw)
        a12 = -np.cos(pitch)*np.sin(yaw)
        a13 = np.sin(pitch)
        a21 = np.cos(roll)*np.sin(yaw)+np.cos(yaw)*np.sin(roll)*np.sin(pitch)
        a22 = np.cos(roll)*np.cos(yaw)-np.sin(roll)*np.sin(pitch)*np.sin(yaw)
        a23 = -np.cos(pitch)*np.sin(roll)
        a31 = np.sin(roll)*np.sin(yaw)-np.cos(roll)*np.cos(yaw)*np.sin(pitch)
        a32 = np.cos(yaw)*np.sin(roll)+np.cos(roll)*np.sin(pitch)*np.sin(yaw)
        a33 = np.cos(roll)*np.cos(pitch)
        
                 
        T = np.matrix([[a11, a12, a13],[a21, a22, a23],[a31, a32, a33]])
        palm_normal = np.matmul(T, palm_normal_0)
        palm_direction = np.matmul(T, palm_direction_0)
        ## Palm Normal 
        Human_operator.left_hand.palm_normal.x = palm_normal[0]
        Human_operator.left_hand.palm_normal.y = palm_normal[1]
        Human_operator.left_hand.palm_normal.z = palm_normal[2]
        ## Palm Direction
        Human_operator.left_hand.palm_direction.x = palm_direction[0]
        Human_operator.left_hand.palm_direction.y = palm_direction[1]
        Human_operator.left_hand.palm_direction.z = palm_direction[2]
        ## Current Time
        # time_now = rospy.get_rostime()
        Human_operator.header.stamp = time_now
        
        ## Palm Center Velocity
        Human_operator.left_hand.palm_velocity = [x, y, z]
        ## etc info
        Human_operator.left_hand.is_present = True
        pub.publish(Human_operator)

        rate.sleep()
    

def main():
    try:   
        rospy.init_node('fake_leap_generator', anonymous=True ,disable_signals=True)    
 
        fake_leap_generator()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()



############################
# 2018 Nov 30 by Inmo Jang
# This code generates a stream of fake leap motion information (palm_normal palm_direction) to test any of control node. 
############################