#!/usr/bin/env python
import time
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

# For teleoperation
import sys, select, termios, tty 
import numpy as np

from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity
from sensor_msgs.msg import JointState # To receive the current state of UR
# from leap_motion.msg import Human  as LeapMsg # To receive the info from LEAP
from leap_motion.msg import Human_orion  as LeapMsg # To receive the info from LEAP
from geometry_msgs.msg import *

import math
import copy

global palm_centre_pre, palm_centre, grab_strength, kin, LeapMsg_local, ur_status_local, pub, teleoperation_mode
ur_status_local = None
palm_centre_pre = None
teleoperation_mode = ""
LeapMsg_local = None
grab_strength = 0


angular_bound = 360 # (deg/s)
Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

client = None

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']




def length(v):
  return math.sqrt(np.inner(v, v))

def angle(v1, v2):
  return math.acos(np.inner(v1, v2) / (length(v1) * length(v2)))*180/math.pi

def angle_projected(v1, v2, index):
  v1_projected = copy.deepcopy(v1)  
  v1_projected[index] = 0
  v2_projected = copy.deepcopy(v2)
  v2_projected[index] = 0

  return math.acos(np.inner(v1_projected, v2_projected) / (length(v1_projected) * length(v2_projected)))*180/math.pi

def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def callback_Leap(status):
    global LeapMsg_local
    LeapMsg_local = status
    LeapMsg_local.header.stamp = rospy.get_rostime()

def callback_UR(status):
    global ur_status_local
    ur_status_local = status

def calc_leap_to_twist():
    global LeapMsg_local, ur_status_local, teleoperation_mode
    global pub, pub_r, pub_p, pub_y
   

    twist_ = TwistStamped()
    twist_r = TwistStamped()
    twist_p = TwistStamped()
    twist_y = TwistStamped()


    freq = 250
    time_to_reach = 0.008
    rate = rospy.Rate(freq)

    num_stack = 40

    vel_0 = np.array([0,0,0,0,0,0])
    vel_joint_stack = [vel_0]*num_stack # Initalise the stack for filtering the desired joint velocity

    vec_0 = np.array([0,0,0])
    palm_normal_stack = [vec_0]*num_stack
    palm_direction_stack = [vec_0]*num_stack
    palm_velocity_stack = [vec_0]*num_stack

    stack_filled_flag = 0
    stack_filled_flag_xyz = 0
    time_pre = None
    delta_time = 0
    delta_time_upperbound = 0.1
    while not rospy.is_shutdown():
        if LeapMsg_local is not None and ur_status_local is not None and teleoperation_mode == "MODE_1":
            
            status_local = LeapMsg_local
            if (status_local.left_hand.is_present is True):
                time_now = status_local.header.stamp
                palm_velocity = status_local.left_hand.palm_velocity

                twist_.twist.linear.x = palm_velocity[0]
                twist_.twist.linear.z = palm_velocity[1]
                twist_.twist.linear.y = palm_velocity[2]
                

                # # Get Current UR Status
                # ur_status = ur_status_local
                # # pose_current = ur_status.position 
                # pose_current = ur_status.actual.positions # (for Gazebo Model)        
                # pose_current_xyz = kin.forward(pose_current) # Convert the currentjoints status to xyz



                #########################################################
                ################## Orientation Control ##################
                # Info From Unity via ROS# (Data type is a ROS msg)
                palm_normal_from_unity = status_local.left_hand.palm_normal
                palm_direction_from_unity = status_local.left_hand.palm_direction            
                # Coordination Transformation to the ROS world (y and z should bereplaced with each other)
                palm_normal = np.array([palm_normal_from_unity.x,palm_normal_from_unity.z, palm_normal_from_unity.y])
                palm_direction = np.array([palm_direction_from_unity.x,palm_direction_from_unity.z, palm_direction_from_unity.y])
                                

                # Filter
                test = palm_normal_stack.pop(0) 
                palm_normal_stack.append(palm_normal) 
                palm_direction_stack.pop(0) 
                palm_direction_stack.append(palm_direction) 

                if np.linalg.norm(test) != 0:
                    stack_filled_flag = 1
                              

                
                if stack_filled_flag == 1:                    
                    palm_normal_mean = np.mean(palm_normal_stack, axis = 0)  
                    palm_direction_mean = np.mean(palm_direction_stack, axis = 0)



                    nor_vec_ = palm_normal_mean
                    dir_vec_ = palm_direction_mean
                    dir_cross_nor_vec_ = np.cross(dir_vec_,nor_vec_)
                    dir_vec_ = np.cross(nor_vec_,dir_cross_nor_vec_) # Directionalvector needs to be corrected as perpendicular. 
                    x_ee_current_ = (nor_vec_)
                    x_ee_current = x_ee_current_/np.linalg.norm(x_ee_current_)
                    y_ee_current_ = ((-dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2)
                    y_ee_current = y_ee_current_/np.linalg.norm(y_ee_current_)
                    z_ee_current_ = ((dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2)
                    z_ee_current = z_ee_current_/np.linalg.norm(z_ee_current_)

                    if time_pre is None:
                        delta_time = delta_time_upperbound + 10
                    else:
                        delta_time = time_now.to_sec() - time_pre.to_sec()

                    if (delta_time < delta_time_upperbound) and (delta_time > 0):                    

                        X_current = np.transpose([x_ee_current, y_ee_current, z_ee_current])
                        X_pre = np.transpose([x_ee_pre, y_ee_pre, z_ee_pre])

                        X_current_X_pre_inv = np.matmul(X_current, np.linalg.inv(X_pre))
                        # Pitch
                        sin_pitch = X_current_X_pre_inv[0,2]
                        pitch = math.asin(sin_pitch)
                        # Yaw
                        cos_pitch = math.cos(pitch)
                        cos_p_sin_y = X_current_X_pre_inv[0,0]
                        cos_yaw = cos_p_sin_y/cos_pitch
                        if cos_yaw > 1:
                            cos_yaw = 1

                        yaw = math.acos(cos_yaw)
                        # Roll
                        cos_p_cos_r = X_current_X_pre_inv[2,2]
                        cos_roll = cos_p_cos_r/cos_pitch
                        if cos_roll > 1:
                            cos_roll = 1
                        roll = math.acos(cos_roll)
                        # roll 

                        roll_to_send = roll*180/math.pi
                        if roll_to_send > angular_bound:
                            roll_to_send = angular_bound
                        elif roll_to_send < -angular_bound:
                            roll_to_send = - angular_bound

                        pitch_to_send = pitch*180/math.pi                        
                        if pitch_to_send > angular_bound:
                            pitch_to_send = angular_bound
                        elif pitch_to_send < -angular_bound:
                            pitch_to_send = - angular_bound

                        yaw_to_send = yaw*180/math.pi 
                        if yaw_to_send > angular_bound:
                            yaw_to_send = angular_bound
                        elif yaw_to_send < -angular_bound:
                            yaw_to_send = - angular_bound

                        twist_.twist.angular.x = roll_to_send*math.pi/180/delta_time
                        twist_.twist.angular.y = pitch_to_send*math.pi/180/delta_time
                        twist_.twist.angular.z = yaw_to_send*math.pi/180/delta_time

                    elif delta_time == 0.0:
                        
                        print("LEAP data hasn't arrived yet at " + str(time_now.to_sec()) + "\n")

                    else:
                        print("Delta time is too long " + str(delta_time) + " at " + str(time_now.to_sec()) + "\n")
                        twist_.twist.angular.x = 0
                        twist_.twist.angular.y = 0
                        twist_.twist.angular.z = 0


                    twist_.header.stamp = time_now 
                    
                    x_ee_pre = copy.deepcopy(x_ee_current)
                    y_ee_pre = copy.deepcopy(y_ee_current)
                    z_ee_pre = copy.deepcopy(z_ee_current)
                    time_pre = copy.deepcopy(time_now)
                
                    pub.publish(twist_)
                    rate.sleep()
            
            else:
                stack_filled_flag = 0
                palm_normal_stack = [vec_0]*num_stack
                palm_direction_stack = [vec_0]*num_stack
                palm_velocity_stack = [vec_0]*num_stack
        else:
            stack_filled_flag = 0
            palm_normal_stack = [vec_0]*num_stack
            palm_direction_stack = [vec_0]*num_stack
            palm_velocity_stack = [vec_0]*num_stack   


def main():
    global pub


    try:

        rospy.init_node("leap_to_twist", anonymous=True, disable_signals=True)
             
        pub = rospy.Publisher('/leap_motion/twist_input',TwistStamped,queue_size=1)



        # Subscribing information from Unity
        rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)          
        # Subscribing information from LEAP
        rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg, callback_Leap, queue_size=1)
        # Subscribing information from UR5
        rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState, callback_UR, queue_size=1)
        # rospy.Subscriber("/joint_states",JointState, callback_UR, queue_size=1)


        calc_leap_to_twist()
       
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
