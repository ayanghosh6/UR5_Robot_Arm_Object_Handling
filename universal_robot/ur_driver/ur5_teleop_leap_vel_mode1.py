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


# For forward/inverse kinematics
from ur_kinematics import Kinematics
import math
import copy

###############################################################
###############################################################
Movement_scale_factor_gazebo = 1
Orientation_scale_factor_gazebo = 0.3

Movement_scale_factor_realrobot = 0.2
Orientation_scale_factor_realrobot = 0.15 #0.075

num_stack = 20 # Stack for Joint_speed value
num_stack_palm = 20 # Stack for Delta of Euler Angle of Hand

vel_safety_bound = 4*math.pi # (rad/s) Prevent movement above this value
###############################################################
###############################################################
###############################################################
###############################################################

global palm_centre_pre, palm_centre, grab_strength, kin, LeapMsg_local, ur_status_local, pub, teleoperation_mode
ur_status_local = None
palm_centre_pre = None
# teleoperation_mode = "MODE_1" # This was for testing purpose. Needs to be changed as '' 
teleoperation_mode = "" # This was for testing purpose. Needs to be changed as '' 

LeapMsg_local = None
grab_strength = 0
kin = Kinematics('ur5')


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


global palm_normal_stack, palm_direction_stack, palm_normal_mean, palm_direction_mean, palm_frame_0, Del_Euler_stack, Del_Euler_mean
global time_pre_cbLEAP, time_now_cbLEAP, palm_frame_pre

palm_normal_stack = None
palm_direction_stack = None
Del_Euler_stack = None
palm_normal_mean = None
palm_direction_mean = None
time_pre_cbLEAP = None
time_now_cbLEAP = None
palm_frame_0 = None
Del_Euler_mean = None
palm_frame_pre = None

def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode



def callback_Leap(status): # Output: Del_Euler_mean
    global LeapMsg_local, palm_normal_stack, palm_direction_stack, pub_palm_vec, g_palm_vec, palm_frame_0
    global time_now_cbLEAP, time_pre_cbLEAP, Del_Euler_mean
    global Del_Euler_stack, ur_status_local, palm_frame_pre
 
    time_now_cbLEAP = rospy.get_rostime()
    while time_now_cbLEAP is time_pre_cbLEAP:
        time_now_cbLEAP = rospy.get_rostime()        
    time_pre_cbLEAP = time_now_cbLEAP
    
    LeapMsg_local = status
    LeapMsg_local.header.stamp = time_now_cbLEAP

    # ur_status = copy.deepcopy(ur_status_local)

    if (LeapMsg_local.left_hand.is_present is True): # and ur_status is not None):  

        # Info From Unity via ROS# (Data type is a ROS msg)
        palm_normal_from_unity = LeapMsg_local.left_hand.palm_normal
        palm_direction_from_unity = LeapMsg_local.left_hand.palm_direction            
        # # Coordination Transformation to the ROS world (y and z should bereplaced with each other)
        palm_normal = np.array([palm_normal_from_unity.x,palm_normal_from_unity.z, palm_normal_from_unity.y])
        palm_direction = np.array([palm_direction_from_unity.x,palm_direction_from_unity.z, palm_direction_from_unity.y])

        # # Debug: Showing Palm Normal

        # if palm_normal_stack is None:
        #     palm_normal_stack = [palm_normal]*num_stack_palm # Initalise the stack for filtering the desired joint velocity
        #     palm_direction_stack = [palm_direction]*num_stack_palm # Initalise the stack for filtering the desired joint velocity   
                
        # else:                   
        #     palm_normal_stack.pop(0) 
        #     palm_normal_stack.append(palm_normal)
        #     palm_direction_stack.pop(0) 
        #     palm_direction_stack.append(palm_direction) 
                               
        # palm_normal_mean = np.mean(palm_normal_stack, axis = 0)
        # palm_direction_mean = np.mean(palm_direction_stack, axis = 0)    


        # g_palm_vec.header.stamp = time_now_cbLEAP

        # g_palm_vec.twist.linear.x = palm_direction_mean[0]
        # g_palm_vec.twist.linear.y = palm_direction_mean[1]
        # g_palm_vec.twist.linear.z = palm_direction_mean[2]
        # g_palm_vec.twist.angular.x = palm_normal_mean[0]
        # g_palm_vec.twist.angular.y = palm_normal_mean[1]
        # g_palm_vec.twist.angular.z = palm_normal_mean[2]
          
        
        # pub_palm_vec.publish(g_palm_vec)
        #
        
        nor_vec_ = palm_normal # This is a filtered value from callback_LEAP
        dir_vec_ = palm_direction
        dir_cross_nor_vec_ = np.cross(dir_vec_,nor_vec_)
        dir_vec_ = np.cross(nor_vec_,dir_cross_nor_vec_) # Directionalvector needs to be corrected as perpendicular. 
        x_ee_desired = np.asmatrix(nor_vec_)
        y_ee_desired = np.asmatrix((-dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2)
        z_ee_desired = np.asmatrix((dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2)       

        palm_frame = np.asmatrix(np.transpose([x_ee_desired, y_ee_desired, z_ee_desired]))


        if palm_frame_0 is None: # To Save Intial Palm/EE Frame             
            palm_frame_0 = copy.deepcopy(palm_frame)
            palm_frame_pre = copy.deepcopy(palm_frame_0)
            # ee_frame = get_ee_frame('gazebo',ur_status)
            # ee_frame_0 = copy.deepcopy(ee_frame)


        Del_Euler_ = get_Euler(palm_frame,palm_frame_pre)

        if Del_Euler_stack is None:
            Del_Euler_stack = [Del_Euler_]*num_stack_palm
        else:
            Del_Euler_stack.pop(0) 
            Del_Euler_stack.append(Del_Euler_)
        Del_Euler_mean = np.mean(Del_Euler_stack, axis = 0)

        palm_frame_pre = copy.deepcopy(palm_frame)

    else: # if not (LeapMsg_local.left_hand.is_present is True)
        palm_frame_0 = None
        Del_Euler_stack = None
        Del_Euler_mean = np.array([0, 0, 0])


def callback_UR(status):
    global ur_status_local
    ur_status_local = status


def get_ee_frame(model_flag, ur_status):

    # Get Current UR Status
    if model_flag == 'real':
        pose_current_ = np.array(ur_status.position) # For Real
    elif model_flag == 'gazebo':
        pose_current_ = np.array(ur_status.actual.positions) # For Gazebo
    
    pose_current = kin.inverse(kin.forward(pose_current_),pose_current_) # To avoid the case, IK(FK(pose_current)) != pose_current, which is wei
    # To avoid joint angles > pi, which may cause large del_joint afterwards
    for i in range(len(pose_current)):
        if pose_current[i] > math.pi:
            pose_current[i] -= 2*math.pi
        elif pose_current[i] < -math.pi:
            pose_current[i] += 2*math.pi

    q_guess = pose_current               # Use current joint angles asa guess value for IK
    pose_current_xyz = kin.forward(pose_current) # Convert the currentjoints status to x
    # Debug 1
    Dummy = pose_current - kin.inverse(pose_current_xyz, q_guess)
    for i in range(len(Dummy)):
        if np.abs(Dummy[i]) > 0.000001:
            while not rospy.is_shutdown():
                print("Problem with IK")
    
    ee_frame = copy.deepcopy(pose_current_xyz[0:3,0:3])
    return [ee_frame, pose_current, pose_current_xyz]

def get_Euler(X_current, X_ref): # Get Euler Angles in Global Frame
    gimbal_lock_flag = 0
    
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

    Euler = np.array([roll, pitch, yaw])
    return Euler

def get_Tmatrix(Euler):
    roll = Euler[0]
    pitch = Euler[1]
    yaw = Euler[2]


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

    return T

def palm_velocity_to_joint_velocity_mode1(myArg1):
    global kin
    global LeapMsg_local, ur_status_local, teleoperation_mode
    global g, g_Euler, g_Euler_EE
    global pub, pub_Euler, pub_Euler_EE
    global Del_Euler_mean
   
    publish_rate = 250
    rate = rospy.Rate(publish_rate)

    model_flag = myArg1
    if model_flag == 'real':
        scale_factor = Movement_scale_factor_realrobot
        scale_factor_ori = Orientation_scale_factor_realrobot
    elif model_flag == 'gazebo':
        scale_factor = Movement_scale_factor_gazebo
        scale_factor_ori = Orientation_scale_factor_gazebo
    else:
        print("Wrong Argument")
        scale_factor = 0
        scale_factor_ori = 0



    vel_0 = np.array([0,0,0,0,0,0])
    vel_joint_stack = [vel_0]*num_stack # Initalise the stack for filtering the desired joint velocity


    while rospy.get_time() == 0: # Waiting for receiving the first message from /clock
        time_now = rospy.get_rostime()
    time_now = rospy.get_rostime()     
    time_pre = time_now

    
    while not rospy.is_shutdown():



        if LeapMsg_local is not None and ur_status_local is not None and teleoperation_mode == "MODE_1":
   
            # Get Current Status
            leap_status = copy.deepcopy(LeapMsg_local)
            ur_status = copy.deepcopy(ur_status_local)
            time_now = rospy.get_rostime()


            while time_now is time_pre: # Sometimes, time_pre and time_now are identical, which is weird
                time_now = rospy.get_rostime()
            # if np.abs(time_now.nsecs - time_pre.nsecs) == 0: # Sometimes, time_pre and time_now are identical, which is weird
            #     time_now = rospy.get_rostime()
            
            
            if (leap_status.left_hand.is_present is True and Del_Euler_mean is not None):                

                ee_frame, pose_current, pose_current_xyz = get_ee_frame(model_flag, ur_status)

                pose_desired_xyz = copy.deepcopy(pose_current_xyz) # Initalise Desired Input as Current Status    





                # if palm_frame_0 is None: # To Save Intial Palm/EE Frame
                    # ee_frame_0 = copy.deepcopy(ee_frame)
                    # ee_frame_pre = copy.deepcopy(ee_frame)
 
                    # pose_current_pre = copy.deepcopy(pose_current)
                    # palm_frame_pre = copy.deepcopy(palm_frame_0)

                Del_Euler_mean_local = Del_Euler_mean
                Del_Euler = scale_factor_ori*Del_Euler_mean_local
                Del_Tmatrix = get_Tmatrix(Del_Euler)
                # Tmatrix_relative = palm_frame*np.linalg.inv(palm_frame_0)
                ee_frame_new = Del_Tmatrix*ee_frame

                pose_desired_xyz[0:3,0:3] = ee_frame_new
                # pose_desired_xyz[0:3,0] = np.transpose(x_ee) # X_ee =Palm Normal Vector Orientation (Base framepoint of view)
                # pose_desired_xyz[0:3,1] = np.transpose(y_ee) # Y_ee 
                # pose_desired_xyz[0:3,2] = np.transpose(z_ee) # Z_ee 
                
                ################ End- Orientation Control ###############
                #########################################################
                
                # time_to_reach_ = time_now - time_pre
                # time_to_reach = (time_to_reach_.secs + time_to_reach_.nsecs/1000000000.0)
                # if time_to_reach == 0:
                #     time_to_reach = 0.008
                time_to_reach = 0.008
                # print("Del_Euler = "+ str(Del_Euler) + "at Del_Time: " + str(time_to_reach)+"\n")


                #########################################################
                ################## Position Control ##################

                palm_velocity = leap_status.left_hand.palm_velocity

                # # For LEAP in Unity
                del_x = palm_velocity[0]*time_to_reach*scale_factor
                del_z = palm_velocity[1]*time_to_reach*scale_factor
                del_y = palm_velocity[2]*time_to_reach*scale_factor

                # For Testing
                # del_x = palm_velocity[0]*time_to_reach
                # del_y = palm_velocity[1]*time_to_reach
                # del_z = palm_velocity[2]*time_to_reach

                del_xyz = np.matrix([[0,0,0,del_x],[0,0,0,del_y],[0,0,0,del_z],[0,0,0,0]])


                pose_desired_xyz = np.add(pose_desired_xyz, del_xyz)  

                
                ################ End- Position Control ###############
                #########################################################
                                 
                weights_joints_kin = [10, 10, 10, 1, 1, 1]
                q_guess =pose_current
                pose_desired = kin.inverse(pose_desired_xyz,q_guess,weights=weights_joints_kin)                 
                if(pose_desired is None):
                    current_time = time.clock()
                    print("[!!!Inverse Kinematics Error!!!]: Please move the robot in a different direction " + str(current_time))
                    vel_joint = vel_0
                    while not rospy.is_shutdown():
                            print("Stop for safety reason!!!: IK Issue \n")
                    
                else:
                    # if time_to_reach == 0:
                    #     vel_joint = np.mean(vel_joint_stack, axis = 0)
                    #     print("Zero Velocity: due to time_to_reach is zero: " + str(time_now) + "\n")  
                    # else:
                    del_joint = pose_desired - pose_current
                    for i in range(len(pose_desired)):
                        if del_joint[i] > math.pi:
                            del_joint[i] -= 2*math.pi
                        elif del_joint[i] < -math.pi:
                            del_joint[i] += 2*math.pi
                    
                    vel_joint = (del_joint)/time_to_reach
                    ############# Debug
                    # Euler_palm = get_Euler(palm_frame,palm_frame_0) # Euler Angle from Initial Palm Frame                     
                    g_Euler.header.stamp = time_now
                    g_Euler.twist.angular.x = Del_Euler_mean_local[0]
                    g_Euler.twist.angular.y = Del_Euler_mean_local[1]
                    g_Euler.twist.angular.z = Del_Euler_mean_local[2]
                    # Euler_EE = get_Euler(ee_frame,ee_frame_0) # Euler Angle from Initial EE Frame
                    # g_Euler_EE.header.stamp = time_now
                    # g_Euler_EE.twist.angular.x = Euler_EE[0]
                    # g_Euler_EE.twist.angular.y = Euler_EE[1]
                    # g_Euler_EE.twist.angular.z = Euler_EE[2]
                    # print("Del_Euler_EE = "+ str(Delta_Euler_EE) + "at Del_Time: " + str(time_now.to_sec)+"\n")
                    ############# END - Debug
                    

                 
                 
                # Filter
                vel_joint_stack.pop(0) 
                vel_joint_stack.append(vel_joint) 
                vel_joint_mean = np.mean(vel_joint_stack, axis = 0)
                # vel_joint_mean = vel_joint


                # Debug
                if np.abs(vel_joint_mean.max()) > vel_safety_bound or np.abs(vel_joint_mean.min()) > vel_safety_bound:
                    while not rospy.is_shutdown():
                        print("Stop for safety reason: Velocity input is too high!!!")


                # Send the target position info
                g.header.stamp = time_now 
                g.points = [
                            JointTrajectoryPoint(velocities=vel_joint_mean)]

                # palm_frame_pre = copy.deepcopy(palm_frame)
                # ee_frame_pre = copy.deepcopy(ee_frame)

            else: # if not (leap_status.left_hand.is_present is True):
                g.header.stamp = time_now 
                g.points = [
                            JointTrajectoryPoint(velocities=[0,0,0,0,0,0])]

                # Debug
                # palm_frame_0 = None
                g_Euler.header.stamp = copy.deepcopy(time_now)
                g_Euler.twist.angular.x = 0
                g_Euler.twist.angular.y = 0
                g_Euler.twist.angular.z = 0    
                print("Zero Velocity: left_hand is not present: " + str(time_now) + "\n")    
                vel_joint_stack = [vel_0]*num_stack # Initalise the stack for filtering the desired joint velocity
                    

            
            pub.publish(g)
            pub_Euler.publish(g_Euler)
            # pub_Euler_EE.publish(g_Euler_EE)
            
            # time_pre = copy.deepcopy(time_now)

            rate.sleep()

        else: # if not teleoperation_mode = 'MODE_1'
            print("LEAP Signal is not coming (Mode 1)\n")
            # palm_frame_0 = None
            vel_joint_stack = [vel_0]*num_stack


def main(myArg1):
    global g, g_Euler, g_Euler_EE
    global pub, pub_Euler, pub_Euler_EE
    global pub_palm_vec, g_palm_vec

    if (myArg1 == 'gazebo') or (myArg1 == 'real'):
        try:

            rospy.init_node("ur5_control_mode1", anonymous=True, disable_signals=True)
                
            pub = rospy.Publisher('/ur_driver/joint_speed_',JointTrajectory,queue_size=1)
            pub_Euler = rospy.Publisher('/debug/euler_input',TwistStamped,queue_size=1) # This is for debug (angular position input)
            pub_Euler_EE = rospy.Publisher('/debug/euler_ee',TwistStamped,queue_size=1) # This is for debug (angular position output)

            # Debug
            pub_palm_vec = rospy.Publisher('/debug/palm_normal',TwistStamped,queue_size=1) # This is for debug (angular position input)


            # Subscribing information from Unity
            rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)          
            # Subscribing information from LEAP
            rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg, callback_Leap, queue_size=1)

            if myArg1 == 'gazebo':
                # rospy.Subscriber("/joint_states_gazebo",JointState, callback_UR, queue_size=1)
                rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState, callback_UR, queue_size=1) 

            elif myArg1 == 'real':             
                # Subscribing information from UR5
                rospy.Subscriber("/joint_states",JointState, callback_UR, queue_size=1)
            
            g = JointTrajectory()
            g.joint_names = JOINT_NAMES

            g_Euler = TwistStamped()
            g_Euler_EE = TwistStamped()
            g_palm_vec = TwistStamped()

            palm_velocity_to_joint_velocity_mode1(myArg1)
        
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
    
    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'gazebo' or 'real'")


if __name__ == '__main__': 
    
    # # This is for real
    if len(sys.argv) < 2:
        print("Usage: ur5_teleop_leap_vel_mode1.py [gazebo] or [real]")
    else:
        main(sys.argv[1])
        
    # This is for debug  
    # main('gazebo')