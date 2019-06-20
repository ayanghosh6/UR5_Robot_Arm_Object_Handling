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
Movement_scale_factor_realrobot = 0.4 # 0.3  # Scaling the joint velocity input (For Real)
Movement_scale_factor_gazebo = 1 # Scaling the joint velocity input (For Gazebo)
num_stack = 20
vel_safety_bound = 4*math.pi # (rad/s) Prevent movement above this value
###############################################################
###############################################################
###############################################################
###############################################################


global palm_centre, LeapMsg_local, ur_status_local, pub, teleoperation_mode
ur_status_local = None
teleoperation_mode = ""
LeapMsg_local = None
kin = Kinematics('ur5')

Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

client = None

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def callback_Leap(status):
    global LeapMsg_local
    LeapMsg_local = status

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


def palm_velocity_to_joint_velocity(myArg1, pub):
    global LeapMsg_local, ur_status_local, teleoperation_mode
   
    kin = Kinematics('ur5')
    
    g = JointTrajectory()
    g.joint_names = JOINT_NAMES

    publish_rate = 1000    
    rate = rospy.Rate(publish_rate)

    time_to_reach = 0.008
    model_flag = myArg1    
    if model_flag == 'real':
        scale_factor = Movement_scale_factor_realrobot
    elif model_flag == 'gazebo':
        scale_factor = Movement_scale_factor_gazebo 
    else:
        print("Wrong Argument")
        scale_factor = 0

    vel_0 = np.array([0,0,0,0,0,0])
    vel_joint_stack = [vel_0]*num_stack # Initalise the stack for filtering the desired joint velocity

    while rospy.get_time() == 0: # Waiting for receiving the first message from /clock
        time_now = rospy.get_rostime() 

    while not rospy.is_shutdown():

        if LeapMsg_local is not None and ur_status_local is not None and teleoperation_mode == "MODE_0":
            
            # Get Current Status            
            leap_status = copy.deepcopy(LeapMsg_local)
            ur_status = copy.deepcopy(ur_status_local)
            time_now = rospy.get_rostime()            

            if (leap_status.left_hand.is_present is True):
                grab_strength = leap_status.left_hand.grab_strength
                if (grab_strength > 0.6):
                    palm_velocity = leap_status.left_hand.palm_velocity
                    # # By LEAP in ROS
                    # del_y = -palm_velocity[0]*time_to_reach
                    # del_z = palm_velocity[1]*time_to_reach
                    # del_x = -palm_velocity[2]*time_to_reach
                    # By LEAP in Unity
                    del_x = palm_velocity[0]*time_to_reach*scale_factor
                    del_z = palm_velocity[1]*time_to_reach*scale_factor
                    del_y = palm_velocity[2]*time_to_reach*scale_factor

                    del_xyz = np.matrix([[0,0,0,del_x],[0,0,0,del_y],[0,0,0,del_z],[0,0,0,0]])


                    ee_frame, pose_current, pose_current_xyz = get_ee_frame(model_flag, ur_status)


                    pose_desired_xyz = np.add(pose_current_xyz, del_xyz)
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
                        del_joint = pose_desired - pose_current
                        for i in range(len(pose_desired)):
                            if del_joint[i] > math.pi:
                                del_joint[i] -= 2*math.pi
                            elif del_joint[i] < -math.pi:
                                del_joint[i] += 2*math.pi
                        

                        vel_joint = (del_joint)/time_to_reach


                    # Filter
                    vel_joint_stack.pop(0) 
                    vel_joint_stack.append(vel_joint) 
                    vel_joint_mean = np.mean(vel_joint_stack, axis = 0)


                    # Debug
                    if np.abs(vel_joint_mean.max()) > vel_safety_bound or np.abs(vel_joint_mean.min()) > vel_safety_bound:
                        while not rospy.is_shutdown():
                            print("Stop for safety reason: Velocity input is too high!!!")

                    # Send the target position info
                    g.header.stamp = time_now 
                    g.points = [
                                JointTrajectoryPoint(velocities=vel_joint_mean)]
                                
                else: # if not (grab_strength > 0.9):
                    print("grab_strength is " + str(grab_strength) + ", which is not enough at time" +  str(time_now) + "\n")
                    g.header.stamp = time_now 
                    g.points = [
                                JointTrajectoryPoint(velocities=[0,0,0,0,0,0])]
                    vel_joint_stack = [vel_0]*num_stack

            else: # if not (leap_status.left_hand.is_present is True):
                g.header.stamp = time_now 
                g.points = [
                            JointTrajectoryPoint(velocities=[0,0,0,0,0,0])]
                vel_joint_stack = [vel_0]*num_stack
            
            pub.publish(g)
            rate.sleep()

        else:
            # print("LEAP Signal is not coming (Mode 0)\n")
            vel_joint_stack = [vel_0]*num_stack




def main(myArg1):
    print(myArg1)
    if (myArg1 == 'gazebo') or (myArg1 == 'real'):
        
        try:

            rospy.init_node("ur_control_mode0",anonymous=True,  disable_signals=True)
            pub = rospy.Publisher('/ur_driver/joint_speed_',JointTrajectory, queue_size=1)

            # Subscribing information from Unity
            rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)          
            # Subscribing information from LdEAP
            rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg, callback_Leap, queue_size=1)
            # Subscribing information from UR5 (You need to select one of the belows)
            if myArg1 == 'gazebo':
                rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState, callback_UR, queue_size=1) 
            elif myArg1 == 'real':
                rospy.Subscriber("/joint_states",JointState, callback_UR, queue_size=1) # For real robot
            
                    



            palm_velocity_to_joint_velocity(myArg1, pub)
       
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'gazebo' or 'real'")
        
    

if __name__ == '__main__': 
    
    # This is for real
    if len(sys.argv) < 2:
        print("Usage: ur5_teleop_leap_vel_mode0.py [gazebo] or [real]")
    else:
        main(sys.argv[1])

    # # This is for debug  
    # main('gazebo')        