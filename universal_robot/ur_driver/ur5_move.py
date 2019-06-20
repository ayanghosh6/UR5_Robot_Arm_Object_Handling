#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState # To receive the current state
from geometry_msgs.msg import Twist 
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import String
# For teleoperation
import sys, select, termios, tty 
import numpy as np

# For forward/inverse kinematics
from ur_kinematics import Kinematics
import math
import copy

#for gripping
import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from time import sleep

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



global kin, ur_status_local, position_box_local, pub, command,flag_active_robot, command_to_execute
position_box_local = None
ur_status_local = None
command_to_execute = None
flag_active_robot = False
kin = Kinematics('ur5')

Q0 = [-0.24434930482973272, -1.5705564657794397, 1.6586036682128906, -1.7131436506854456, -1.5757301489459437, 2.0950961112976074]
Q1 = [-0.077033821736471, -1.7938678900348108, 2.388179302215576, -0.5992730299579065, 1.5730737447738647, 0.883016049861908] #[0,0,0,0,0,0]
Q2 = [-0.6978743712054651, -1.7263992468463343, 1.80125093460083, -0.12365466753114873, 0.9547401666641235, 0.8828726410865784] # [3.14,0,0,0,0,0]
Q3 = [0.29904013872146606, -1.517637077962057, 1.7205395698547363, -0.2273772398578089, 1.9801467657089233, 0.8827288150787354]
# [3.14,1.5,0,0,0,0]

P1 = [0.53131966009, 0.207277888813, 0.343564112929]
P2 = [0.635731702922, -0.0187221912166, 0.209686042357]
P3 = [0.61462452551, -0.137835219584, 0.393898536678]

#ObjectsPosition

Object_Position=[[0.808153673419, -0.11590057548, -0.0485741928011],[0.820029211552, 0.17391434705, -0.0535285562353]]


client = None

def callback_UR(status):
    global ur_status_local
    ur_status_local = status

def callback_virtualbox(status):
    global position_box_local, flag_active_robot, command_to_execute
    position_box_local = status
    flag_active_robot = True

def callback_robot_operation_command(message_):
    global command_to_execute
    command_to_execute = message_.data
    

def callback_reached(temp):
    global flag_active_robot, command_to_execute 
    # print(temp.status_list.__len__())
    length = temp.status_list.__len__()
    if temp.status_list[length-1].status == 3:
        if command_to_execute == 'pickup':
            if flag_active_robot == True:
                rospy.sleep(2)
                flag_active_robot = False
                now = rospy.get_rostime()
                print("The robot already reached: " + str(now.secs))

def ur_get_status():

    ##### This is for the Gazebo model
    # current_data = rospy.wait_for_message("arm_controller/state", JointTrajectoryControllerState)
    
    ##### This is for the real robot
    current_data = rospy.wait_for_message("joint_states", JointState)
        
    return current_data


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
    



    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    ur_status = ur_get_status()
    pose_current = ur_status.position    

    
    d = 2
    g.trajectory.points = [
            JointTrajectoryPoint(positions=pose_current, velocities=[0]*6, time_from_start=rospy.Duration(0))]


    for i in range(4):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2      
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2         
    
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def gripperInitial():
    
    # pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
    #command = outputMsg.SModel_robot_output();
     # Activation
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rICF = 0
    command.rFRA = 0 # This does not matter in Gazebo

    command.rMOD = 1
    command.rPRA = 0
    
    pub.publish(command)
    print("dddd")


def grip():
   
    #pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
    #command = outputMsg.SModel_robot_output();
    command.rMOD = 1
    command.rPRA = 110
    pub.publish(command)
    print ("ccc")

def release():
    
    #pub = rospy.Publisher('SModelRobotOutput',outputMsg.SModel_robot_output)
    #command = outputMsg.SModel_robot_output();
    command.rMOD = 1
    command.rPRA = 0
    pub.publish(command)

def move_to_basket(myArg1):
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # Get Current Status
    model_flag = myArg1
    ur_status = copy.deepcopy(ur_status_local)
    ee_frame, pose_current, pose_current_xyz = get_ee_frame(model_flag, ur_status)

    # Initialisation of trajectory
    g.trajectory.points = [JointTrajectoryPoint(positions=pose_current, velocities=[0]*6, time_from_start=rospy.Duration(0))]


    # Set Desired position
    pose_desired_xyz = copy.deepcopy(pose_current_xyz) # Initialisation
    
    pose_desired_xyz[0,3] = 0.477500049937
    pose_desired_xyz[1,3] = -0.341893124039
    pose_desired_xyz[2,3] = 0.20567763361


     # Compute the target joint space position 
    weights_joints_kin = [10, 10, 10, 1, 1, 1]
    q_guess =pose_current
    pose_desired = kin.inverse(pose_desired_xyz,q_guess,weights=weights_joints_kin)

     # Send the goal to the robot
    if(pose_desired is None):
        current_time = time.clock()
        print("[!!!Inverse Kinematics Error!!!]: Please move the robot a different direction " + str(current_time))
                    
        while not rospy.is_shutdown():
             print("Stop for safety reason!!!: IK Issue \n")

    else:
        now = rospy.get_rostime()
        print("Command Received at " + str(now.secs))

        duration = 4

                        

        g.trajectory.points.append(JointTrajectoryPoint(positions=pose_desired, velocities=[0]*6, time_from_start=rospy.Duration(duration)))

        client.send_goal(g)
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        
        if myArg1 == 'real':
            release() 



def move_initial(myArg1):
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES



    # Get Current Status
    model_flag = myArg1
    ur_status = copy.deepcopy(ur_status_local)
    ee_frame, pose_current, pose_current_xyz = get_ee_frame(model_flag, ur_status) 

    
    d = 3
    g.trajectory.points = [
            JointTrajectoryPoint(positions=pose_current, velocities=[0]*6, time_from_start=rospy.Duration(0))]
    
    g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(d)))

    
    client.send_goal(g)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

    if myArg1 == 'real':
        gripperInitial() 



def move_by_pointing(myArg1):

    global kin, ur_status_local, position_box_local, flag_active_robot, command_to_execute    

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

#    ur_status = ur_get_status()
#    pose_current = ur_status.position    

    model_flag = myArg1


 
    while not rospy.is_shutdown(): 

        if command_to_execute == 'pickup':

            if flag_active_robot is True:
            
                # Get Current Status
                ur_status = copy.deepcopy(ur_status_local)
                ee_frame, pose_current, pose_current_xyz = get_ee_frame(model_flag, ur_status) 

                # Initialisation of trajectory
                g.trajectory.points = [
                                JointTrajectoryPoint(positions=pose_current, velocities=[0]*6, time_from_start=rospy.Duration(0))]


                # Set Desired position
                pose_desired_xyz = copy.deepcopy(pose_current_xyz) # Initialisation
                position_box = copy.deepcopy(position_box_local)

                if position_box is not None:
                    desired_xyz = P1
                    desired_xyz[0] = position_box.linear.x # Set the goal
                    desired_xyz[1] = position_box.linear.y # Set the goal
                    desired_xyz[2] = position_box.linear.z # Set the goal
                
                    # print("Received value=" + str(position_box))
                    # print("Box position = " + str(desired_xyz))
        
                    pose_desired_xyz[0,3] = desired_xyz[0]
                    pose_desired_xyz[1,3] = desired_xyz[1]
                    pose_desired_xyz[2,3] = desired_xyz[2]

                    #pose_desired_xyz[0,3] = pose_desired_xyz[0,3]
                    #pose_desired_xyz[1,3] = pose_desired_xyz[1,3]
                    #pose_desired_xyz[2,3] =  pose_desired_xyz[2,3]-7

                    # Compute the target joint space position 
                    weights_joints_kin = [10, 10, 10, 1, 1, 1]
                    q_guess =pose_current
                    pose_desired = kin.inverse(pose_desired_xyz,q_guess,weights=weights_joints_kin)

                    # Send the goal to the robot
                    if(pose_desired is None):
                        current_time = time.clock()
                        print("[!!!Inverse Kinematics Error!!!]: Please move the robot a different direction " + str(current_time))
                        
                        while not rospy.is_shutdown():
                                print("Stop for safety reason!!!: IK Issue \n")
                        
                    else:
                        now = rospy.get_rostime()
                        print("Command Received at " + str(now.secs))

                        duration = 4

                            

                        g.trajectory.points.append(
                                JointTrajectoryPoint(positions=pose_desired, velocities=[0]*6, time_from_start=rospy.Duration(duration)))
                            
                        
                        client.send_goal(g)
                        try:
                            client.wait_for_result()
                        except KeyboardInterrupt:
                            client.cancel_goal()
                            raise
                        

                        if myArg1 == 'real':
                            grip() 
                        
                        command_to_execute = None   
                        
                        # rospy.sleep(duration + 1) 
                        # 
                        rospy.sleep(2)
                        # move to basket

                        
                    
            else:
                rospy.sleep(1)   
        
        if command_to_execute == 'drop':
            move_to_basket (myArg1)
            command_to_execute = None
                


def main(myArg1):
    global client, pub, command

    if (myArg1 == 'gazebo') or (myArg1 == 'real'):    
        try:
            rospy.init_node("test_move", anonymous=True, disable_signals=True)
            rospy.Subscriber("/temp/pos_virtual_box/",Twist, callback_virtualbox, queue_size=1) 

            rospy.Subscriber("/follow_joint_trajectory/status",GoalStatusArray, callback_reached, queue_size=1) 
            
            rospy.Subscriber("/test_pub",String,callback_robot_operation_command,queue_size=1)
            
            if myArg1 == 'gazebo':
                # For Gazebo
                rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState, callback_UR, queue_size=1) 

                client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

                
            elif myArg1 == 'real':     
                
                pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output) 
                command = outputMsg.SModel_robot_output();                           
                # Subscribing information from UR5
                rospy.Subscriber("/joint_states",JointState, callback_UR, queue_size=1)

                client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            
            
            print "Waiting for server..."
            client.wait_for_server()
            print "Connected to server"
            #move1()
            #move_repeated()
            move_initial(myArg1)
            
           
            move_by_pointing(myArg1)

                
            #move_disordered()
            #move_interrupt()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'gazebo' or 'real'")

if __name__ == '__main__': 

    
    # # # # This is for real
     if len(sys.argv) < 2:
         print("Usage: ur5_teleop_leap_vel_mode1.py [gazebo] or [real]")
     else:
         main(sys.argv[1])
        
    