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

from sensor_msgs.msg import JointState # To receive the current state of UR
from leap_motion.msg import Human  as LeapMsg # To receive the info from LEAP


reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

# For forward/inverse kinematics
from ur_kinematics import Kinematics
import math


global palm_centre_pre, palm_centre, grab_strength, g, time_pre
global joint_speed_local
joint_speed_local = None
time_pre = None
palm_centre_pre = None
grab_strength = 0

# For Debugging: Should be deleted when committed
reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0,0,0,0,0,0]
Q2 = [3.14,0,0,0,0,0]
Q3 = [3.14,1.5,0,0,0,0]
Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

client = None

pi = math.pi

def callback_UR(status):
    global joint_state
    global pub

    joint_state.header = status.header
    joint_state.position = status.actual.positions
    joint_state.velocity = status.actual.velocities
    pub.publish(joint_state)
            

def cb_joint_speed(status):
    global joint_speed_local
    joint_speed_local = status

def control_gazebo_ur():
    global g
    global client
    global pose
    global time_pre
    global joint_speed_local
    
    time_to_reach = 0.1

    num_stack = 2 # This is to get time_pre
    time_info_stack = [0]*num_stack 


    while not rospy.is_shutdown():

       
        if joint_speed_local is not None:

            joint_speed_now = joint_speed_local
            time_now = joint_speed_now.header.stamp.secs + joint_speed_now.header.stamp.nsecs/(1000000000.0)

            time_info_stack.pop(0) 
            time_info_stack.append(time_now)
            time_pre = np.amin(time_info_stack)
            time_now = np.amax(time_info_stack) 

            if (time_now - time_pre) > 0 and (time_now - time_pre) < 1:

                

                desired_joint_speed = joint_speed_now.points[0].velocities   

                time_to_reach = time_now - time_pre
                del_pose = np.dot(time_to_reach,desired_joint_speed)
                pose = np.add(pose,del_pose)
                    

                g.trajectory.points = [
                    JointTrajectoryPoint(positions=pose.tolist(), effort=[10]*6, time_from_start=rospy.Duration(0.04))]
                client.send_goal(g)
                try:
                    client.wait_for_result()
                except KeyboardInterrupt:
                    client.cancel_goal()
                    raise
            elif (time_now - time_pre) > 1:
                print("Far-away sequential signals at " + str(time_now) + " \n")
        else:
            print("No Signal at " + str(rospy.get_rostime()) + "\n")



  




def joint_speed_to_actioncall():
    global pub, joint_state

    teleop_leap_initialisation()
 
    pub = rospy.Publisher('/joint_states_gazebo',JointState,queue_size=1)
    joint_state = JointState()
    joint_state.name = JOINT_NAMES 

    rospy.Subscriber("/ur_driver/joint_speed",JointTrajectory, cb_joint_speed, queue_size=1)
    rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState, callback_UR, queue_size=1)
    control_gazebo_ur()
   


def teleop_leap_initialisation(): # UR5 control via LEAP
    global g
    global pose
    global time_pre    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # pose = ur_status.actual.positions

    # Initilise UR5 position (to the predefined position, Q1)
    pose = Q1
    g.trajectory.points = [
        JointTrajectoryPoint(positions=pose, velocities=[0]*6, time_from_start=rospy.Duration(0.1))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise    

    time.sleep(2)

    # Then to the predefined position, Q0)
    pose = Q0
    g.trajectory.points = [
        JointTrajectoryPoint(positions=pose, velocities=[0]*6, time_from_start=rospy.Duration(0.04))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise  

    




def main():
    global client
 
    try:
        rospy.init_node("joint_speed_to_GazeboUR", anonymous=True, disable_signals=True)        
        # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       

        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"

        joint_speed_to_actioncall()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
