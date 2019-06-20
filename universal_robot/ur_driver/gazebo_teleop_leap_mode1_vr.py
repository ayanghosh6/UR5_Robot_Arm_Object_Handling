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

from sensor_msgs.msg import JointState # To receive the current state
# from leap_motion.msg import Human  as LeapMsg # Existing LEAP ROS msg , i.e, "Human.msg")
from leap_motion.msg import Human_orion  as LeapMsg_orion # Newly defined LEAP ROS msg
from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity


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

client = None


# For Teleoperation by Keyboard
msg2 = """
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Reading from the keyboard  and Publishing to move UR5!
---------------------------
Pose Command : 0, 1, 2
---------------------------
Orientation Command : 3, 4, 5
---------------------------
anything else : stop or CTRL-C to quit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
=> """


del_q = 0.1
del_x = 0.01
pi = math.pi
del_ori = pi/50


nor_vecBindings = {
        '3':np.array([1, 0, 0]), # Rightward Left Hand (Base Frame Point of View)
        '4':np.array([0, 1, 0]), # Forward(Up) Left Hand 
        '5':np.array([0, 0, -1]), # Down Left Hand 
            }

dir_vecBindings = {
        '3':np.array([0, 1, 0]), # Rightward Left Hand  
        '4':np.array([0, 0, 1]), # Forward(Up) Left Hand  
        '5':np.array([0, 1, 0]),  # Down Left Hand 
            }

poseBindings={
        '0':np.array(Q0),
		'1':(0,0,0,0,0,0),
        '2':(3.14,0,0,0,0,0),
	      }
       

def getKey():
	tty.setraw(sys.stdin.fileno()) # Inmo: sys.stdin.fileno() is an integer representing open file; ttw.setraw() puts the terminal into a raw mode 
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1) 
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def ur_get_status():

    current_data = rospy.wait_for_message("arm_controller/state", JointTrajectoryControllerState)
    return current_data

def Leap_callback_MODE1(status):

    global palm_centre_pre, palm_normal_from_unity_pre, palm_direction_from_unity_pre
    
    global g
    global client
    global pose
    del_tolerance = 0.00
    del_upper_limit = 0.20
    scale_factor_movement = 0.5
    # pose = ur_status.actual.positions

    global teleoperation_mode
    if(teleoperation_mode == "MODE_1"):

        if (status.left_hand.is_present):
            palm_centre = status.left_hand.palm_center
            # Info From Unity via ROS# (Data type is a ROS msg)
            palm_normal_from_unity = status.left_hand.palm_normal
            palm_direction_from_unity = status.left_hand.palm_direction

            if  (palm_centre_pre is not None):
                ###### Position Control
                del_x = (palm_centre.x - palm_centre_pre.x)
                del_z = palm_centre.y - palm_centre_pre.y
                del_y = (palm_centre.z - palm_centre_pre.z)

                if (abs(del_x) < del_tolerance or abs(del_x) > del_upper_limit):
                    del_x = 0
                else: 
                    del_x = del_x*scale_factor_movement

                if (abs(del_y) < del_tolerance or abs(del_y) > del_upper_limit):
                    del_y = 0
                else:
                    del_y = del_y*scale_factor_movement
                
                if (abs(del_z) < del_tolerance or abs(del_z) > del_upper_limit):
                    del_z = 0
                else: 
                    del_z = del_z*scale_factor_movement

                del_xyz = np.matrix([[0,0,0,del_x],[0,0,0,del_y],[0,0,0,del_z],[0,0,0,0]])

                ###### Orientation Control                
                # Coordination Transformation to the ROS world (y and z should be replaced with each other)
                palm_normal = np.array([palm_normal_from_unity.x, palm_normal_from_unity.z, palm_normal_from_unity.y])
                palm_direction = np.array([palm_direction_from_unity.x, palm_direction_from_unity.z, palm_direction_from_unity.y])
                

                kin = Kinematics('ur5')


                q_guess = pose               # Use current joint angles as a guess value for IK
                pose_xyz = kin.forward(pose) # Convert the current joints status to xyz
                nor_vec_ = palm_normal
                dir_vec_ = palm_direction
                dir_cross_nor_vec_ = np.cross(dir_vec_,nor_vec_)
                dir_vec_ = np.cross(nor_vec_,dir_cross_nor_vec_) # Directional vector needs to be corrected as perpendicular. 
                pose_xyz[0:3,0] = np.transpose(np.asmatrix(nor_vec_)) # X_ee = Palm Normal Vector Orientation (Base framepoint of view)
                y_ee = (-dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2
                pose_xyz[0:3,1] = np.transpose(np.asmatrix(y_ee)) # Y_ee 
                z_ee = (dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2
                pose_xyz[0:3,2] = np.transpose(np.asmatrix(z_ee)) # Z_ee 
                
                pose_xyz = pose_xyz = np.add(pose_xyz, del_xyz) 
                weights_joints_kin = [1, 1, 1, 1, 1, 1]
                pose_ = kin.inverse(pose_xyz,q_guess,weights=weights_joints_kin) # Reconvert the desired xyz onto the joint space  
                
                if(pose_ is None):
                    current_time = time.clock()
                    # ur_status = ur_get_status()
                    # pose = ur_status.actual.positions
                    print("[!!!Inverse Kinematics Error!!!]: Please move the robot in a different direction " + str(current_time))
                
                else:
                    pose = pose_
                    g.trajectory.points = [
                        JointTrajectoryPoint(positions=pose, effort=[100]*6, time_from_start=rospy.Duration(0.01))]
                    client.send_goal(g)
                    current_time = time.clock()
                    print("Palm Normal = "+ str(palm_normal) + " Direction = " + str(palm_direction) + "At " + str(current_time))
                    try:
                        client.wait_for_result()
                    except KeyboardInterrupt:
                        client.cancel_goal()
                        raise

            # Save previous info    
            palm_centre_pre = palm_centre
            palm_normal_from_unity_pre = palm_normal_from_unity 
            palm_direction_from_unity_pre = palm_direction_from_unity

        else:
            print "MODE1: Position your left hand on the sensor"

    else:
        # Get Current UR Status (otherwise, when the control mode is switched, robot movement is bumpy)
        ur_status = ur_get_status()
        pose = ur_status.actual.positions


def callback_mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def listener_from_leap():

    teleop_leap_initialisation()

    # Initialise "teleoperation_mode", otherwise callback error comes out from "callback_Gripper"
    global teleoperation_mode
    teleoperation_mode = ""

    # Subscribing information from LEAP
    rospy.Subscriber("/rain/status",RainMsg, callback_mode, queue_size=1)    
    rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg_orion, Leap_callback_MODE1, queue_size=1)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def teleop_leap_initialisation(): # UR5 control via LEAP
    global g
    global pose
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    global palm_centre_pre
    palm_centre_pre = None
    # pose = ur_status.actual.positions

    # Initilise UR5 position (to the predefined position, Q0)
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
        rospy.init_node("endeffector_controller_by_leap", anonymous=True, disable_signals=True)        
        # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       

        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        #move1()
        # move_repeated()
        #move_disordered()
        #move_interrupt()
        # teleop_key()
        listener_from_leap()
        # teleop_key_vec()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin)
    main()
