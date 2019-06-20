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
Q1 = [0,0,0,0,0,math.pi/3]
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
Orientation Command : 3, 4, 5, 6
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
        '6':np.array([math.sqrt(2)/2, -math.sqrt(2)/2, 0]),
            }

dir_vecBindings = {
        '3':np.array([0, 1, 0]), # Rightward Left Hand  
        '4':np.array([0, 0, 1]), # Forward(Up) Left Hand  
        '5':np.array([0, 1, 0]),  # Down Left Hand 
        '6':np.array([0, 0, 1]), # Forward(Up) Left Hand  
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

def teleop_key_vec():
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # pose = ur_status.actual.positions

    # Initilise UR5 position
    pose = Q1
    g.trajectory.points = [
        JointTrajectoryPoint(positions=pose, velocities=[0]*6, time_from_start=rospy.Duration(0.04))]
    client.send_goal(g)

    kin = Kinematics('ur5')

    run_flag = 1
    while(run_flag):

        pose_valid_flag = 0 # user input position validity (initialisation) 
        while(pose_valid_flag == 0):
            print(msg2)    
            key = getKey()
            print("=> Key input = " + str(key) + "\n")
            if key in nor_vecBindings.keys():
                q_guess = pose               # Use current joint angles as a guess value for IK
                pose_xyz = kin.forward(pose) # Convert the current joints status to xyz

                nor_vec_ = nor_vecBindings[key]
                dir_vec_ = dir_vecBindings[key]
                dir_cross_nor_vec_ = np.cross(dir_vec_,nor_vec_)

                pose_xyz[0:3,0] = np.transpose(np.asmatrix(nor_vec_)) # X_ee = Palm Normal Vector Orientation (Base frame point of view)

                y_ee = (-dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2
                pose_xyz[0:3,1] = np.transpose(np.asmatrix(y_ee)) # Y_ee 

                z_ee = (dir_vec_ + dir_cross_nor_vec_)*math.sqrt(2)/2
                pose_xyz[0:3,2] = np.transpose(np.asmatrix(z_ee)) # Z_ee 
                
                position = np.array([0.5,0,0.5]) # Temporary position (will be delted)
                pose_xyz[0:3,3] = np.transpose(np.asmatrix(position))

                pose = kin.inverse(pose_xyz,q_guess) # Reconvert the desired xyz onto the joint space  


            elif key in poseBindings.keys():
                pose = poseBindings[key]
          
            else:
                pose = pose            
       

            
            if pose is not None:
                pose_valid_flag = 1
            else:
                ur_status = ur_get_status()
                pose = ur_status.actual.positions
                print("[!!!Inverse Kinematics Error!!!]: Please move the robot in a different direction")

        if (key == '\x03'):
            break

        g.trajectory.points = [
            JointTrajectoryPoint(positions=pose, effort=[100]*6, time_from_start=rospy.Duration(0.001))]
        client.send_goal(g)
        print(pose)
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise


def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)        
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
        teleop_key_vec()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin)
    main()
