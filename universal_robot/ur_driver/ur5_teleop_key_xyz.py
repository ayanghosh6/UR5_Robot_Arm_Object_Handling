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
Q1 = [0,0,0,0,0,0]
Q2 = [3.14,0,0,0,0,0]
Q3 = [3.14,1.5,0,0,0,0]
Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 0.001
    g.trajectory.points = []
    for i in range(2):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1      
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

# For Teleoperation by Keyboard
msg = """
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Reading from the keyboard  and Publishing to move UR5!
---------------------------
Joint Control: J1 J2 J3 J4 J5 J6
     UP      : q  w  e  r  t  y
     DOWN    : a  s  d  f  g  h     
    (NB: Incremental joint angle = 0.1 (rad))
---------------------------
Pose Command : 1, 2
---------------------------
anything else : stop or CTRL-C to quit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++"""

msg2 = """
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Reading from the keyboard  and Publishing to move UR5!
---------------------------
Position Control:      (NB: Increment = 0.01 (m)) 
    - Forward/Backware/Left/Right (x-y-axis): u / j / h / k
    - Up/Down (z-axis): t / g
Orientation Control:   (NB: Increment = pi/50 (rad))
    - Pitch Up/Down  : w / s
    - Roll Left/Right: a / d
    - Yaw Left/Right : q / e
---------------------------
Pose Command : 1, 2
---------------------------
anything else : stop or CTRL-C to quit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
=> """

pi = math.pi

###############################################################
###############################################################
# Movement Control
time_to_reach = 0.008 # desired time to finish each movement
del_x = 0.01 # position movement increment (meter)
velocity_test = 0.5
del_ori = pi/50
###############################################################
###############################################################
###############################################################
###############################################################



del_q = 0.1

moveBindings = {
		'q':(del_q,0,0,0,0,0),
		'a':(-del_q,0,0,0,0,0),
		'w':(0,del_q,0,0,0,0),
		's':(0,-del_q,0,0,0,0),
		'e':(0,0,del_q,0,0,0),
		'd':(0,0,-del_q,0,0,0),
		'r':(0,0,0,del_q,0,0),
		'f':(0,0,0,-del_q,0,0),
		't':(0,0,0,0,del_q,0),
		'g':(0,0,0,0,-del_q,0),
		'y':(0,0,0,0,0,del_q),
		'h':(0,0,0,0,0,-del_q),
		  }

moveBindings2 = {		
        'u':np.matrix([[0,0,0,del_x],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),
        'j':np.matrix([[0,0,0,-del_x],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),
        'h':np.matrix([[0,0,0,0],[0,0,0,del_x],[0,0,0,0],[0,0,0,0]]),
        'k':np.matrix([[0,0,0,0],[0,0,0,-del_x],[0,0,0,0],[0,0,0,0]]),
        't':np.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,del_x],[0,0,0,0]]),
        'g':np.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,-del_x],[0,0,0,0]]),      
		  }

oriBindings = {
        'w':np.matrix([[math.cos(del_ori),0,math.sin(-del_ori)],[0,1,0],[math.sin(del_ori),0,math.cos(del_ori)]]), # Pitch control: Up
        's':np.matrix([[math.cos(del_ori),0,math.sin(del_ori)],[0,1,0],[math.sin(-del_ori),0,math.cos(del_ori)]]), # Pitch control: Down
        'a':np.matrix([[1,0,0],[0,math.cos(del_ori),math.sin(del_ori)],[0,math.sin(-del_ori),math.cos(del_ori)]]), # Role control: Left
        'd':np.matrix([[1,0,0],[0,math.cos(del_ori),math.sin(-del_ori)],[0,math.sin(del_ori),math.cos(del_ori)]]), # Role control: Right
        'q':np.matrix([[math.cos(del_ori),math.sin(-del_ori),0],[math.sin(del_ori),math.cos(del_ori),0],[0,0,1]]), # Yaw control: Left
        'e':np.matrix([[math.cos(del_ori),math.sin(del_ori),0],[math.sin(-del_ori),math.cos(del_ori),0],[0,0,1]]), # Yaw control: Right
            }

poseBindings={
		'1':(0,0,0,0,0,0),
        '2':(3.14,0,0,0,0,0),
	      }

delBindings={
		'+':(0.01),
        '-':(-0.01),
	      }          

def getKey():
	tty.setraw(sys.stdin.fileno()) # Inmo: sys.stdin.fileno() is an integer representing open file; ttw.setraw() puts the terminal into a raw mode 
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1) 
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def teleop_key():
    del_q = rospy.get_param("/del_q", 0.5) # Initial Joint Angle Increment

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    ur_status = ur_get_status()
    pose = ur_status.actual.positions
    
        
    while(1):
        print(msg)    
        key = getKey()
        print("=> Key input = " + str(key) + "\n")
        if key in moveBindings.keys():
            pose = np.add(pose, moveBindings[key])

        # elif key in poseBindings.keys():
            # pose = poseBindings[key]

        elif key in delBindings.keys():
            del_q = np.add(del_q,delBindings[key])
            print("(Not work yet) Current Incremental Joint Angle = %s" % (del_q))
			# speed = speed * speedBindings[key][0]
			# turn = turn * speedBindings[key][1]

			# print(vels(speed,turn))
			# if (status == 14):
			# 	print(msg)
			# status = (status + 1) % 15
            #             
        else:
			pose = Q1            
			if (key == '\x03'):
				break        


        g.trajectory.points = [
            JointTrajectoryPoint(positions=pose, velocities=[0]*6, time_from_start=rospy.Duration(0.04))]
        client.send_goal(g)
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise

def ur_get_status():

    ##### This is for the Gazebo model
    # current_data = rospy.wait_for_message("arm_controller/state", JointTrajectoryControllerState)
    
    ##### This is for the real robot
    current_data = rospy.wait_for_message("joint_states", JointState)
        
    return current_data    

def teleop_key_xyz():
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES



    kin = Kinematics('ur5')

    run_flag = 1
    while(run_flag):

        pose_valid_flag = 0 # user input position validity (initialisation) 
        while(pose_valid_flag == 0):
            print(msg2)    
            key = getKey()
            print("=> Key input = " + str(key) + "\n")
            ur_status = ur_get_status()
            pose_current = ur_status.position       
            if key in moveBindings2.keys():
                q_guess = pose_current               # Use current joint angles as a guess value for IK
                pose_current_xyz = kin.forward(pose_current) # Convert the current joints status to xyz

                pose_xyz = np.add(pose_current_xyz, moveBindings2[key])
                pose = kin.inverse(pose_xyz,q_guess) # Reconvert the desired xyz onto the joint space  

            elif key in oriBindings.keys():
                q_guess = pose_current               # Use current joint angles as a guess value for IK
                pose_current_xyz = kin.forward(pose_current) # Convert the current joints status to xyz
                ori_current = pose_current_xyz[0:3,0:3] # Current orientation
                trans_matrix = oriBindings[key]
                ori_new = np.matmul(trans_matrix,ori_current)
                pose_current_xyz[0:3,0:3] = ori_new
                pose = kin.inverse(pose_current_xyz, q_guess) # Reconvert the desired xyz onto the joint space  

            elif key in poseBindings.keys():
                pose = poseBindings[key]

            elif key in delBindings.keys():
                del_q = np.add(del_q,delBindings[key])
                print("(Not work yet) Current Incremental Joint Angle = %s" % (del_q))
                # speed = speed * speedBindings[key][0]
                # turn = turn * speedBindings[key][1]

                # print(vels(speed,turn))
                # if (status == 14):
                # 	print(msg)
                # status = (status + 1) % 15
                #             
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
            JointTrajectoryPoint(positions=pose_current, velocities=[0]*6, time_from_start=rospy.Duration(0)),
            JointTrajectoryPoint(positions=pose, velocities=[velocity_test]*6, time_from_start=rospy.Duration(time_to_reach))]
        client.send_goal(g)

        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise


def main():
    global client
    try:
        rospy.init_node("ur5_teleop_key_xyz", disable_signals=True)        
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        # client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       

        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        #move1()
        # move_repeated()
        #move_disordered()
        #move_interrupt()
        # teleop_key()
        teleop_key_xyz()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin)
    main()
