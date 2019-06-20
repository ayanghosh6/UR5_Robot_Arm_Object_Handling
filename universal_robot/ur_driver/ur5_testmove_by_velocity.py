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

publish_rate = 125


def move_joint1_on_off():
    global rate, pub

    g = JointTrajectory()
    g.joint_names = JOINT_NAMES
    
    vel_j1 = 0.6
    period = 1 # On Off chaning period
    while not rospy.is_shutdown():

        now = rospy.get_rostime()
        now_10 = now.secs / period
        
        if np.remainder(now_10,2) == 1:
            g.points = [
                JointTrajectoryPoint(velocities=[-vel_j1,0,0,0,0,0])]

        else:
            g.points = [
                JointTrajectoryPoint(velocities=[vel_j1,0,0,0,0,0])]
            
        g.header.stamp = now
        pub.publish(g)
        rate.sleep()




def move_joint1_sine():
    global rate, pub

    g = JointTrajectory()
    g.joint_names = JOINT_NAMES
    
    vel_j = 0.6
    period = 2
    end_time = 100
    time_j1 = np.linspace(0,end_time,publish_rate*end_time+1)
    vel_j1 = vel_j*np.sin(2*np.pi*time_j1/period)
    
    for i in range(len(time_j1)):

        g.points = [
                    JointTrajectoryPoint(velocities=[vel_j1[i],0,0,0,0,0])]

        now = rospy.get_rostime()
        g.header.stamp = now  
        pub.publish(g)
        rate.sleep()
    


def main():
    global rate, pub

    try:   
        rospy.init_node("ur5_teleop_key_vel", anonymous=True ,disable_signals=True)  
        pub = rospy.Publisher('/ur_driver/joint_speed',JointTrajectory,queue_size=1)
        rate = rospy.Rate(publish_rate)

        # move_joint1_on_off()
        move_joint1_sine()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
