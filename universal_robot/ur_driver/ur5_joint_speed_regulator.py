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

reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

global joint_speed_local
joint_speed_local = None


# For Debugging: Should be deleted when committed
reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



def cb_joint_speed(status):
    global joint_speed_local
    joint_speed_local = status

def joint_speed_regulator(pub):
    global joint_speed_local
    
    publish_rate = 250
    rate = rospy.Rate(publish_rate)

    g = JointTrajectory()
    g.joint_names = JOINT_NAMES

    while joint_speed_local is None:
        print("Waiting joint_speed_ signal \n")
        rate.sleep()
    print("Connected \n")

    while not rospy.is_shutdown(): 

        g.header.stamp = rospy.get_rostime() 
        g.points = joint_speed_local.points
            
        pub.publish(g)
           
        rate.sleep()


def main():
 
    try:
        rospy.init_node("joint_speed_regulator", anonymous=True, disable_signals=True)        
        pub = rospy.Publisher('/ur_driver/joint_speed',JointTrajectory, queue_size=1)       

        # Subscribing information from Unity
        rospy.Subscriber('/ur_driver/joint_speed_',JointTrajectory, cb_joint_speed, queue_size=1)  
        joint_speed_regulator(pub)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    main()
