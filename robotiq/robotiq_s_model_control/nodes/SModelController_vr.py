#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a S-Model gripper.

This serves as an example for publishing messages on the 'SModelRobotOutput' topic using the 'SModel_robot_output' msg type for sending commands to a S-Model gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from leap_motion.msg import Human_orion  as LeapMsg_orion # Newly defined LEAP ROS msg
from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity

import sys
import copy
from time import sleep
import numpy as np
###############################################################
###############################################################
value_starting_to_close = 0.1 # Unit:Grab_strength (0-1)
value_position_tolerance = 20 # Unit:0-255. If gPOA - rPRA is less than this value, then the gripper does not move
value_for_max_speed = 125 # Unit:0-255. If gPOA - rPRA is more than this value, then the gripper moves in the fastest manner
###############################################################
###############################################################
###############################################################
###############################################################

global teleoperation_mode, LeapMsg_local, GripperMsg_local, command, pub
teleoperation_mode = ""
LeapMsg_local = None
GripperMsg_local = None

def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def callback_Leap(status):
    global LeapMsg_local
    LeapMsg_local = status

def callback_Gripper(status):
    global GripperMsg_local
    GripperMsg_local = status

def control_gripper():

    global pub
    global teleoperation_mode, LeapMsg_local, GripperMsg_local
    global command

    publish_rate = 250
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():

        if LeapMsg_local is not None and GripperMsg_local is not None and teleoperation_mode == "MODE_1":
            # Get Current Status
            leap_status = copy.deepcopy(LeapMsg_local)

            if (leap_status.left_hand.is_present is True):
                # Control the Gripper's position
                grab_strength_from_unity = leap_status.left_hand.grab_strength
                if (grab_strength_from_unity <= 1 and grab_strength_from_unity > value_starting_to_close): # Fully Close
                    # grab_strength = 255 # This is for the on-off controller
                    grab_strength = round(255*(grab_strength_from_unity-value_starting_to_close)*1.0/(1.0-value_starting_to_close),0) # Give an integer
                else: # Fully Open
                    grab_strength = 0 # Fully Open
                    
                # Control the Gripper's speed
                current_position = copy.deepcopy(GripperMsg_local.gPOA)
                grap_speed_ = np.abs(grab_strength - current_position)
   
                # Additional treatment
                command.rGTO = 1  
                if grap_speed_ < value_position_tolerance:            
                    grab_strength = current_position
                    grap_speed = 0
                    command.rGTO = 0 # Stops 
                    
                elif grap_speed_ > value_for_max_speed:
                    grap_speed = 255
                else:
                    grap_speed = round((grap_speed_ - value_position_tolerance)*1.0/(value_for_max_speed-value_position_tolerance)*255,0)

                print("Here Grapstrength: " + str(grab_strength) + " Speed : " + str(grap_speed) + "\n")
                command.rPRA = grab_strength
                command.rSPA = grap_speed


        pub.publish(command)
        rate.sleep()
        



def main(myArg1):

    global pub
    global command
    
    if (myArg1 == 'gazebo') or (myArg1 == 'real'):
        try:
            rospy.init_node('Controller_Gripper', anonymous=True,  disable_signals=True)
    
            if myArg1 == 'gazebo':
                ## This is for the gazebo Gripper:
                pub = rospy.Publisher('right_hand/command', outputMsg.SModel_robot_output, queue_size = 1)

            elif myArg1 == 'real':
                ## This is for the real Gripper:
                pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size = 1)
  



            # Initilise the gripper
            command = outputMsg.SModel_robot_output()
    
            command.rACT = 1 # Activation
            command.rGTO = 1 # 
            command.rSPA = 255 # Opening/Closing Speed (Maximum = 255)
            command.rICF = 0 # Individual Control Finger mode (1: Yes / 0: No)
            command.rFRA = 0 # Final grasping force (Maximum = 255). This does not matter in Gazebo.

            command.rPRA = 0 # Fully Open



            pub.publish(command)
            rospy.sleep(1)


            # Subscribing information from LEAP in Unity
            rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)    
            rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg_orion, callback_Leap, queue_size=1)

            rospy.Subscriber("/SModelRobotInput",inputMsg.SModel_robot_input, callback_Gripper, queue_size=1)    
            control_gripper()

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise            
    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'gazebo' or 'real'")



if __name__ == '__main__': 
    # # This is for real
    # if len(sys.argv) < 2:
    #     print("Usage: SModelController.py [gazebo] or [real]")
    # else:
    #     main(sys.argv[1])
        
    # This is for debug    
    main('real')
