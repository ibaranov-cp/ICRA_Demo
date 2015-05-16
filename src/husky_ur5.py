#! /usr/bin/env python
import roslib;
import rospy
import time
import math

import sys
import numpy as np

import actionlib
import tf
from robotiq_s_model_control.msg import *
#import jaco_msgs.msg
#from jaco_msgs.srv import *
import std_msgs.msg
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

#We want position: spineshoulder (20) - wristRight (10)
# Scalling factor
# orientation: wristRight (10) and handRight (11) 
spine = [0.0,0.0,0.0]
wristRight = [0.0,0.0,0.0]
handRight = [0.0,0.0,0.0]
gripState = 0
scale = 1.5

def kinect_x_call(array):
    #rospy.loginfo(array.data[20])
    global spine
    global wristRight
    global handRight
    global gripState
    spine[0] = array.data[20]
    wristRight[0] = array.data[10]
    handRight[0] = array.data[11]
    
    if array.data[26] >= 3.0:
        gripState += 1
    else:
        gripState -= 1
    
    if gripState <= 0:
        gripState = 0
    if gripState >= 15:
        gripState = 15


def kinect_y_call(array):
    #rospy.loginfo(array.data[20])
    global spine
    global wristRight
    global handRight
    spine[1] = array.data[20]
    wristRight[1] = array.data[10]
    handRight[1] = array.data[11]


def kinect_z_call(array):
    #rospy.loginfo(array.data[20])
    global spine
    global wristRight
    global handRight
    spine[2] = array.data[20]
    wristRight[2] = array.data[10]
    handRight[2] = array.data[11]


def joy_call(input):
    if input.buttons[0] == 1:
        rospy.loginfo(input.buttons)

if __name__ == '__main__':
    try:

        arm_pub = rospy.Publisher('/husky_ur5Arm', Pose, queue_size = 1)
        msgArm = Pose()

        finger_pub = rospy.Publisher('/SModelRobotOutput', SModel_robot_output, queue_size=1)
        msg = SModel_robot_output()
        msg.rACT =1	
        msg.rMOD =0	
        msg.rGTO =1	
        msg.rATR =0	
        msg.rGLV =0	
        msg.rICF =0	
        msg.rICS =0	
        msg.rPRA =127
        msg.rSPA =255
        msg.rFRA =0	
        msg.rPRB =155
        msg.rSPB =0	
        msg.rFRB =0	
        msg.rPRC =255
        msg.rSPC =0	
        msg.rFRC =0	
        msg.rPRS =0	
        msg.rSPS =0	
        msg.rFRS =0	    

        rospy.Subscriber("Position_X", Float32MultiArray, kinect_x_call)
        rospy.Subscriber("Position_Y", Float32MultiArray, kinect_y_call)
        rospy.Subscriber("Position_Z", Float32MultiArray, kinect_z_call)
        rospy.Subscriber("/joy_teleop/joy", Joy, joy_call)
        rospy.init_node('husky_ur5')

        rate = rospy.Rate(10) # 10hz

        #gripper_client([200.0,200.0]) 
        grip = 0       
        offset = 0.6

        while not rospy.is_shutdown():

	    #if homing == 1:
	    #   home()
 
            #Calc position
            diff = [-scale*(wristRight[0] - spine[0])+offset,scale*(wristRight[1] - spine[1]),scale*(wristRight[2] - spine[2])]

            #Calc orientation
            V_D = [handRight[0] - wristRight[0],handRight[1] - wristRight[1],handRight[2]-wristRight[2]]
            rpy = [0,math.asin(V_D[2]), math.atan2(V_D[2],V_D[1])]

            quat = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            quat = tf.transformations.quaternion_from_euler(1.571,0,0)

            msgArm.position.x = diff[0]
            msgArm.position.y = diff[1]
            msgArm.position.z = diff[2]
            msgArm.orientation.x = quat[0]
            msgArm.orientation.y = quat[1]
            msgArm.orientation.z = quat[2]
            msgArm.orientation.w = quat[3]
          
            arm_pub.publish(msgArm)          
            
            if (gripState >= 15) & (grip == 0):
                rospy.loginfo('Closing!')
                msg.rPRA = 127
                finger_pub.publish(msg)
                grip = 1
            if (gripState <= 0) & (grip == 1):
                rospy.loginfo('Opening!')       
                msg.rPRA = 0
                finger_pub.publish(msg)
                grip = 0

                            

            #rospy.loginfo(result)
            rate.sleep()


        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
