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
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

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

# 14(X) for close, 13(O) for open
def joy_call(input):
    if input.buttons[14] == 1:
        rospy.loginfo('Closing!')
        msg.rPRA = 127
        finger_pub.publish(msg)
	if input.buttons[13] == 1:
        rospy.loginfo('Closing!')
        msg.rPRA = 0

if __name__ == '__main__':
    try:

        arm_pub = rospy.Publisher('/husky_ur5Arm', Pose, queue_size = 1)
        msgArm = Pose()

        finger_pub = rospy.Publisher('/SModelRobotOutput', SModel_robot_output, queue_size=1)  

        rospy.Subscriber("/joy", Joy, joy_call)
        rospy.init_node('ridgeback_ur5')

        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():         
            finger_pub.publish(msg)
            rate.sleep()

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
