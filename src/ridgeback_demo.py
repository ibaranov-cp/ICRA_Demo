#! /usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import math
import sys
import numpy as np
import tf
from robotiq_s_model_control.msg import *
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [1.57,-1.57,0,-1.57,0,0]
Q2 = [-1.57,-1.57,0,-1.57,0,0]
Q3 = [-1.57,0,-2.79,-1.57,0,0]

client = None

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
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for arm server..."
        client.wait_for_server()
        print "Connected to arm server"

        finger_pub = rospy.Publisher('/SModelRobotOutput', SModel_robot_output, queue_size=1)  

        rospy.Subscriber("/joy", Joy, joy_call)
        rospy.init_node('ridgeback_ur5')

        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():         
            finger_pub.publish(msg)
            rate.sleep()
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = JOINT_NAMES
            g.trajectory.points = [
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
            client.send_goal(g)
            try:
                client.wait_for_result()
            except KeyboardInterrupt:
                client.cancel_goal()
                raise

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
