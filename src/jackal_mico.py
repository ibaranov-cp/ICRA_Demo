#! /usr/bin/env python
import roslib; roslib.load_manifest('jaco_demo')
import rospy
import time
import math

import sys
import numpy as np

import actionlib
import tf
import jaco_msgs.msg
from jaco_msgs.srv import *
import std_msgs.msg
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg
from sensor_msgs.msg import Joy

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/mico_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    goal.fingers.finger3 = 0.0

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the gripper action timed-out')
        return None

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/mico_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id='mico_api_origin')
    goal.pose.pose.position = geometry_msgs.msg.Point(x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(0.50)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        #print('        the cartesian action timed-out')
        return None

#We want position: spineshoulder (20) - wristRight (10)
# Scalling factor
# orientation: wristRight (10) and handRight (11) 
spine = [0.0,0.0,0.0]
wristRight = [0.0,0.0,0.0]
handRight = [0.0,0.0,0.0]
gripState = 0
scale = 1.5
homing = 0
home = rospy.ServiceProxy('/mico_arm_driver/in/home_arm', HomeArm)

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
    if gripState >= 30:
        gripState = 30


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
    global homing
    #rospy.loginfo(input.buttons)
    if input.buttons[14] == 1:
        homing = 1
    else:
        homing = 0
    #rospy.loginfo(homing)

if __name__ == '__main__':
    try:


        rospy.Subscriber("Position_X", Float32MultiArray, kinect_x_call)
        rospy.Subscriber("Position_Y", Float32MultiArray, kinect_y_call)
        rospy.Subscriber("Position_Z", Float32MultiArray, kinect_z_call)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, joy_call)
        rospy.init_node('jackal_mico')

        rate = rospy.Rate(10) # 10hz
        rospy.wait_for_service('/mico_arm_driver/in/home_arm')
        home()

        gripper_client([200.0,200.0]) 
        grip = 0       
        offset = 0.6

        while not rospy.is_shutdown():

	    if homing == 1:
	       home()
 
            #Calc position
            diff = [-scale*(wristRight[0] - spine[0])+offset,scale*(wristRight[1] - spine[1]),scale*(wristRight[2] - spine[2])]

            #Calc orientation
            V_D = [handRight[0] - wristRight[0],handRight[1] - wristRight[1],handRight[2]-wristRight[2]]
            rpy = [0,math.asin(V_D[2]), math.atan2(V_D[2],V_D[1])]

            quat = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            quat = tf.transformations.quaternion_from_euler(1.571,0,0)
          
            #rospy.loginfo(tf.transformations.euler_from_quaternion([0.707,0,0,0.707]))
            #rospy.loginfo(rpy)
            #rospy.loginfo(quat)

            raw_pose = [diff[0], diff[2], diff[1], quat[0], quat[1],quat[2],quat[3]]
            mag = np.sqrt(sum(np.power(raw_pose[3:], 2)))
            poses = [(raw_pose[:3], raw_pose[3:])]
            
            
            if (gripState >= 30) & (grip == 0):
                rospy.loginfo('Closing!')
                time.sleep(1.0)       
                result = gripper_client([5500.0,5500.0])
                rospy.loginfo(result)
                grip = 1
                time.sleep(5.0)
            elif (gripState <= 0) & (grip == 1):
                rospy.loginfo('Opening!')       
                time.sleep(1.0)
                grip = 0
                result = gripper_client([200.0,200.0])
                rospy.loginfo(result)
                home()
                #time.sleep(4.0)
            else:
                for pos, orient in poses:
                    if pos != [offset,0.0,0.0]:
                        result = cartesian_pose_client(pos, orient)
                            

            #rospy.loginfo(result)
            rate.sleep()


        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
