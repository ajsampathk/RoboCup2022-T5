#!/usr/bin/env python
# Author: AJ
# Date: 08/02/2022 

from turtle import update
import rospy
import numpy as np
import math
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


HEIGHT_CONSTANT = 0.345
PUBLISH_RATE  = 50
STEP_CONSTANT  = 0.01
ROTATION_CONSTANT = 0.05
nao_current_state = ModelState()
nao_current_state.model_name = "nao11"
nao_current_state1 = ModelState()
nao_current_state1.model_name = "nao15"
ball_current_state = ModelState()
ball_current_state.model_name = "cricket_ball"
state_pub = None
rate = None

roll = pitch = yaw = 0.0
kp = 0.5

def nao_state(msg):
    global nao_current_state
    nao_idx = list(msg.name).index("nao11")
    nao_current_state.pose = msg.pose[nao_idx]
    nao_current_state.twist = msg.twist[nao_idx]
    orientation_q = nao_current_state.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # print(roll, pitch, yaw)
def nao_state1(msg):
    global nao_current_state1
    nao1_idx = list(msg.name).index("nao15")
    nao_current_state1.pose = msg.pose[nao1_idx]
    nao_current_state1.twist = msg.twist[nao1_idx]
    orientation_q = nao_current_state1.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # print(roll, pitch, yaw)

def ball_state(msg):
    global ball_current_state
    ball_idx = list(msg.name).index("cricket_ball")
    ball_current_state.pose = msg.pose[ball_idx]
    ball_current_state.twist = msg.twist[ball_idx]

# def get_rotation(msg):
#     global roll, pitch, yaw
#     orientation_q = msg.pose.pose.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
#     print(yaw)

def move_x(l):
    global nao_current_state,rate
    updated_state = ModelState()
    while not abs(nao_current_state.pose.position.x-l) <0.05:
        updated_state=nao_current_state
        updated_state.pose.position.x+=(STEP_CONSTANT*cmp(l-nao_current_state.pose.position.x,0))
        rospy.loginfo(updated_state.pose.position.x)
        state_pub.publish(updated_state)
        rate.sleep()

def move_y(l):
    global nao_current_state,rate
    updated_state = ModelState()
    while not abs(nao_current_state.pose.position.y-l) <0.05:
        updated_state=nao_current_state
        updated_state.pose.position.y+=(STEP_CONSTANT*cmp(l-nao_current_state.pose.position.y,0))
        rospy.loginfo(updated_state.pose.position.y)
        state_pub.publish(updated_state)
        rate.sleep()

def rotate(radian):
    global nao_current_state, rate
    updated_state = ModelState()
    radian = math.pi - radian 
    # while (abs(roll - radian) > 0.05):
    updated_state = nao_current_state
    # if(radian < 0):
    #     radian = radian + math.pi
    quat = quaternion_from_euler(radian,pitch,yaw)
    #  - 1.5708
    #  + 1.5708
    print(quat)
    updated_state.pose.orientation.w = quat[0]
    updated_state.pose.orientation.x = quat[1]
    updated_state.pose.orientation.y = quat[2]
    updated_state.pose.orientation.z = quat[3]

    state_pub.publish(updated_state)

    return True

def move_interpolate(x2,y2):
    global nao_current_state,rate
    updated_state = ModelState()
    current_x = nao_current_state.pose.position.x
    current_y = nao_current_state.pose.position.y
    radians = math.atan2(y2-current_y, x2-current_x)
    print(radians)
    if(rotate(radians)):
        diff_x = abs(current_x-x2)
        diff_y = abs(current_y-y2)
        greater = (diff_x if diff_x >= diff_y else diff_y)
        number_increments = abs(int(greater/STEP_CONSTANT)) #number of increments
        x_values = np.linspace(current_x, x2, number_increments)
        y_values = np.linspace(current_y, y2, number_increments)
        values1, values2 = 1,1
        #while not ((abs(current_state.pose.position.y - y2) <0.05) and (abs(current_state.pose.position.x - x2) < 0.05)):
        for a in range(number_increments):
            updated_state = nao_current_state
            updated_state.pose.position.x = x_values[a]
            updated_state.pose.position.y = y_values[a]
            # rospy.loginfo(updated_state.pose.position.x)
            state_pub.publish(updated_state)
            rate.sleep()
def move_interpolate2(x2,y2):
    global nao_current_state1,rate
    updated_state = ModelState()
    current_x = nao_current_state1.pose.position.x
    current_y = nao_current_state1.pose.position.y
    radians = math.atan2(y2-current_y, x2-current_x)
    print(radians)
    if(rotate(radians)):
        diff_x = abs(current_x-x2)
        diff_y = abs(current_y-y2)
        greater = (diff_x if diff_x >= diff_y else diff_y)
        number_increments = abs(int(greater/STEP_CONSTANT)) #number of increments
        x_values = np.linspace(current_x, x2, number_increments)
        y_values = np.linspace(current_y, y2, number_increments)
        values1, values2 = 1,1
        #while not ((abs(current_state.pose.position.y - y2) <0.05) and (abs(current_state.pose.position.x - x2) < 0.05)):
        for a in range(number_increments):
            updated_state = nao_current_state1
            updated_state.pose.position.x = x_values[a]
            updated_state.pose.position.y = y_values[a]
            # rospy.loginfo(updated_state.pose.position.x)
            state_pub.publish(updated_state)
            rate.sleep()
    
if __name__ == '__main__':

    rospy.init_node('Simple_controller')
    rospy.Subscriber('/gazebo/model_states',ModelStates,nao_state)
    rospy.Subscriber('/gazebo/model_states',ModelStates,nao_state1)
    rospy.Subscriber('/gazebo/model_states',ModelStates,ball_state)
    #rospy.Subscriber('/odom', Odometry, get_rotation)
    rate = rospy.Rate(PUBLISH_RATE)
    state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
   
   
    
    while not rospy.is_shutdown():

        p = ball_current_state.pose.position.x
        r = ball_current_state.pose.position.y
        # if y == 0:
        #     move_x(x)
        # elif x == 0:
        #     move_y(y)
        # else:
        move_interpolate(p-0.1,r-0.1)
 	move_interpolate2(p,r)
	
	
	

