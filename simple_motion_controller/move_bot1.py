#!/usr/bin/env python
# Author: AJ
# Date: 08/02/2022 

import rospy
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point


HEIGHT_CONSTANT = 0.345
PUBLISH_RATE  = 50
STEP_CONSTANT  = 0.01
current_state = ModelState()
current_state.model_name = "nao11"
current_state1 = ModelState()
current_state1.model_name = "cricket_ball"
current_state2 = ModelState()
current_state2.model_name = "RoboCup 3D Simulator Goal"
state_pub = None
rate = None


def nao_state(msg):
    global current_state
    nao_idx = list(msg.name).index("nao11")
    current_state.pose = msg.pose[nao_idx]
    current_state.twist = msg.twist[nao_idx]

def ball_state(msg):
    global current_state1
    ball_idx = list(msg.name).index("cricket_ball")
    current_state1.pose = msg.pose[ball_idx]
    current_state1.twist = msg.twist[ball_idx]
def goal_state(msg):
    global current_state2
    goal_idx = list(msg.name).index("RoboCup 3D Simulator Goal")
    current_state2.pose = msg.pose[goal_idx]
    current_state2.twist = msg.twist[goal_idx]

def move_x(l):
    global current_state,rate
    updated_state = ModelState()
    while not abs(current_state.pose.position.x-l) <0.01:
        updated_state=current_state
        updated_state.pose.position.x+=(STEP_CONSTANT*cmp(l-current_state.pose.position.x,0))
        rospy.loginfo(updated_state.pose.position.x)
        state_pub.publish(updated_state)
        rate.sleep()

def move_x1(l):
    global current_state1,rate
    updated_state1 = ModelState()
    while not abs(current_state1.pose.position.x-l) <0.01:
        updated_state1=current_state1
        updated_state1.pose.position.x+=(STEP_CONSTANT*cmp(l-current_state1.pose.position.x,0))
        rospy.loginfo(updated_state1.pose.position.x)
        state_pub.publish(updated_state1)
        rate.sleep()

def move_y(l):
    global current_state,rate
    updated_state = ModelState()
    while not abs(current_state.pose.position.y-l) <0.01:
        updated_state=current_state
        updated_state.pose.position.y+=(STEP_CONSTANT*cmp(l-current_state.pose.position.y,0))
        rospy.loginfo(updated_state.pose.position.y)
        state_pub.publish(updated_state)
        rate.sleep()

def move_y1(l):
    global current_state1,rate
    updated_state1 = ModelState()
    while not abs(current_state1.pose.position.y-l) <0.01:
        updated_state1=current_state1
        updated_state1.pose.position.y+=(STEP_CONSTANT*cmp(l-current_state1.pose.position.y,0))
        rospy.loginfo(updated_state1.pose.position.y)
        state_pub.publish(updated_state1)
        rate.sleep()
def move_interpolate(x2,y2):
    global current_state,rate
    updated_state = ModelState()
    x = current_state.pose.position.x
    y = current_state.pose.position.y
    greater = (x2 if (abs(x)-abs(x-x2) > abs(y)-abs(y-y2)) else y2)
    number_increments = int(greater/0.01) #number of increments
    x_values = np.linspace(x,x+x2,number_increments)
    y_values = np.linspace(y,y+y2,number_increments)
    values1, values2 = 1,1
    #while not ((abs(current_state.pose.position.y - y2) <0.05) and (abs(current_state.pose.position.x - x2) < 0.05)):
    for a in range(number_increments):
        updated_state = current_state
        updated_state.pose.position.x = x_values[a]
        updated_state.pose.position.y = y_values[a]
        rospy.loginfo(updated_state.pose.position.x)
        state_pub.publish(updated_state)
        rate.sleep()
    
if __name__ == '__main__':

    rospy.init_node('Simple_controller')
    rospy.Subscriber('/gazebo/model_states',ModelStates,nao_state)
    rospy.Subscriber('/gazebo/model_states',ModelStates,ball_state)
    rospy.Subscriber('/gazebo/model_states',ModelStates,goal_state)	
    rate = rospy.Rate(PUBLISH_RATE)
    state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)

    while(1):

	
        x =current_state1.pose.position.x
        y = current_state1.pose.position.y
  	x1 =current_state.pose.position.x
        y1 = current_state.pose.position.y
	x2 =current_state2.pose.position.x
        y2 = current_state2.pose.position.y
	print(x2)
	print(y)
	print(abs(x-x1))
	if abs(x-x1) > 0.01 and abs(y-y1) > 0.01:
		move_x(x)
		move_y(y)
	elif abs(x-x1) < 0.01 and abs(y-y1) < 0.01:
		move_x1(x2)
		move_y1(y2)
		


		 	






