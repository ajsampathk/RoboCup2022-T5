#! /usr/bin/env python


# Author: AJ
# Date: 08/02/2022 

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point


HEIGHT_CONSTANT = 0.345
PUBLISH_RATE  = 10
STEP_CONSTANT  = 0.05
current_state = ModelState()
current_state.model_name = "nao_v40"
state_pub = None
rate = rospy.Rate(PUBLISH_RATE)


def nao_state(msg):
    global current_state
    nao_idx = list(msg.name).index("nao_v40")
    current_state.pose = msg.pose[nao_idx]
    current_state.twist = msg.twist[nao_idx]

def move_x(l):
    global current_state
    updated_state = ModelState()
    while not abs(current_state.pose.position.x-l) <0.05:
        updated_state=current_state
        updated_state.pose.position.x+=(STEP_CONSTANT*cmp(l-current_state.pose.position.x,0))
        rospy.loginfo(updated_state.pose.position.x)
        state_pub.publish(updated_state)
        rate.sleep()
    
if __name__ == '__main__':

    rospy.init_node('Simple_controller')
    rospy.Subscriber('/gazebo/model_states',ModelStates,nao_state)

    state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)

    while not rospy.is_shutdown():
        x = float(input('X='))
        move_x(x)