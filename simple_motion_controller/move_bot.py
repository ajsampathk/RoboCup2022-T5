#! /usr/bin/env python


# Author: AJ
# Date: 08/02/2022 

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist, Quaternion 
from tf.transformations import quaternion_from_euler
from math import atan2

ball_current_state = ModelState()
ball_current_state.model_name = "cricket_ball"

def ball_state(msg):
    global ball_current_state
    ball_idx = list(msg.name).index("cricket_ball")
    ball_current_state.pose = msg.pose[ball_idx]
    ball_current_state.twist = msg.twist[ball_idx]



class robot():
    
    def __init__(self,name):
        self.sub_model_name=name
        self.pub_model_name=name+"::nao_body"
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.nao_state)
        self.state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
        self.current_state=ModelState()

    def nao_state(self,msg):

        nao_idx = list(msg.name).index(self.sub_model_name)
        self.current_state.pose = msg.pose[nao_idx]

    def move(self,x,y):
        updated_state = ModelState()
        empty=Twist()
        if not (abs(self.current_state.pose.position.y-y) <0.05 and abs(self.current_state.pose.position.x-x) <0.05 ):
            updated_state.pose=self.current_state.pose
            updated_state.pose.orientation = Quaternion()

            dy=self.current_state.pose.position.y-y
            dx=self.current_state.pose.position.x-x

            q=quaternion_from_euler(0,0,atan2(dy,dx)-1.5707)
            updated_state.pose.orientation.x=q[0]
            updated_state.pose.orientation.y=q[1]
            updated_state.pose.orientation.z=q[2]
            updated_state.pose.orientation.w=q[3]

            updated_state.model_name = self.pub_model_name
            updated_state.twist=empty
            updated_state.twist.linear.y =(2*(abs(dy)/(abs(dx)+abs(dy)))*cmp(y-self.current_state.pose.position.y,0))
            updated_state.twist.linear.x =(2*(abs(dx)/(abs(dx)+abs(dy)))*cmp(x-self.current_state.pose.position.x,0))

            rospy.loginfo(updated_state)
            self.state_pub.publish(updated_state)

if __name__ == '__main__':

    rospy.init_node('Simple_controller')
    rospy.Subscriber('/gazebo/model_states',ModelStates,ball_state)
    # rospy.Subscriber('/gazebo/model_states',ModelStates,nao_state)
    # rate = rospy.Rate(PUBLISH_RATE)
    # state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
    nao11 =  robot("nao11")
    nao15 = robot("nao15")
    
    while not rospy.is_shutdown():
	p = ball_current_state.pose.position.x
        r = ball_current_state.pose.position.y
        nao11.move(p,r)
        nao15.move(p,r)
	if (nao11.current_state.pose.position.x - p < 0.2) and (nao11.current_state.pose.position.y - r < 0.2):
		nao11.move(4,1)


