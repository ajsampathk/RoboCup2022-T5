#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist, Quaternion 
from tf.transformations import quaternion_from_euler
from math import atan2,sqrt
import random


# def kickToGoal(team):
#     ball_current_state.twist=Twist()
#     ball_current_state.twist.linear.x = vx
#     ball_current_state.twist.linear.z = vz
#     rospy.loginfo(ball_current_state)
#     state_pub.publish(ball_current_state)
class ball():
    def __init__(self):
        self.ball_current_state = ModelState()
        self.ball_current_state.model_name = "foot_ball"
        rospy.Subscriber('/gazebo/model_states',ModelStates, self.ball_state)
        self.state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)

    def ball_state(self,msg):
        ball_idx = list(msg.name).index("foot_ball")
        self.ball_current_state.pose = msg.pose[ball_idx]
        self.ball_current_state.twist = msg.twist[ball_idx]
    
    def kick(self, x, y, force):
        self.ball_current_state.twist = Twist()
        self.ball_current_state.twist.linear.x = x*force
        self.ball_current_state.twist.linear.y = y*force
        # rospy.loginfo(self.ball_current_state)
        self.state_pub.publish(self.ball_current_state)


class robot():
        
    def __init__(self,name):
        self.sub_model_name=name
        self.pub_model_name=name+"::nao_body"
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.nao_state)
        self.state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
        self.current_state=ModelState()
        self.all_robots = ["blueA","blueG","blueDR","blueDL","yellowA","yellowG","yellowDR","yellowDL"]
        self.all_robots.remove(name)
        self.all_states = []
        for i in self.all_robots:
            state=ModelState()
            state.model_name=i
            self.all_states.append(state)

    def nao_state(self,msg):

        nao_idx = list(msg.name).index(self.sub_model_name)
        self.current_state.pose = msg.pose[nao_idx]
        for i in range(len(self.all_robots)):
            self.all_states[i].pose = msg.pose[list(msg.name).index(self.all_robots[i])]

    def followBall(self, ball,speed=1,x_offset=0,y_offset=0):
        # print(ball.ball_current_state.pose)
        self.move(ball.ball_current_state.pose.position.x+x_offset, ball.ball_current_state.pose.position.y+y_offset,speed)

    def moveKeeper(self, ball):
        if(self.current_state.pose.position.y > 0.013968 and self.current_state.pose.position.y < 1.799283):
            self.move(self.current_state.pose.position.x, ball.ball_current_state.pose.position.y)


    def move(self,x,y,speed=1):
        updated_state = ModelState()
        empty=Twist()
        if not (abs(self.current_state.pose.position.y-y) <0.05 and abs(self.current_state.pose.position.x-x) <0.05):
            updated_state.pose=self.current_state.pose
            updated_state.pose.orientation = Quaternion()

            dy=(self.current_state.pose.position.y-y)
            dx=(self.current_state.pose.position.x-x)

            q=quaternion_from_euler(0,0,atan2(dy,dx)-1.5707)
            updated_state.pose.orientation.x=q[0]
            updated_state.pose.orientation.y=q[1]
            updated_state.pose.orientation.z=q[2]
            updated_state.pose.orientation.w=q[3]



            updated_state.model_name = self.pub_model_name
            updated_state.twist=empty
            updated_state.twist.linear.y =(0.8*(abs(dy)/(abs(dx)+abs(dy)))*cmp(y-self.current_state.pose.position.y,0))
            updated_state.twist.linear.x =(0.8*(abs(dx)/(abs(dx)+abs(dy)))*cmp(x-self.current_state.pose.position.x,0))



            collission_flag=False
            collided_bot = None
            for i in self.all_states:
                if(abs(i.pose.position.x-updated_state.pose.position.x)<0.3 and abs(i.pose.position.y-updated_state.pose.position.y)<0.3):
                    collission_flag=True
                    collided_bot=i
            
            if collission_flag:
                dy=random.uniform(0,1)*(self.current_state.pose.position.y-collided_bot.pose.position.y)
                dx=random.uniform(0,1)*(self.current_state.pose.position.x-collided_bot.pose.position.x)
                updated_state.twist.linear.y =-(0.8*(abs(dy)/(abs(dx)+abs(dy)))*cmp(collided_bot.pose.position.y-self.current_state.pose.position.y,0))
                updated_state.twist.linear.x =-(0.8*(abs(dx)/(abs(dx)+abs(dy)))*cmp(collided_bot.pose.position.x-self.current_state.pose.position.x,0))

            updated_state.twist.linear.x = speed*(updated_state.twist.linear.x)/sqrt(updated_state.twist.linear.x**2+updated_state.twist.linear.y**2)
            updated_state.twist.linear.y = speed*(updated_state.twist.linear.y)/sqrt(updated_state.twist.linear.x**2+updated_state.twist.linear.y**2)


            # rospy.loginfo(updated_state)
            self.state_pub.publish(updated_state)

if __name__ == '__main__':

    rospy.init_node('Simple_controller')

    state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
    
    rospy.spin()		