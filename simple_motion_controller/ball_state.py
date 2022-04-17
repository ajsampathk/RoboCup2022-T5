#! /usr/bin/env python

from re import L
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState, ModelStates
from math import hypot

ball_current_state = ModelState()
ball_current_state.model_name = "foot_ball"


class ball_state():

    def __init__(self):
        self.opponent = 'O'
        self.free = 'free'
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.ball_position)
        self.pubBlue = rospy.Publisher('ball_state/blue', String, queue_size=1)
        self.pubYellow = rospy.Publisher('ball_state/yellow', String, queue_size=1)

    def ball_position(self, msg):
        global ball_current_state
        ball_idx = list(msg.name).index("foot_ball")
        ball_current_state.pose = msg.pose[ball_idx]
        self.ball_occupier()
    
    def ball_occupier(self):
        ball_x = ball_current_state.pose.position.x
        ball_y = ball_current_state.pose.position.y
        
        ball_status=["free", "with_oppo"]
        diffBlue=[]
        diffYellow=[]
        
        for i in range(0,4):
            diffBlue.append((hypot(teamBlue[i].current_state.pose.position.x - ball_x, teamBlue[i].current_state.pose.position.y - ball_y)))
            diffYellow.append((hypot(teamYellow[i].current_state.pose.position.x - ball_x, teamYellow[i].current_state.pose.position.y - ball_y)))

        # print(min(diffBlue))
        # print(min(diffYellow))

        # is no bot is within 0.3 of the ball, that means the ball is free
        # otherwise check which bot is closest to the ball and assign to it
        if ((min(diffBlue) > 0.3) and (min(diffYellow) > 0.3)):
            self.pubBlue.publish(ball_status[0])
            self.pubYellow.publish(ball_status[0])
        elif (min(diffBlue) < min(diffYellow)):
            # print(teamBlue[diffBlue.index(min(diffBlue))].current_state.model_name)
            self.pubBlue.publish(teamBlue[diffBlue.index(min(diffBlue))].current_state.model_name)
            self.pubYellow.publish(ball_status[1])
        else:
            # print(teamYellow[diffYellow.index(min(diffYellow))].current_state.model_name)
            self.pubYellow.publish(teamYellow[diffYellow.index(min(diffYellow))].current_state.model_name)
            self.pubBlue.publish(ball_status[1])



class robot():
    
    def __init__(self,name):
        self.sub_model_name=name
        self.pub_model_name=name+"::nao_body"
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.nao_state)
        self.current_state=ModelState()

    def nao_state(self,msg):
        nao_idx = list(msg.name).index(self.sub_model_name)
        self.current_state.model_name = self.sub_model_name
        self.current_state.pose = msg.pose[nao_idx]


if __name__ == '__main__':

    global teamBlue
    global teamYellow
    rospy.init_node('ball_states')

    blueA = robot('blueA')
    blueDL = robot('blueDL')
    blueDR = robot('blueDR')
    blueG = robot('blueG')
    yellowA = robot('yellowA')
    yellowDL = robot('yellowDL')
    yellowDR = robot('yellowDR')
    yellowG = robot('yellowG')

    teamBlue = [blueA, blueDL, blueDR, blueG]
    teamYellow = [yellowA, yellowDL, yellowDR, yellowG]

    ball = ball_state()

    #ball_current_state.twist=Twist()
    rospy.spin()
		
