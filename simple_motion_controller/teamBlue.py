#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist, Quaternion 
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from math import atan2
import move_api

ball_occupier = ""

def ball_state(msg):
    global ball_occupier
    ball_occupier = msg.data

if __name__ == '__main__':

    rospy.init_node('Team_Blue')
    rospy.Subscriber('chatter/blue', String, ball_state)
    
    attacher = move_api.robot("blueA")
    defenderL = move_api.robot("blueDL")
    defenderR = move_api.robot("blueDR")
    goalkeeper = move_api.robot("blueG")
    ball = move_api.ball()

    rospy.wait_for_message('chatter/blue', String)

    while not rospy.is_shutdown():
        ball_position_x = ball.ball_current_state.pose.position.x
        print(ball_occupier)
        if(ball_occupier == "free" and ball_position_x < 0.5):
            attacher.followBall(ball)
            defenderL.followBall(ball)
            # print("here")
        elif(ball_occupier == "free" and ball_position_x > 0.5):
            defenderL.followBall(ball)
            defenderR.followBall(ball)
            attacher.move(0.5,0)
            goalkeeper.moveKeeper(ball)
        
        if(ball_position_x > 0):
            goalkeeper.moveKeeper()
        
        # if(ball_occupier == "attacker"):
        #     move_api.kickToGoal("blue")
        #     move_api.passTheBall()
        # elif(ball_occupier == "defenderL"):
        #     move_api.kickToGoal("blue")
        #     move_api.passTheBall()
        # elif(ball_occupier == "defenderR"):
        #     move_api.passTheBall()
        # elif(ball_occupier == "goalkeeper"):
        #     move_api.passTheBall()
        #     move_api.kickToGoal()
        




        


    
        


