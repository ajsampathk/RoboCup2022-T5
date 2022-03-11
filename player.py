#!/usr/bin/env python
from mimetypes import init
from turtle import update
import rospy
import numpy as np
import math
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Player:
    
    HEIGHT_CONSTANT = 0.345
    PUBLISH_RATE  = 50
    STEP_CONSTANT  = 0.01
    ROTATION_CONSTANT = 0.05
    ball_current_state = ModelState()
    ball_current_state.model_name = "cricket_ball"
    state_pub = None
    rate = None

    def __init__(self, bot_name):
        print("here")
        self.nao_current_state = ModelState()
        self.nao_current_state.model_name = bot_name
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.kp = 0.5
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.nao_state)
        self.rate = rospy.Rate(self.PUBLISH_RATE)
        self.state_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
        print("here")

    def nao_state(self, msg):
        nao_idx = list(msg.name).index(self.nao_current_state.model_name)
        self.nao_current_state.pose = msg.pose[nao_idx]
        self.nao_current_state.twist = msg.twist[nao_idx]

    def move_x(self, l):
        update_state = ModelState()
        update_state = self.nao_current_state
        update_state.twist.linear.x = l
        update_state.model_name =  "nao11::nao_body"
        self.state_pub.publish(update_state)
        self.rate.sleep()
        # while not abs(self.nao_current_state.pose.position.x-l) < 0.05:
        #     update_state = self.nao_current_state
        #     update_state.pose.position.x += (self.STEP_CONSTANT * cmp(l-self.nao_current_state.pose.position.x,0))
        #     rospy.loginfo(update_state.pose.position.x)
        #     self.state_pub.publish(update_state)
        #     self.rate.sleep()

    

