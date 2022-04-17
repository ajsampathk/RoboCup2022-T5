#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates

def callback(msg):
    rospy.loginfo("Blue:0|Yellow:0")

if __name__=='__main__':
    rospy.init_node('game_monitor')
    rospy.Subscriber('/gazebo/model_states',ModelStates,callback)
    while not rospy.is_shutdown():
        rospy.spin()
