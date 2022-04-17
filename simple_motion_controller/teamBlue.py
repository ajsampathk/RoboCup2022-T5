#! /usr/bin/env python

from cmath import atan
from turtle import right
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist, Quaternion 
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, sin, cos
import move_api
import random

ball_occupier = ""
goal_x = -3.95
goal_left_pole = 1.946
goal_right_pole = 0.0305

def ball_state(msg):
    global ball_occupier
    ball_occupier = msg.data

# checks if another bot is in vision
def robot_inVision(ball, bot1, bot2):
    orientation_list = [bot1.current_state.pose.orientation.x, bot1.current_state.pose.orientation.y, bot1.current_state.pose.orientation.z, bot1.current_state.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    bot_position = bot1.current_state.pose.position.x
    yaw1 = yaw - 0.523599
    yaw2 = yaw + 0.523599
    if yaw1 < 0:
        yaw1 = yaw1 + 6.283
    if yaw2 < 0:
        yaw2 = yaw2 + 6.283
    
    bot_x = bot1.current_state.pose.position.x - bot2.current_state.pose.position.x
    bot_y = bot1.current_state.pose.position.y - bot2.current_state.pose.position.y

    angle_to_bot = atan2(bot_y, bot_x) - 3.14

    if angle_to_bot < 0:
        angle_to_bot = angle_to_bot + 6.283
    
    if angle_to_bot > yaw1 and angle_to_bot < yaw2:
        print("passing")
        x = cos(angle_to_bot)
        y = sin(angle_to_bot)
        ball.kick(x, y, 1.5)
        return True
    else:
        return False

# dribble and shooting
def kickToGoal(ball, bot):
    orientation_list = [bot.current_state.pose.orientation.x, bot.current_state.pose.orientation.y, bot.current_state.pose.orientation.z, bot.current_state.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    bot_position = bot.current_state.pose.position.x
    max_before_attempt = -1.267 # global coordinate of max distance before attempting the goal
    # viewing angle of the bot, which is 60 degrees
    yaw1 = yaw - 0.523599
    yaw2 = yaw + 0.523599

    # left pole and right pole angle calculate, left and right is as per the yellow team point of view. 
    gpx = bot.current_state.pose.position.x - goal_x
    glpy = bot.current_state.pose.position.y - goal_left_pole
    grpy = bot.current_state.pose.position.y - goal_right_pole

    left_post_angle = atan2(glpy, gpx) - 3.14
    right_post_angle = atan2(grpy, gpx) - 3.14

    if left_post_angle < 0:
        left_post_angle = left_post_angle + 6.283
    if right_post_angle < 0:
        right_post_angle = right_post_angle + 6.283
    if yaw1 < 0:
        yaw1 = yaw1 + 6.283
    if yaw2 < 0:
        yaw2 = yaw2 + 6.283
    
    #shoot randomly keeper
    if (ball_occupier == "blueG"):
        y_shoot = random.uniform(min(left_post_angle, right_post_angle), max(left_post_angle,right_post_angle))
        x = cos(y_shoot)
        y = sin(y_shoot)
        ball.kick(x, y, 3)
        return

    # if the goal is in view or close to the goal that you can't see the post anymore
    if (left_post_angle > yaw1 and right_post_angle < yaw2) or (left_post_angle < yaw1 and right_post_angle > yaw2):
        y_shoot = random.uniform(min(left_post_angle, right_post_angle), max(left_post_angle,right_post_angle))
        # shoot for goal if within the distance
        x = cos(y_shoot)
        y = sin(y_shoot)
        if bot_position <= max_before_attempt:
            ball.kick(x, y, 3)
        else:
            # otherwise dribble the ball to the goal
            ball.kick(x, y, 2)
        # if left post is visible, then shoot the ball between the left post and yaw2
    elif (right_post_angle > yaw2 and left_post_angle < yaw2 and left_post_angle > yaw1):
        y_random = random.uniform(min(left_post_angle, yaw2), max(left_post_angle, yaw2))
        x = cos(y_random)
        y = sin(y_random)
        if bot_position <= max_before_attempt:
            ball.kick(x,y, 3)
        else: 
            ball.kick(x,y, 2)
        # if the right post is visible, then shoot the ball betweem the right post and yaw1
    elif (left_post_angle < yaw1 and right_post_angle > yaw1 and right_post_angle < yaw2):
        y_random = random.uniform(min(right_post_angle, yaw1),max(right_post_angle, yaw1))
        x = cos(y_random)
        y = sin(y_random)
        if bot_position <= max_before_attempt:
            ball.kick(x,y, 3)
        else: 
            ball.kick(x,y, 2)
        # if the robot is facing left and goal is not visible then, dribble to the right
    elif left_post_angle < yaw1 and right_post_angle < yaw2: # facing left, goal not in range
        x =  cos(yaw1)
        y =  sin(yaw1)
        ball.kick(x, y, 1)
        # if the robot is facing left and goal is not visible then, dribble to the left
    elif left_post_angle > yaw1 and right_post_angle > yaw2: # facing right, goal not in range
        x =  cos(yaw2)
        y =  sin(yaw2)
        ball.kick(x, y, 1)
    

if __name__ == '__main__':

    rospy.init_node('Team_Blue')
    rospy.Subscriber('ball_state/blue', String, ball_state)
    rate = rospy.Rate(10)
    attacker = move_api.robot("blueA")
    defenderL = move_api.robot("blueDL")
    defenderR = move_api.robot("blueDR")
    goalkeeper = move_api.robot("blueG")
    ball = move_api.ball()

    rospy.wait_for_message('ball_state/blue', String)

    while not rospy.is_shutdown():
        ball_position_x = ball.ball_current_state.pose.position.x
        if((ball_occupier == "free" or ball_occupier == "with_oppo") and ball_position_x < 0.5):
            attacker.followBall(ball)
            defenderL.followBall(ball)
            defenderR.move(2,2)
        elif((ball_occupier == "free" or ball_occupier == "with_oppo") and ball_position_x > 0.5):
            defenderL.followBall(ball)
            defenderR.followBall(ball)
            attacker.move(0.5,0)
            goalkeeper.moveKeeper(ball)

        if ball_occupier == "blueA":
            kickToGoal(ball, attacker)
        elif ball_occupier == "blueDL":
            if not robot_inVision(ball, defenderL, attacker):
                kickToGoal(ball, defenderL)
        elif ball_occupier == "blueDR":
            if not robot_inVision(ball, defenderR, attacker):
                if not robot_inVision(ball, defenderR, defenderL):
                    kickToGoal(ball, defenderR)
        elif ball_occupier == "blueG":
            kickToGoal(ball, goalkeeper)
        rate.sleep()
        